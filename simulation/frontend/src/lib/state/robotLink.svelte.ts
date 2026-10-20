/// <reference types="w3c-web-serial" />
// Live link to the real robot, straight from the browser: USB (Web Serial) or the HM-10 BLE module
// (Web Bluetooth). The firmware's Telemetry class (robot/src/hardware/comms/Telemetry.h) streams
// newline-delimited JSON on both: "hello" when the link comes up, "state" frames, and "log" lines.
// Any line that is not a JSON object is kept as a raw log line.
//
// Units are the firmware frame: cm, x right, y toward the opponent goal, angles in radians
// clockwise from +y (compass heading/offset in degrees).

const HM10_SERVICE = 0xffe0;
const HM10_DATA = 0xffe1;
const MAX_LOGS = 500;
const MAX_RECORD = 200_000;
const MAX_PARTIAL = 8192;

export type Transport = 'usb' | 'bluetooth';

export interface HelloMsg {
	t: 'hello';
	v: number;
	board: string;
	ms: number;
}

export interface BallMsg {
	seen: boolean;
	angle: number;
	dist: number;
	x: number;
	y: number;
	conf: number;
}

export interface PoseMsg {
	x: number;
	y: number;
	theta: number;
	vx: number;
	vy: number;
	omega: number;
}

export interface StateMsg {
	t: 'state';
	ms: number;
	seq: number;
	heading?: number;
	offset?: number;
	enc?: number[];
	sonar?: (number | null)[];
	ball?: BallMsg;
	ir?: number[];
	line?: number[];
	pose?: PoseMsg;
	drop?: number;
}

export interface LogMsg {
	t: 'log';
	ms: number;
	m: string;
}

export interface RawMsg {
	t: 'raw';
	m: string;
}

export type RobotMsg = HelloMsg | StateMsg | LogMsg;

export interface LogLine {
	id: number;
	ms: number | null; // robot uptime; null for raw lines
	text: string;
	raw: boolean;
}

/** Reassembles a chunked byte stream (BLE packets are ≤ 20 bytes) into complete lines. */
export class LineSplitter {
	private partial = '';

	push(chunk: string): string[] {
		const parts = (this.partial + chunk).split('\n');
		this.partial = parts.pop() ?? '';
		if (this.partial.length > MAX_PARTIAL) this.partial = '';
		return parts.map((l) => l.replace(/\r$/, '')).filter((l) => l.length > 0);
	}

	reset() {
		this.partial = '';
	}
}

/** One line from the robot → a message, or a raw text line. `null` for a malformed JSON object. */
export function parseLine(line: string): RobotMsg | RawMsg | null {
	if (!line.startsWith('{')) return { t: 'raw', m: line };
	try {
		const msg = JSON.parse(line);
		if (msg && (msg.t === 'hello' || msg.t === 'state' || msg.t === 'log')) return msg as RobotMsg;
	} catch {
		// fall through
	}
	return null;
}

/** Firmware frame (cm, clockwise from +y) → SoccerField space (mm, +Y = enemy goal, π/2 = facing it). */
export function fwToField(x: number, y: number, theta = 0): { x: number; y: number; theta: number } {
	return { x: x * 10, y: y * 10, theta: Math.PI / 2 - theta };
}

/** Ball in the robot frame (cm, x right, y forward) → field frame (cm), given the robot pose. */
export function ballInField(robot: { x: number; y: number; theta: number }, ball: { x: number; y: number }) {
	const c = Math.cos(robot.theta);
	const s = Math.sin(robot.theta);
	return { x: robot.x + ball.x * c + ball.y * s, y: robot.y - ball.x * s + ball.y * c };
}

class RobotLink {
	readonly serialSupported = typeof navigator !== 'undefined' && 'serial' in navigator;
	readonly bluetoothSupported = typeof navigator !== 'undefined' && 'bluetooth' in navigator;

	transport = $state<Transport | null>(null);
	connected = $state(false);
	connecting = $state(false);
	error = $state<string | null>(null);

	hello = $state.raw<HelloMsg | null>(null);
	state = $state.raw<StateMsg | null>(null);
	logs = $state.raw<LogLine[]>([]);

	msgRate = $state(0); // messages per second
	parseErrors = $state(0);
	missedFrames = $state(0); // gaps in "seq"
	recorded = $state(0);

	private record: string[] = [];
	private splitter = new LineSplitter();
	private decoder = new TextDecoder();
	private lastSeq: number | null = null;
	private nextLogId = 0;
	private rateCount = 0;
	private rateTimer: ReturnType<typeof setInterval> | null = null;

	private port: SerialPort | null = null;
	private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
	private device: BluetoothDevice | null = null;

	async connectSerial() {
		if (!this.serialSupported) return;
		await this.disconnect();
		this.begin('usb');
		try {
			const port = await navigator.serial.requestPort();
			await port.open({ baudRate: 115200 });
			// The firmware only streams while DTR is set (UsbSerialPort::connected()).
			await port.setSignals({ dataTerminalReady: true, requestToSend: true });
			this.port = port;
			this.up();
			void this.readSerial(port);
		} catch (e) {
			this.fail(e);
		}
	}

	async connectBluetooth() {
		if (!this.bluetoothSupported) return;
		await this.disconnect();
		this.begin('bluetooth');
		try {
			const device = await navigator.bluetooth.requestDevice({
				filters: [{ services: [HM10_SERVICE] }, { namePrefix: 'Bucky' }, { namePrefix: 'HMSoft' }],
				optionalServices: [HM10_SERVICE]
			});
			device.addEventListener('gattserverdisconnected', this.onBluetoothLost);
			this.device = device;
			const server = await device.gatt!.connect();
			const service = await server.getPrimaryService(HM10_SERVICE);
			const data = await service.getCharacteristic(HM10_DATA);
			data.addEventListener('characteristicvaluechanged', (e) => {
				const value = (e.target as BluetoothRemoteGATTCharacteristic).value;
				if (value) this.feed(this.decoder.decode(value, { stream: true }));
			});
			await data.startNotifications();
			this.up();
		} catch (e) {
			this.fail(e);
		}
	}

	async disconnect() {
		const reader = this.reader;
		const device = this.device;
		this.reader = null;
		this.device = null;
		if (reader) await reader.cancel().catch(() => {}); // readSerial() closes the port
		if (device) {
			device.removeEventListener('gattserverdisconnected', this.onBluetoothLost);
			device.gatt?.disconnect();
		}
		this.down();
	}

	clearLogs() {
		this.logs = [];
	}

	/** Every message received this session, one JSON object per line, stamped with browser time. */
	downloadJsonl() {
		const blob = new Blob([this.record.join('\n') + '\n'], { type: 'application/x-ndjson' });
		const url = URL.createObjectURL(blob);
		const a = document.createElement('a');
		a.href = url;
		a.download = `bucky-${new Date().toISOString().replace(/[:.]/g, '-')}.jsonl`;
		a.click();
		URL.revokeObjectURL(url);
	}

	private async readSerial(port: SerialPort) {
		const reader = port.readable!.getReader();
		this.reader = reader;
		try {
			while (true) {
				const { value, done } = await reader.read();
				if (done) break;
				if (value) this.feed(this.decoder.decode(value, { stream: true }));
			}
		} catch (e) {
			this.error = errorText(e); // unplugged, or the robot reset
		} finally {
			reader.releaseLock();
			await port.close().catch(() => {});
			if (this.port === port) {
				this.port = null;
				this.down();
			}
		}
	}

	private onBluetoothLost = () => {
		this.device = null;
		this.error = 'Bluetooth link lost';
		this.down();
	};

	private begin(transport: Transport) {
		this.transport = transport;
		this.connecting = true;
		this.error = null;
		this.hello = null;
		this.state = null;
		this.lastSeq = null;
		this.missedFrames = 0;
		this.parseErrors = 0;
		this.splitter.reset();
		this.decoder = new TextDecoder();
	}

	private up() {
		this.connecting = false;
		this.connected = true;
		this.rateTimer ??= setInterval(() => {
			this.msgRate = this.rateCount;
			this.rateCount = 0;
		}, 1000);
	}

	private down() {
		this.connecting = false;
		this.connected = false;
		if (this.rateTimer) clearInterval(this.rateTimer);
		this.rateTimer = null;
		this.msgRate = 0;
	}

	private fail(e: unknown) {
		// Closing the chooser without picking a device is not an error worth showing.
		this.error = e instanceof DOMException && e.name === 'NotFoundError' ? null : errorText(e);
		void this.disconnect();
	}

	private feed(chunk: string) {
		const fresh: LogLine[] = [];
		for (const line of this.splitter.push(chunk)) {
			const msg = parseLine(line);
			if (msg === null) {
				this.parseErrors++;
				fresh.push(this.logLine(null, line, true));
				continue;
			}
			this.rateCount++;
			if (this.record.length < MAX_RECORD) {
				this.record.push(JSON.stringify({ at: Date.now(), ...msg }));
				this.recorded = this.record.length;
			}
			switch (msg.t) {
				case 'hello':
					this.hello = msg;
					this.lastSeq = null;
					break;
				case 'state':
					if (this.lastSeq !== null && msg.seq > this.lastSeq + 1) this.missedFrames += msg.seq - this.lastSeq - 1;
					this.lastSeq = msg.seq;
					this.state = msg;
					break;
				case 'log':
					fresh.push(this.logLine(msg.ms, msg.m, false));
					break;
				case 'raw':
					fresh.push(this.logLine(null, msg.m, true));
					break;
			}
		}
		if (fresh.length > 0) this.logs = [...this.logs, ...fresh].slice(-MAX_LOGS);
	}

	private logLine(ms: number | null, text: string, raw: boolean): LogLine {
		return { id: this.nextLogId++, ms, text, raw };
	}
}

function errorText(e: unknown): string {
	return e instanceof Error ? e.message : String(e);
}

export const robotLink = new RobotLink();
export default robotLink;
