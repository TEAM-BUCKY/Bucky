import { track } from "$state/utils";

export const SERVICE_UUID = "33c7afab-1609-4a7e-861d-9cfbefb33541";

export function encodeStringToUUID(str: string) {
    const hex = str
        .split("")
        .map((c) => c.charCodeAt(0).toString(16).padStart(2, "0"))
        .join("");

    const paddedHex = hex.padEnd(32, "0").substring(0, 32);

    return [
        paddedHex.substring(0, 8),
        paddedHex.substring(8, 12),
        paddedHex.substring(12, 16),
        paddedHex.substring(16, 20),
        paddedHex.substring(20, 32),
    ].join("-");
}

export function decodeUUIDToString(uuid: string) {
    const hex = uuid.replace(/-/g, "");

    return hex
        .split(/(\w\w)/g)
        .filter((p) => p)
        .map((c) => String.fromCharCode(Number.parseInt(c, 16)))
        .join("")
        .replace(/\0+$/, "");
}

interface QueueItem {
    characteristic: BluetoothRemoteGATTCharacteristic;
    value: Uint8Array;
}

export class BluetoothWriteQueue {
    queue: QueueItem[] = [];
    processing = false;

    async handleQueue() {
        this.processing = true;
        while (this.queue.length > 0) {
            const { characteristic, value } = this.queue.shift();
            try {
                await characteristic.writeValue(value);
            } catch (e) {}
        }
        this.processing = false;
    }

    write(characteristic: BluetoothRemoteGATTCharacteristic, value: Uint8Array) {
        this.queue.push({ characteristic, value });
        if (!this.processing) this.handleQueue().then();
    }
}

class BluetoothState {
    device = $state<BluetoothDevice>();
    server = $state<BluetoothRemoteGATTServer>();
    controlService = $state<BluetoothRemoteGATTService>();
    connected = $state(false);

    keys = new Map<string, BluetoothRemoteGATTCharacteristic>();
    queue = new BluetoothWriteQueue();

    constructor() {
        $effect.root(() => {
            $effect(() => {
                this.keys.clear();

                if (!this.controlService) return;
                this.controlService.getCharacteristics().then((characteristics) => {
                    characteristics.forEach((characteristic) => {
                        this.keys.set(
                            decodeUUIDToString(characteristic.uuid),
                            characteristic,
                        );
                    });
                });
            });
        });
    }

    async setup() {
        this.server = await this.device.gatt.connect();
        this.controlService = await this.server.getPrimaryService(SERVICE_UUID);
        this.connected = true;
    }

    async connect() {
        this.connected = false;
        if (!("bluetooth" in navigator)) {
            //
            return;
        }

        this.device = await navigator.bluetooth.requestDevice({
            filters: [{ services: [SERVICE_UUID] }],
        });
        this.device.addEventListener("gattserverdisconnected", async () => {
            this.connected = false;
            this.server = null;
            this.controlService = null;

            await this.setup();
        });

        await this.setup();
    }
}

export default new BluetoothState();