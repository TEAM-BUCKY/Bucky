// src/lib/state/simulation.svelte.ts
//
// Client for the Bucky backend. The live data stream is a public, read-only
// WebSocket that auto-reconnects with backoff. Control actions (launching or
// stopping a job) are authenticated HTTP POSTs, gated by the credentials the
// user enters in the UI (sent as HTTP Basic auth).
//
// Stream (server → browser): step | metrics | status | heartbeat | runs
// Control (browser → server, Basic auth): POST /jobs | POST /jobs/stop

export interface RewardTerms {
	approach: number;
	ball_to_goal: number;
	possession: number;
	front_alignment: number;
	goal: number;
	goal_against: number;
	out_of_bounds: number;
	lack_of_progress: number;
	defective: number;
	spin: number;
	time_penalty: number;
	action_magnitude: number;
	[key: string]: number;
}

/** Per-robot referee status (match frames). */
export interface RobotStatus {
	removed: boolean;
	suspended: boolean;
	defective: boolean;
	penalty_remaining: number;
}

export interface SimFrame {
	type: 'step';
	/** 'play' for a 1v1 match; absent/'train' for a single-agent training rollout. */
	mode?: 'train' | 'play';
	robot_pos: [number, number];
	robot_heading: number;
	/** Opponent robot (only present in match / self-play frames). */
	robot2_pos?: [number, number];
	robot2_heading?: number;
	/** Running 1v1 scoreboard (match frames only). */
	score?: { a: number; b: number };
	/** Match clock — seconds elapsed in the current half (match frames only). */
	clock?: number;
	/** Current half (1 or 2) and end-of-match flag (match frames only). */
	half?: number;
	match_over?: boolean;
	/** Per-robot referee status, keyed 'a'/'b' (match frames only). */
	status?: { a: RobotStatus; b: RobotStatus };
	/** Referee events fired this frame (e.g. 'out_of_bounds_a', 'lack_of_progress'). */
	events?: string[];
	ball_pos: [number, number];
	reward_terms: RewardTerms;
	reward_total?: number;
	obs: number[];
	episode: number;
	step: number;
	total_return?: number;
	num_timesteps?: number;
}

export type TrainingState =
	| 'unknown'
	| 'idle'
	| 'launching'
	| 'running'
	| 'stopping'
	| 'exited';

export type RunType = 'train' | 'match';

export interface TrainingStatus {
	state: TrainingState;
	trainer_connected: boolean;
	run_name?: string;
	run_type?: RunType;
	stage?: string;
	timesteps?: number;
	n_envs?: number;
	seed?: number;
	domain_rand?: boolean;
	phase?: string;
	num_timesteps?: number;
	/** Epoch seconds when the run process started (set by the backend). */
	started_at?: number;
	/** Epoch seconds when the run process exited (set by the backend). */
	ended_at?: number;
	/** Stop condition the run was launched with. */
	stop_kind?: StopKind;
	/** For time-based runs: epoch seconds the run will stop at. */
	deadline?: number;
	code?: number;
	message?: string;
}

/** How a training run decides when to stop. */
export type StopKind = 'steps' | 'duration' | 'until';
export interface StopCondition {
	kind: StopKind;
	/** steps → timesteps; duration → seconds; until → epoch seconds. */
	value: number;
}

export interface LaunchConfig {
	stage: string;
	timesteps: number;
	n_envs: number;
	seed: number;
	domain_rand: boolean;
	stop?: StopCondition;
	resume_from?: { run: string; checkpoint: string };
}

/** A queued run waiting to launch (back-to-back, optionally at a scheduled time). */
export interface QueueItem {
	id: string;
	mode: 'train' | 'play';
	config: Record<string, unknown> & { stage?: string; stop?: StopCondition };
	/** Epoch seconds it should start at/after, or null = as soon as the queue reaches it. */
	start_at: number | null;
	status: 'pending' | 'running' | 'failed';
	enqueued_at: number;
	message?: string;
}

export interface PolicyRef {
	run: string;
	checkpoint: string;
}

export interface MatchConfig {
	policyA: PolicyRef;
	policyB: PolicyRef;
	seed: number;
}

export interface RunInfo {
	run: string;
	checkpoints: string[];
}

export interface MetricPoint {
	x: number; // num_timesteps
	y: number;
}

const RETURNS_CAP = 50;
const METRIC_CAP = 400;
const STALE_MS = 5000;
const CREDS_KEY = 'bucky.creds';

/** Resolve the HTTP + WS base URLs from VITE_API_BASE (default same-origin '/api'). */
function apiBases() {
	const raw = (((import.meta as any).env?.VITE_API_BASE as string | undefined) ?? '/api').replace(
		/\/+$/,
		''
	);
	const origin = typeof window !== 'undefined' ? window.location.origin : '';
	const httpBase = /^https?:\/\//.test(raw) ? raw : origin + raw;
	const wsBase = httpBase.replace(/^http/, 'ws');
	return { httpBase, wsBase };
}

class SimulationState {
	/** Latest live-rollout frame. */
	frame = $state<SimFrame | null>(null);
	/** Connection to the stream (not the same as a job running). */
	connected = $state(false);
	error = $state<string | null>(null);
	/** Last 50 completed-episode returns, for the sparkline. */
	episodeReturns = $state<number[]>([]);
	/** Job lifecycle reported by the server. */
	status = $state<TrainingStatus>({ state: 'unknown', trainer_connected: false });
	/** Per-scalar time series keyed by metric name (e.g. "train/loss"). */
	metrics = $state<Record<string, MetricPoint[]>>({});
	/** Existing runs + their checkpoints, for the "continue from checkpoint" picker. */
	runs = $state<RunInfo[]>([]);
	/** Scheduled/queued runs waiting to launch. */
	queue = $state<QueueItem[]>([]);
	/** Epoch ms of the most recent message — used to show live vs stale. */
	lastMessageAt = $state(0);
	/** Ticks roughly once a second so `live` re-evaluates without new traffic. */
	now = $state(0);

	/** Username for control actions (shown in the UI when logged in). */
	username = $state('');
	/** Last control auth/error message, surfaced near the login control. */
	authError = $state<string | null>(null);

	private _password = '';
	private ws: WebSocket | null = null;
	private _lastEpisode = 0;
	private _manualClose = false;
	private _reconnectDelay = 500;
	private _reconnectTimer: ReturnType<typeof setTimeout> | null = null;
	private _clock: ReturnType<typeof setInterval> | null = null;

	/** True when connected and frames/heartbeats are arriving. */
	get live() {
		return this.connected && this.now - this.lastMessageAt < STALE_MS;
	}

	/** Whether credentials have been entered (not whether they're valid). */
	get hasCredentials() {
		return this.username.length > 0 && this._password.length > 0;
	}

	// ── auth ───────────────────────────────────────────────────────────────────
	setCredentials(username: string, password: string) {
		this.username = username.trim();
		this._password = password;
		this.authError = null;
		if (typeof window !== 'undefined') {
			try {
				window.localStorage.setItem(CREDS_KEY, JSON.stringify({ u: this.username, p: password }));
			} catch {
				/* storage unavailable */
			}
		}
	}

	logout() {
		this.username = '';
		this._password = '';
		this.authError = null;
		if (typeof window !== 'undefined') {
			try {
				window.localStorage.removeItem(CREDS_KEY);
			} catch {
				/* ignore */
			}
		}
	}

	private _loadCredentials() {
		if (this.hasCredentials || typeof window === 'undefined') return;
		try {
			const raw = window.localStorage.getItem(CREDS_KEY);
			if (!raw) return;
			const { u, p } = JSON.parse(raw) as { u: string; p: string };
			if (u && p) {
				this.username = u;
				this._password = p;
			}
		} catch {
			/* ignore malformed storage */
		}
	}

	// ── connection (public, read-only stream) ──────────────────────────────────
	connect() {
		this._loadCredentials();
		this._manualClose = false;
		this._clearReconnect();
		this._closeSocket();

		if (!this._clock) {
			this._clock = setInterval(() => (this.now = Date.now()), 1000);
		}

		const { wsBase } = apiBases();
		const url = `${wsBase}/stream`;
		try {
			const ws = new WebSocket(url);
			this.ws = ws;

			ws.onopen = () => {
				this.connected = true;
				this.error = null;
				this._reconnectDelay = 500;
				this.lastMessageAt = Date.now();
				this.now = Date.now();
			};

			ws.onclose = () => {
				this.connected = false;
				if (!this._manualClose) this._scheduleReconnect();
			};

			ws.onerror = () => {
				this.error = 'Cannot reach the server';
				this.connected = false;
			};

			ws.onmessage = (evt: MessageEvent) => {
				this.lastMessageAt = Date.now();
				this.now = this.lastMessageAt;
				let data: Record<string, unknown>;
				try {
					data = JSON.parse(evt.data as string);
				} catch {
					return;
				}
				this._dispatch(data);
			};
		} catch (e) {
			this.error = String(e);
			this._scheduleReconnect();
		}
	}

	disconnect() {
		this._manualClose = true;
		this._clearReconnect();
		this._closeSocket();
		this.connected = false;
	}

	// ── control (authenticated) ─────────────────────────────────────────────────
	async launch(config: LaunchConfig) {
		await this._control('/jobs', { mode: 'train', ...config });
	}

	/** Start a 1v1 match between two checkpointed policies. */
	async match(config: MatchConfig) {
		await this._control('/jobs', {
			mode: 'play',
			policy_a: config.policyA,
			policy_b: config.policyB,
			seed: config.seed
		});
	}

	async kill() {
		await this._control('/jobs/stop');
	}

	/** Add a run to the schedule queue. `startAt` is epoch seconds, or null for ASAP. */
	async enqueue(config: LaunchConfig | MatchConfig, startAt: number | null = null) {
		const isMatch = 'policyA' in config;
		const body: Record<string, unknown> = isMatch
			? {
					mode: 'play',
					policy_a: (config as MatchConfig).policyA,
					policy_b: (config as MatchConfig).policyB,
					seed: (config as MatchConfig).seed
				}
			: { mode: 'train', ...(config as LaunchConfig) };
		body.start_at = startAt;
		await this._control('/queue', body);
	}

	async dequeue(id: string) {
		await this._control(`/queue/${id}`, undefined, 'DELETE');
	}

	async clearQueue() {
		await this._control('/queue/clear');
	}

	private async _control(
		path: string,
		body?: Record<string, unknown>,
		method: 'POST' | 'DELETE' = 'POST'
	): Promise<boolean> {
		if (!this.hasCredentials) {
			this.authError = 'Log in to control training.';
			return false;
		}
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + path, {
				method,
				headers: {
					'Content-Type': 'application/json',
					Authorization: 'Basic ' + btoa(`${this.username}:${this._password}`)
				},
				body: method === 'DELETE' ? undefined : JSON.stringify(body ?? {})
			});
			if (res.status === 401) {
				this.authError = 'Invalid username or password.';
				return false;
			}
			if (res.status === 503) {
				this.status = { ...this.status, message: 'Control is disabled on the server.' };
				return false;
			}
			if (!res.ok) {
				let detail = `Request failed (${res.status})`;
				try {
					const j = await res.json();
					if (j?.detail) detail = String(j.detail);
				} catch {
					/* non-JSON error body */
				}
				this.status = { ...this.status, message: detail };
				return false;
			}
			this.authError = null;
			return true;
		} catch {
			this.status = { ...this.status, message: 'Cannot reach the server.' };
			return false;
		}
	}

	// ── internals ────────────────────────────────────────────────────────────
	private _dispatch(data: Record<string, unknown>) {
		switch (data.type) {
			case 'step':
				this._onStep(data as unknown as SimFrame);
				break;
			case 'status':
				this.status = data as unknown as TrainingStatus;
				break;
			case 'metrics':
				this._onMetrics(data as { num_timesteps: number; values: Record<string, number> });
				break;
			case 'runs':
				if (Array.isArray(data.runs)) this.runs = data.runs as RunInfo[];
				break;
			case 'queue':
				if (Array.isArray(data.items)) this.queue = data.items as QueueItem[];
				break;
			case 'heartbeat':
				if (typeof data.trainer_connected === 'boolean') {
					this.status = { ...this.status, trainer_connected: data.trainer_connected };
				}
				break;
		}
	}

	private _onStep(data: SimFrame) {
		if (data.episode !== this._lastEpisode) {
			if (this.frame && typeof this.frame.total_return === 'number') {
				this.episodeReturns = [
					...this.episodeReturns.slice(-(RETURNS_CAP - 1)),
					this.frame.total_return
				];
			}
			this._lastEpisode = data.episode;
		}
		this.frame = data;
	}

	private _onMetrics(data: { num_timesteps: number; values: Record<string, number> }) {
		const next = { ...this.metrics };
		for (const [key, value] of Object.entries(data.values ?? {})) {
			if (typeof value !== 'number' || Number.isNaN(value)) continue;
			const series = next[key] ? next[key].slice(-(METRIC_CAP - 1)) : [];
			next[key] = [...series, { x: data.num_timesteps, y: value }];
		}
		this.metrics = next;
	}

	private _scheduleReconnect() {
		this._clearReconnect();
		const delay = this._reconnectDelay;
		this._reconnectDelay = Math.min(this._reconnectDelay * 2, 5000);
		this._reconnectTimer = setTimeout(() => this.connect(), delay);
	}

	private _clearReconnect() {
		if (this._reconnectTimer) {
			clearTimeout(this._reconnectTimer);
			this._reconnectTimer = null;
		}
	}

	private _closeSocket() {
		if (this.ws) {
			this.ws.onopen = this.ws.onclose = this.ws.onerror = this.ws.onmessage = null;
			try {
				this.ws.close();
			} catch {
				/* already closing */
			}
			this.ws = null;
		}
	}
}

export const simulation = new SimulationState();
