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
	speed: number;
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
	action_smoothness: number;
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
	/** Kicker charge state, 1 = ready / 0 = recharging (match frames only). */
	kick_ready_a?: number;
	kick_ready_b?: number;
	reward_terms: RewardTerms;
	reward_total?: number;
	obs: number[];
	episode: number;
	step: number;
	total_return?: number;
	num_timesteps?: number;
	/** Source run name, tagged by the backend on every streamed frame. */
	run?: string;
}

/** A live frame from an evaluation drill. Distinct from {@link SimFrame} (type `eval_step`,
 * not `step`) so it can never disturb the training viewer that shares the same stream. */
export interface EvalStepFrame {
	type: 'eval_step';
	mode?: 'train' | 'play';
	robot_pos: [number, number];
	robot_heading: number;
	robot2_pos?: [number, number];
	robot2_heading?: number;
	ball_pos: [number, number];
	/** Instantaneous reward terms this step. */
	reward_terms: RewardTerms;
	reward_total: number;
	/** Per-term running total for the current episode (the "build-up"). */
	reward_cumulative: RewardTerms;
	obs: number[];
	episode: number;
	step: number;
	total_return: number;
	episodes_total: number;
	run?: string;
	device?: string;
}

/** The N-episode aggregate emitted once at the end of an evaluation drill. */
export interface EvalSummary {
	type: 'eval_summary';
	checkpoint: string;
	stage: string;
	n_episodes: number;
	deterministic: boolean;
	opponent?: string;
	return_mean: number;
	return_std: number;
	success_rate: number;
	per_term_mean: Record<string, number>;
	per_term_std: Record<string, number>;
	episode_returns: number[];
	run?: string;
	device?: string;
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

/** PPO hyperparameters; any subset may be sent (missing keys keep server defaults). */
export interface Hyperparams {
	learning_rate?: number;
	n_steps?: number;
	batch_size?: number;
	n_epochs?: number;
	gamma?: number;
	gae_lambda?: number;
	clip_range?: number;
	ent_coef?: number;
	vf_coef?: number;
	max_grad_norm?: number;
}

export interface LaunchConfig {
	stage: string;
	/** FULL_TRAINING only: per-phase budget fractions (APPROACH / PUSH / SELF_PLAY). */
	full_training_split?: number[];
	timesteps: number;
	n_envs: number;
	seed: number;
	domain_rand: boolean;
	stop?: StopCondition;
	resume_from?: { run: string; checkpoint: string };
	// Per-model config (optional — creates a named/versioned "kind" of model).
	name?: string;
	version?: string;
	hyperparams?: Hyperparams;
	net_arch?: number[];
	reward_weights?: Record<string, number>;
	/** Keep periodic model_<N>_steps.zip snapshots (off by default). */
	save_step_checkpoints?: boolean;
	/** Where the run executes: 'any' (default), 'server', or a device id. */
	target?: string;
	/** Animate the live field while training (off by default — headless trains faster). */
	viz?: boolean;
	/**
	 * Split one run across devices via FedAvg — either even shards, or a per-device
	 * map sizing each device's shard to its capacity (target = device id or 'server').
	 */
	distributed?: {
		shards?: number;
		sync_every: number;
		devices?: { target: string; n_envs: number }[];
	};
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
	/** Let a human drive red (robot A) via the control channel instead of its policy. */
	manualRed?: boolean;
}

/** A "play a friend by game code" session this browser is part of. */
export interface GameSession {
	code: string;
	runName: string;
	mode: 'casual' | 'match';
	/** Which robot this browser drives: 'a' = blue, 'b' = red, null = spectator. */
	side: 'a' | 'b' | null;
	/** Secret per-side token that authorises control (open guest play — no login). */
	sideToken: string;
	spectator: boolean;
	opponentPresent: boolean;
}

export interface RunInfo {
	run: string;
	checkpoints: string[];
}

/** A single checkpoint file within a model run. */
export interface ModelCheckpoint {
	file: string;
	size: number;
	mtime: number;
}

/** An enriched model record for the overview panel (one per run directory). */
export interface ModelInfo {
	run: string;
	name: string;
	version: string | null;
	stage: string | null;
	status: string | null;
	created_at: number | null;
	created_by: string | null;
	timesteps_trained?: number | null;
	parent?: string | null;
	best_eval: number | null;
	has_meta: boolean;
	config?: Record<string, unknown> | null;
	checkpoints: ModelCheckpoint[];
}

export interface MetricPoint {
	x: number; // num_timesteps
	y: number;
}

/** A registered guest training device. */
export interface DeviceInfo {
	id: string;
	name: string;
	created_at: number;
	last_seen: number | null;
	/** Runs this device is currently training (a device may hold several at once). */
	current_jobs: string[];
	/** Back-compat: the first current job, or null when idle. */
	current_job: string | null;
	online: boolean;
	/** Admin concurrency cap (slider); null = run at the worker's reported capacity. */
	max_slots: number | null;
	/** Worker-reported CPU core count (the slider's ceiling); null until first check-in. */
	reported_cores: number | null;
	/** Worker-reported recommended concurrency (the default when max_slots is unset). */
	reported_capacity: number | null;
	/** Effective concurrency in force now: max_slots ?? reported_capacity (≥1). */
	effective_slots: number;
}

/** One in-flight run (local or leased to a device), as listed by the `active_runs` frame. */
export interface ActiveRun {
	run_name: string;
	/** 'server' = the in-process local trainer, otherwise a device id. */
	device: string;
	run_type?: string;
	state?: string;
	phase?: string;
	num_timesteps?: number;
	/** Play match where a human drives red (robot A). */
	manual_red?: boolean;
	stage?: string;
	model_name?: string;
	model_version?: string;
	started_at?: number;
	cancel_requested?: boolean;
	stop_kind?: StopKind;
	deadline?: number;
	/** Set when this run is one shard of a federated distributed group. */
	dist_group?: string;
}

const RETURNS_CAP = 50;
/** How many eval frames to keep for scrubbing (~2 min at 50 fps). */
const EVAL_BUFFER_CAP = 6000;
const METRIC_CAP = 400;
const STALE_MS = 5000;
const CREDS_KEY = 'bucky.creds';
const GAME_KEY = 'bucky.game';

/** Resolve the HTTP + WS base URLs from VITE_API_BASE (default same-origin '/api'). */
export function apiBases() {
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
	/** Enriched model records (name/version/config/best-eval) for the overview panel. */
	models = $state<ModelInfo[]>([]);
	/** Registered guest training devices. */
	devices = $state<DeviceInfo[]>([]);
	/** Latest live frame per source device ('server' = the in-process trainer). */
	framesByDevice = $state<Record<string, SimFrame>>({});
	/** Latest live frame per run name — lets the play view follow a specific match even
	 * when other runs (e.g. training) stream concurrently from the same device. */
	framesByRun = $state<Record<string, SimFrame>>({});
	/** Live phase/steps reported by remote workers, keyed by device id. */
	remoteStatus = $state<Record<string, { phase?: string; num_timesteps?: number; run?: string }>>(
		{}
	);
	/** Which device's stream to follow in the viewer; '' = auto (latest to arrive). */
	selectedDevice = $state<string>('');
	/** Scheduled/queued runs waiting to launch. */
	queue = $state<QueueItem[]>([]);
	/** All in-flight runs (local + leased to devices), for the multi-run panel. */
	activeRuns = $state<ActiveRun[]>([]);
	/** Concurrent-run capacity of the server's in-process worker. */
	localSlots = $state(1);
	/** How many of the server's local slots are currently busy. */
	localActive = $state(0);
	/** Epoch ms of the most recent message — used to show live vs stale. */
	lastMessageAt = $state(0);
	/** Ticks roughly once a second so `live` re-evaluates without new traffic. */
	now = $state(0);

	/** Username for control actions (shown in the UI when logged in). In OAuth mode
	 * this mirrors the signed-in GitHub login. */
	username = $state('');
	/** Last control auth/error message, surfaced near the login control. */
	authError = $state<string | null>(null);
	/** True when the server gates control with GitHub OAuth (vs the password fallback). */
	oauthMode = $state(false);
	/** The GitHub org whose members may control the server (OAuth mode only). */
	oauthOrg = $state('');
	/** The signed-in GitHub user (OAuth mode only), or null when not logged in. */
	authUser = $state<{ login: string; name: string | null; avatar_url: string | null } | null>(null);
	/** True once /auth/me has been read at least once (so the UI can avoid flicker). */
	authReady = $state(false);

	/** The "play a friend by game code" session this browser is in, or null. */
	game = $state<GameSession | null>(null);
	/** Last error from a game create/join/leave action, surfaced in the lobby. */
	gameError = $state<string | null>(null);

	// ── evaluation tool (/eval page) ───────────────────────────────────────────
	/** The eval run this page launched; the /eval view filters the stream to it so a
	 * concurrent training run's frames never leak in (and eval frames never touch /viz). */
	evalRun = $state<string | null>(null);
	/** Latest eval step frame for {@link evalRun} (the live tail). */
	evalFrame = $state<EvalStepFrame | null>(null);
	/** Rolling buffer of recent eval frames, so playback can be paused and scrubbed/stepped
	 * (frame-by-frame debugging) through history while the rollout is frozen. */
	evalFrames = $state<EvalStepFrame[]>([]);
	/** Index into {@link evalFrames} of the frame currently shown (the playhead). */
	evalCursor = $state(-1);
	/** True = following the live tail; false = paused (the server-side rollout is frozen too). */
	evalPlaying = $state(true);
	/** The N-episode aggregate, set when the drill finishes. */
	evalSummary = $state<EvalSummary | null>(null);
	/** Server-reported eval lifecycle ({phase, running}). */
	evalStatus = $state<{ phase?: string; running: boolean }>({ running: false });
	/** Last error from a start/stop eval action. */
	evalError = $state<string | null>(null);
	/** Live playback-speed multiplier for the running eval (1.0 = real time). */
	evalSpeed = $state(1);
	/** A single-step was requested; advance the playhead onto the next frame that arrives. */
	private _evalPendingStep = false;

	/** The eval frame currently shown (the playhead), scrubbing-aware. */
	get evalView(): EvalStepFrame | null {
		return this.evalFrames[this.evalCursor] ?? null;
	}

	/** Whether the playhead sits on the most recent buffered frame. */
	get evalAtLiveEdge(): boolean {
		return this.evalCursor >= this.evalFrames.length - 1;
	}

	/** Cumulative return trace for the shown frame's episode, up to the playhead. */
	get evalCumulativeHistory(): MetricPoint[] {
		const cur = this.evalView;
		if (!cur) return [];
		const out: MetricPoint[] = [];
		for (let i = 0; i <= this.evalCursor && i < this.evalFrames.length; i++) {
			const f = this.evalFrames[i];
			if (f.episode === cur.episode) out.push({ x: f.step, y: f.total_return });
		}
		return out;
	}

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

	/** Whether the user can issue control actions: a live OAuth session, or (in the
	 * password fallback) entered credentials. */
	get hasCredentials() {
		if (this.oauthMode) return this.authUser !== null;
		return this.username.length > 0 && this._password.length > 0;
	}

	// ── auth ───────────────────────────────────────────────────────────────────
	/** Read the server's auth mode + current user. Safe to call repeatedly. */
	async refreshAuth() {
		const { httpBase } = apiBases();
		// Surface a GitHub-org rejection bounced back to the SPA as ?auth_error=...
		if (typeof window !== 'undefined') {
			const params = new URLSearchParams(window.location.search);
			if (params.get('auth_error') === 'not_member') {
				this.authError = `That GitHub account is not a member of the ${
					params.get('org') || 'required'
				} organization.`;
				params.delete('auth_error');
				params.delete('org');
				const qs = params.toString();
				window.history.replaceState({}, '', window.location.pathname + (qs ? `?${qs}` : ''));
			}
		}
		try {
			const res = await fetch(httpBase + '/auth/me', { credentials: 'include' });
			if (!res.ok) return;
			const j = await res.json();
			this.oauthMode = !!j.oauth;
			this.oauthOrg = j.org ?? '';
			if (this.oauthMode) {
				this.authUser = (j.user as typeof this.authUser) ?? null;
				this.username = j.user?.login ?? '';
			}
			this.authReady = true;
		} catch {
			/* server unreachable — leave auth state as-is */
		}
	}

	/** Begin the GitHub OAuth flow (full-page redirect to the backend). */
	loginWithGitHub() {
		const { httpBase } = apiBases();
		window.location.href = httpBase + '/auth/login';
	}

	/** A 401 means different things per mode: re-auth with GitHub vs bad password. */
	private _authErrorMessage(): string {
		if (this.oauthMode) {
			this.authUser = null; // session is gone/expired — reflect logged-out state
			return 'Your session expired. Sign in with GitHub again.';
		}
		return 'Invalid username or password.';
	}

	/** Request init carrying the right credentials for the active auth mode. */
	private _authInit(headers: Record<string, string> = {}): RequestInit {
		if (this.oauthMode) {
			return { headers, credentials: 'include' };
		}
		return {
			headers: { ...headers, Authorization: 'Basic ' + btoa(`${this.username}:${this._password}`) }
		};
	}

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

	async logout() {
		if (this.oauthMode) {
			const { httpBase } = apiBases();
			try {
				await fetch(httpBase + '/auth/logout', { method: 'POST', credentials: 'include' });
			} catch {
				/* best-effort */
			}
			this.authUser = null;
			this.username = '';
			this.authError = null;
			return;
		}
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
		this._loadGame();
		// Determine auth mode (OAuth vs password) and current user, in the background.
		void this.refreshAuth();
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
			seed: config.seed,
			manual_red: config.manualRed ?? false
		});
	}

	/** Launch an evaluation drill. Returns the eval run name (so the page can filter the
	 * stream to it), or null on failure. Runs in the server's separate eval lane. */
	async startEval(config: {
		run: string;
		checkpoint: string;
		stage: string;
		nEpisodes: number;
		seed: number;
		deterministic: boolean;
	}): Promise<string | null> {
		if (!this.hasCredentials) {
			this.evalError = 'Log in to run evaluations.';
			return null;
		}
		// Reset the view for the new run before frames start arriving.
		this.evalSummary = null;
		this.evalFrame = null;
		this.evalFrames = [];
		this.evalCursor = -1;
		this.evalPlaying = true;
		this.evalSpeed = 1;
		this._evalPendingStep = false;
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + '/eval', {
				method: 'POST',
				...this._authInit({ 'Content-Type': 'application/json' }),
				body: JSON.stringify({
					run: config.run,
					checkpoint: config.checkpoint,
					stage: config.stage,
					n_episodes: config.nEpisodes,
					seed: config.seed,
					deterministic: config.deterministic
				})
			});
			if (!res.ok) {
				let detail = `Eval failed (${res.status})`;
				try {
					const j = await res.json();
					if (j?.detail) detail = String(j.detail);
				} catch {
					/* non-JSON error body */
				}
				this.evalError = detail;
				return null;
			}
			const j = await res.json();
			this.evalError = null;
			this.evalRun = (j?.run_name as string) ?? null;
			this.evalStatus = { phase: 'launching', running: true };
			return this.evalRun;
		} catch {
			this.evalError = 'Cannot reach the server.';
			return null;
		}
	}

	/** Stop the running evaluation drill (targeted SIGINT on the server). */
	async stopEval(): Promise<void> {
		await this._control('/eval/stop');
	}

	/** Send a live transport command to the running eval (best-effort, fire-and-forget). */
	private async _evalControl(body: { speed?: number; paused?: boolean; step?: number }): Promise<void> {
		if (!this.hasCredentials) return;
		const { httpBase } = apiBases();
		try {
			await fetch(httpBase + '/eval/control', {
				method: 'POST',
				...this._authInit({ 'Content-Type': 'application/json' }),
				body: JSON.stringify(body)
			});
		} catch {
			/* transport is best-effort; ignore transient errors */
		}
	}

	/** Live-adjust the running eval's playback speed (1.0 = real time). */
	async setEvalSpeed(speed: number): Promise<void> {
		this.evalSpeed = speed;
		await this._evalControl({ speed });
	}

	/** Play / pause the rollout. Pausing freezes the server-side sim; playing snaps the
	 * playhead back to the live tail and resumes streaming. */
	async setEvalPlaying(playing: boolean): Promise<void> {
		this.evalPlaying = playing;
		if (playing) this.evalCursor = this.evalFrames.length - 1;
		await this._evalControl({ paused: !playing });
	}

	/** Pause and move the playhead one frame back through the buffer. */
	evalStepBack(): void {
		if (this.evalPlaying) this.setEvalPlaying(false);
		this.evalCursor = Math.max(0, this.evalCursor - 1);
	}

	/** Step one frame forward: within the buffer if scrubbed back, else advance the frozen
	 * sim by exactly one frame (true frame-by-frame debugging). */
	async evalStepForward(): Promise<void> {
		if (this.evalPlaying) await this.setEvalPlaying(false);
		if (this.evalCursor < this.evalFrames.length - 1) {
			this.evalCursor += 1;
		} else {
			this._evalPendingStep = true;
			await this._evalControl({ step: 1 });
		}
	}

	/** Pause and jump the playhead to the first buffered frame of the shown episode. */
	evalRewind(): void {
		if (this.evalPlaying) this.setEvalPlaying(false);
		const cur = this.evalView;
		if (!cur) return;
		let i = this.evalCursor;
		while (i > 0 && this.evalFrames[i - 1]?.episode === cur.episode) i--;
		this.evalCursor = i;
	}

	/** Pause and move the playhead to an absolute buffer index (scrubber drag). */
	evalSeek(index: number): void {
		if (this.evalPlaying) this.setEvalPlaying(false);
		this.evalCursor = Math.max(0, Math.min(this.evalFrames.length - 1, Math.round(index)));
	}

	/** Run name of the currently-streaming manual (human-controlled) match, or null. */
	get manualMatchRun(): string | null {
		const run = this.frame?.run;
		if (this.frame?.mode !== 'play' || !run) return null;
		const active = this.activeRuns.find((r) => r.run_name === run);
		return active?.manual_red ? run : null;
	}

	/** Send a manual control command (human action and/or red mode) to a running match.
	 * Best-effort and quiet: this fires at ~25 Hz, so transient failures are ignored rather
	 * than flashing error banners. */
	async pushControl(
		run: string,
		payload: { action?: number[]; redMode?: 'human' | 'ai' }
	): Promise<void> {
		if (!this.hasCredentials || !run) return;
		const body: Record<string, unknown> = { run };
		if (payload.action) body.action = payload.action;
		if (payload.redMode) body.red_mode = payload.redMode;
		const { httpBase } = apiBases();
		try {
			await fetch(httpBase + '/control', {
				method: 'POST',
				...this._authInit({ 'Content-Type': 'application/json' }),
				body: JSON.stringify(body)
			});
		} catch {
			/* control is best-effort; ignore transient errors */
		}
	}

	// ── play a friend by game code (open guest play — no login) ──────────────────
	/** Latest live frame for the current game's match (null until one arrives). */
	get gameFrame(): SimFrame | null {
		const run = this.game?.runName;
		return run ? (this.framesByRun[run] ?? null) : null;
	}

	get gameRun(): string | null {
		return this.game?.runName ?? null;
	}

	private async _errDetail(res: Response, fallback: string): Promise<string> {
		try {
			const j = await res.json();
			if (j?.detail) return String(j.detail);
		} catch {
			/* non-JSON error body */
		}
		return fallback;
	}

	/** Host a new game; returns true on success. Claims `side` (blue 'a' by default) and
	 * stores the secret token that authorises this browser's control. */
	async createGame(mode: 'casual' | 'match' = 'casual', side: 'a' | 'b' = 'a'): Promise<boolean> {
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + '/games', {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify({ mode, claim_side: side })
			});
			if (!res.ok) {
				this.gameError = await this._errDetail(res, 'Could not start a game.');
				return false;
			}
			const j = await res.json();
			this.game = {
				code: j.code,
				runName: j.run_name,
				mode: j.mode,
				side: j.your_side ?? null,
				sideToken: j.side_token ?? '',
				spectator: false,
				opponentPresent: false
			};
			this._saveGame();
			this.gameError = null;
			return true;
		} catch {
			this.gameError = 'Cannot reach the server.';
			return false;
		}
	}

	/** Join an existing game by its code. Takes a free side, or becomes a spectator if full. */
	async joinGame(code: string): Promise<boolean> {
		const { httpBase } = apiBases();
		const c = code.trim().toUpperCase();
		if (!c) {
			this.gameError = 'Enter a game code.';
			return false;
		}
		try {
			const res = await fetch(httpBase + `/games/${encodeURIComponent(c)}/join`, {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify({})
			});
			if (!res.ok) {
				this.gameError = await this._errDetail(res, 'Could not join that game.');
				return false;
			}
			const j = await res.json();
			this.game = {
				code: c,
				runName: j.run_name,
				mode: j.mode,
				side: j.your_side ?? null,
				sideToken: j.side_token ?? '',
				spectator: !!j.spectator,
				opponentPresent: !!j.opponent_present
			};
			this._saveGame();
			this.gameError = null;
			return true;
		} catch {
			this.gameError = 'Cannot reach the server.';
			return false;
		}
	}

	/** Leave the current game (releases the side so someone else can take it). */
	async leaveGame(): Promise<void> {
		const g = this.game;
		this.game = null;
		this._clearGame();
		if (!g || g.spectator || !g.side) return;
		const { httpBase } = apiBases();
		try {
			await fetch(httpBase + `/games/${encodeURIComponent(g.code)}/leave`, {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify({ side: g.side, token: g.sideToken })
			});
		} catch {
			/* best-effort */
		}
	}

	/** Poll the room: track whether the opponent is present, and detect a game that ended. */
	async refreshGame(): Promise<void> {
		const g = this.game;
		if (!g) return;
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + `/games/${encodeURIComponent(g.code)}`);
			if (res.status === 404) {
				this.game = null;
				this._clearGame();
				this.gameError = 'The game has ended.';
				return;
			}
			if (!res.ok) return;
			const j = await res.json();
			if (g.side) {
				const opp = g.side === 'a' ? 'b' : 'a';
				this.game = { ...g, opponentPresent: !!j.slots?.[opp]?.claimed };
			}
		} catch {
			/* ignore transient poll errors */
		}
	}

	/** Send this player's control to their side of the game. Best-effort, fires at ~30 Hz. */
	async pushGameControl(payload: { action?: number[]; mode?: 'human' | 'ai' }): Promise<void> {
		const g = this.game;
		if (!g || g.spectator || !g.side) return;
		const body: Record<string, unknown> = { side: g.side, token: g.sideToken };
		if (payload.action) body.action = payload.action;
		if (payload.mode) body.mode = payload.mode;
		const { httpBase } = apiBases();
		try {
			await fetch(httpBase + `/games/${encodeURIComponent(g.code)}/control`, {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify(body)
			});
		} catch {
			/* control is best-effort; ignore transient errors */
		}
	}

	private _saveGame() {
		if (typeof window === 'undefined' || !this.game) return;
		try {
			window.localStorage.setItem(GAME_KEY, JSON.stringify(this.game));
		} catch {
			/* storage unavailable */
		}
	}

	private _clearGame() {
		if (typeof window === 'undefined') return;
		try {
			window.localStorage.removeItem(GAME_KEY);
		} catch {
			/* ignore */
		}
	}

	private _loadGame() {
		if (this.game || typeof window === 'undefined') return;
		try {
			const raw = window.localStorage.getItem(GAME_KEY);
			if (!raw) return;
			this.game = JSON.parse(raw) as GameSession;
			void this.refreshGame(); // confirm it still exists; clears it if the match ended
		} catch {
			/* ignore malformed storage */
		}
	}

	async kill() {
		await this._control('/jobs/stop');
	}

	/** Stop one specific run by name (local run → terminated; remote run → cancel
	 * requested, the worker checkpoints and stops). */
	async stopRun(runName: string) {
		await this._control(`/jobs/${encodeURIComponent(runName)}/stop`);
	}

	/** Whether the server's in-process worker has a free slot to launch into now. */
	get canLaunchLocal(): boolean {
		return this.localActive < this.localSlots;
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

	/** Create + launch a new model version now. `config` carries name/version/settings. */
	async createModel(config: LaunchConfig) {
		await this.launch(config);
	}

	/** Queue a new model version (optionally at a scheduled time). */
	async queueModel(config: LaunchConfig, startAt: number | null = null) {
		await this.enqueue(config, startAt);
	}

	async deleteModel(run: string) {
		await this._control(`/models/${encodeURIComponent(run)}`, undefined, 'DELETE');
	}

	async deleteCheckpoint(run: string, checkpoint: string) {
		await this._control(
			`/models/${encodeURIComponent(run)}/${encodeURIComponent(checkpoint)}`,
			undefined,
			'DELETE'
		);
	}

	/** Delete the periodic *_steps.zip snapshots in a run (keeps best/final). */
	async pruneStepCheckpoints(run: string) {
		await this._control(`/models/${encodeURIComponent(run)}/prune-steps`);
	}

	/**
	 * Download a checkpoint .zip. Uses an authenticated fetch → blob → object-URL
	 * because the download endpoint is gated by Basic auth, which a plain <a href>
	 * cannot carry.
	 */
	async downloadModel(run: string, checkpoint: string) {
		if (!this.hasCredentials) {
			this.authError = 'Log in to download models.';
			return;
		}
		const { httpBase } = apiBases();
		const path = `/models/${encodeURIComponent(run)}/${encodeURIComponent(checkpoint)}/download`;
		try {
			const res = await fetch(httpBase + path, this._authInit());
			if (res.status === 401) {
				this.authError = this._authErrorMessage();
				return;
			}
			if (!res.ok) {
				this.status = { ...this.status, message: `Download failed (${res.status})` };
				return;
			}
			const blob = await res.blob();
			const url = URL.createObjectURL(blob);
			const a = document.createElement('a');
			a.href = url;
			a.download = `${run}_${checkpoint}`;
			document.body.appendChild(a);
			a.click();
			a.remove();
			URL.revokeObjectURL(url);
			this.authError = null;
		} catch {
			this.status = { ...this.status, message: 'Cannot reach the server.' };
		}
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
				...this._authInit({ 'Content-Type': 'application/json' }),
				body: method === 'DELETE' ? undefined : JSON.stringify(body ?? {})
			});
			if (res.status === 401) {
				this.authError = this._authErrorMessage();
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

	// ── devices (distributed training) ──────────────────────────────────────────
	/** Register a guest device. Returns its one-time token, or null on failure. */
	async registerDevice(name: string): Promise<string | null> {
		if (!this.hasCredentials) {
			this.authError = 'Log in to register devices.';
			return null;
		}
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + '/devices', {
				method: 'POST',
				...this._authInit({ 'Content-Type': 'application/json' }),
				body: JSON.stringify({ name })
			});
			if (res.status === 401) {
				this.authError = this._authErrorMessage();
				return null;
			}
			if (!res.ok) {
				this.status = { ...this.status, message: `Register failed (${res.status})` };
				return null;
			}
			this.authError = null;
			const j = await res.json();
			return (j?.token as string) ?? null;
		} catch {
			this.status = { ...this.status, message: 'Cannot reach the server.' };
			return null;
		}
	}

	async revokeDevice(id: string) {
		await this._control(`/devices/${encodeURIComponent(id)}`, undefined, 'DELETE');
	}

	/**
	 * Set how many jobs a device may train concurrently. `null` clears the override so
	 * the device runs at its worker-reported capacity. The cap takes effect on the
	 * worker's next poll/heartbeat; the server re-broadcasts the device list on success.
	 */
	async setDeviceSlots(id: string, maxSlots: number | null) {
		return this._control(`/devices/${encodeURIComponent(id)}/slots`, { max_slots: maxSlots });
	}

	/** Issue a fresh token for a device (old one stops working). Returns it once, or null. */
	async rotateDeviceToken(id: string): Promise<string | null> {
		if (!this.hasCredentials) {
			this.authError = 'Log in to manage devices.';
			return null;
		}
		const { httpBase } = apiBases();
		try {
			const res = await fetch(httpBase + `/devices/${encodeURIComponent(id)}/token`, {
				method: 'POST',
				...this._authInit()
			});
			if (res.status === 401) {
				this.authError = this._authErrorMessage();
				return null;
			}
			if (!res.ok) {
				this.status = { ...this.status, message: `Token rotation failed (${res.status})` };
				return null;
			}
			this.authError = null;
			const j = await res.json();
			return (j?.token as string) ?? null;
		} catch {
			this.status = { ...this.status, message: 'Cannot reach the server.' };
			return null;
		}
	}

	/** Base URL a worker passes as `--server` (the API origin, sans the `/api` suffix). */
	get workerServerUrl(): string {
		return apiBases().httpBase.replace(/\/api$/, '');
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
			case 'models':
				if (Array.isArray(data.models)) this.models = data.models as ModelInfo[];
				break;
			case 'devices':
				if (Array.isArray(data.devices)) this.devices = data.devices as DeviceInfo[];
				break;
			case 'remote_status': {
				const dev = data.device as string;
				if (dev)
					this.remoteStatus = {
						...this.remoteStatus,
						[dev]: {
							phase: data.phase as string | undefined,
							num_timesteps: data.num_timesteps as number | undefined,
							run: data.run as string | undefined
						}
					};
				break;
			}
			case 'queue':
				if (Array.isArray(data.items)) this.queue = data.items as QueueItem[];
				break;
			case 'active_runs':
				if (Array.isArray(data.runs)) this.activeRuns = data.runs as ActiveRun[];
				if (typeof data.local_slots === 'number') this.localSlots = data.local_slots;
				if (typeof data.local_active === 'number') this.localActive = data.local_active;
				break;
			case 'run_exited':
				// Drop any lingering per-device frame for an ended run's source.
				break;
			case 'heartbeat':
				if (typeof data.trainer_connected === 'boolean') {
					this.status = { ...this.status, trainer_connected: data.trainer_connected };
				}
				break;
			case 'eval_step':
				this._onEvalStep(data as unknown as EvalStepFrame);
				break;
			case 'eval_summary': {
				const s = data as unknown as EvalSummary;
				if (!this.evalRun || s.run === this.evalRun) this.evalSummary = s;
				break;
			}
			case 'eval_status': {
				const ev = data.eval as { run_name?: string; phase?: string } | null;
				this.evalStatus = { phase: ev?.phase, running: !!ev };
				// Recover an eval already running when this page loaded (e.g. after a reload),
				// so the stream filter locks onto it.
				if (ev?.run_name && !this.evalRun) this.evalRun = ev.run_name;
				break;
			}
		}
	}

	private _onEvalStep(data: EvalStepFrame) {
		// Only follow the eval run this page launched, so a concurrent training run's frames
		// never leak into the eval viewer (and these frames never touch `frame`/`episodeReturns`).
		if (this.evalRun && data.run !== this.evalRun) return;
		this.evalFrame = data;
		const buf = [...this.evalFrames, data];
		if (buf.length > EVAL_BUFFER_CAP) buf.splice(0, buf.length - EVAL_BUFFER_CAP);
		this.evalFrames = buf;
		// Advance the playhead to the new frame when playing, or when fulfilling a single-step.
		// While paused-and-scrubbing the rollout is frozen, so no frames arrive to move it.
		if (this.evalPlaying || this._evalPendingStep) {
			this.evalCursor = buf.length - 1;
			this._evalPendingStep = false;
		}
	}

	private _onStep(data: SimFrame) {
		// Tag-aware: frames carry `device` ('server' for the local trainer). Keep the
		// latest per device, and only drive the viewer (frame + episode returns) for
		// the followed device — '' means auto-follow whichever device is streaming.
		const dev = (data as SimFrame & { device?: string }).device ?? 'server';
		this.framesByDevice = { ...this.framesByDevice, [dev]: data };
		// Also key by run so the play view can follow its match regardless of device.
		if (data.run) this.framesByRun = { ...this.framesByRun, [data.run]: data };

		const target = this.selectedDevice || dev;
		if (dev !== target) return;

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

	/** Device ids that have streamed at least one frame this session. */
	get streamingDevices(): string[] {
		return Object.keys(this.framesByDevice);
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
