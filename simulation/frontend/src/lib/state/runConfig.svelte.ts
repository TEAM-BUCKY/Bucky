import type {
	Hyperparams,
	LaunchConfig,
	MatchConfig,
	StopCondition,
	StopKind
} from './simulation.svelte.js';
import { apiBases } from './simulation.svelte.js';

/** Which feature groups a host form exposes (drives chips + payload contents). */
export interface RunConfigCaps {
	/** Train/Match mode toggle + match-mode chips (live page). */
	match?: boolean;
	/** Net architecture, PPO hyperparams, reward weights, step checkpoints. */
	advanced?: boolean;
	/** Named/versioned model identity (overview "create"). */
	identity?: boolean;
}

export const STAGES = [{ value: 'SELF_PLAY_1V1', label: 'Self-play 1v1' }];

export const STOP_TABS: { value: StopKind; label: string }[] = [
	{ value: 'steps', label: 'Steps' },
	{ value: 'duration', label: 'Duration' },
	{ value: 'until', label: 'Until' }
];

export const HP_FIELDS: { key: keyof Hyperparams; label: string; step: number }[] = [
	{ key: 'learning_rate', label: 'learning rate', step: 0.0001 },
	{ key: 'n_steps', label: 'n_steps', step: 128 },
	{ key: 'batch_size', label: 'batch size', step: 64 },
	{ key: 'n_epochs', label: 'n_epochs', step: 1 },
	{ key: 'gamma', label: 'gamma', step: 0.001 },
	{ key: 'gae_lambda', label: 'gae_lambda', step: 0.001 },
	{ key: 'clip_range', label: 'clip range', step: 0.01 },
	{ key: 'ent_coef', label: 'ent_coef', step: 0.001 },
	{ key: 'vf_coef', label: 'vf_coef', step: 0.05 },
	{ key: 'max_grad_norm', label: 'max grad norm', step: 0.05 }
];

// Pretty labels for known reward weights. The set of weights and their default values comes
// from the backend (the Python ``RewardConfig`` — single source of truth, see loadRewardDefaults).
// Any weight the backend returns that isn't listed here still shows up, with a label derived
// from its key, so adding a weight in rewards.py needs no frontend change.
const REWARD_LABELS: Record<string, string> = {
	w_approach: 'approach',
	w_speed: 'speed',
	w_ball_to_goal: 'ball→goal',
	w_possession: 'possession',
	w_front_align: 'front align',
	w_goal: 'goal',
	w_goal_against: 'goal against',
	w_out_of_bounds: 'out of bounds',
	w_lack_of_progress: 'lack of progress',
	w_defective: 'defective',
	w_spin: 'spin',
	// Skilled play (kicker + opponent-aware) — let the policy learn to shoot, not just dribble.
	w_steal: 'steal',
	w_blocked_shot: 'blocked shot',
	w_kick_goal: 'kick goal',
	w_bank_shot: 'bank shot',
	w_risky_shot: 'risky shot',
	w_kick_lost: 'kick lost',
	w_kick_at_opponent: 'kick at opponent',
	w_kick_attempt: 'kick attempt',
	w_kick_power_to_goal: 'kick power→goal',
	w_shot_on_goal: 'shot on goal',
	w_time: 'time',
	w_action_smooth: 'action smooth'
};

function rewardLabel(key: string): string {
	return REWARD_LABELS[key] ?? key.replace(/^w_/, '').replace(/_/g, ' ');
}

/**
 * Reward-weight defaults + ordered field list, loaded from the backend ``RewardConfig`` so the
 * code is the single source of truth. Until {@link loadRewardDefaults} resolves, ``defaults`` is
 * empty and ``fields`` falls back to the labelled keys above so the editor still renders.
 */
export const rewardState = $state<{
	defaults: Record<string, number>;
	fields: { key: string; label: string }[];
}>({
	defaults: {},
	fields: Object.keys(REWARD_LABELS).map((key) => ({ key, label: rewardLabel(key) }))
});

let _rewardDefaultsLoaded = false;
/**
 * Fetch the reward-weight defaults from the backend once and refresh {@link rewardState}.
 * Idempotent; failures keep the fallbacks. Returns the loaded defaults (or {} on failure).
 */
export async function loadRewardDefaults(): Promise<Record<string, number>> {
	if (_rewardDefaultsLoaded) return rewardState.defaults;
	try {
		const { httpBase } = apiBases();
		const res = await fetch(httpBase + '/reward-defaults');
		if (!res.ok) return rewardState.defaults;
		const data = await res.json();
		const weights = (data?.weights ?? {}) as Record<string, number>;
		if (Object.keys(weights).length === 0) return rewardState.defaults;
		rewardState.defaults = weights;
		rewardState.fields = Object.keys(weights).map((key) => ({ key, label: rewardLabel(key) }));
		_rewardDefaultsLoaded = true;
	} catch {
		/* keep fallbacks */
	}
	return rewardState.defaults;
}

function fmtCount(n: number): string {
	if (!Number.isFinite(n)) return '0';
	if (n >= 1_000_000) return `${+(n / 1_000_000).toFixed(2)}M`;
	if (n >= 1_000) return `${+(n / 1_000).toFixed(n % 1_000 === 0 ? 0 : 1)}k`;
	return String(n);
}

function fmtTime(epoch: number | null): string {
	if (epoch == null) return 'ASAP';
	return new Date(epoch * 1000).toLocaleString([], {
		month: 'short',
		day: 'numeric',
		hour: '2-digit',
		minute: '2-digit'
	});
}

/**
 * The single source of truth for a training/match run configuration, shared by the
 * live page (VisualizerControls) and the overview "create" form (ModelCreateForm).
 * Holds all fields, derives validity, builds the launch/queue payloads, and exposes
 * per-chip summary strings for the chip-based UI.
 */
export class RunConfig {
	caps: RunConfigCaps;

	/** 'train' = single-agent training; 'match' = two saved policies head-to-head. */
	mode = $state<'train' | 'match'>('train');

	// ── basics ─────────────────────────────────────────────────────────────────
	stage = $state('SELF_PLAY_1V1');
	timesteps = $state(200000);
	n_envs = $state(16);
	seed = $state(0);
	domain_rand = $state(true);
	/** Animate the live field while training. Off by default — headless trains faster. */
	viz = $state(false);

	// ── distributed (FedAvg across devices) ──────────────────────────────────────
	/** Split this run into shards that train together and average weights periodically. */
	distributed = $state(false);
	distShards = $state(2);
	distSyncEvery = $state(50000);
	/**
	 * Per-device assignment: each enabled device runs one shard at its own env count
	 * (target = device id or 'server'). When ≥2 entries are set, it overrides even
	 * shards so heterogeneous boxes each train at their capacity.
	 */
	distDevices = $state<{ target: string; n_envs: number }[]>([]);

	// ── stop condition ───────────────────────────────────────────────────────────
	stopKind = $state<StopKind>('steps');
	durH = $state(2);
	durM = $state(0);
	untilEpoch = $state<number | null>(null);

	/** Optional scheduled start for queued runs (epoch seconds); null = ASAP. */
	startEpoch = $state<number | null>(null);

	// ── resume ───────────────────────────────────────────────────────────────────
	cont = $state(false);
	srcRun = $state('');
	srcCkpt = $state('');

	/** Where the run executes: 'any', 'server', or a device id. */
	target = $state('any');

	// ── match mode ───────────────────────────────────────────────────────────────
	runA = $state('');
	ckptA = $state('');
	runB = $state('');
	ckptB = $state('');
	matchSeed = $state(0);
	/** Human-controlled red robot (test mode): drive red yourself against blue's AI. */
	manualRed = $state(false);

	// ── identity (caps.identity) ──────────────────────────────────────────────────
	name = $state('');
	version = $state('1');

	// ── advanced (caps.advanced) ──────────────────────────────────────────────────
	saveStepCheckpoints = $state(false);
	netArchStr = $state('64,64');
	hp = $state<Required<Hyperparams>>({
		learning_rate: 0.0003,
		n_steps: 1024,
		batch_size: 512,
		n_epochs: 10,
		gamma: 0.99,
		gae_lambda: 0.95,
		clip_range: 0.2,
		ent_coef: 0.01,
		vf_coef: 0.5,
		max_grad_norm: 0.5
	});
	/**
	 * Reward weights, seeded from the backend defaults ({@link rewardState}). Empty until the
	 * defaults load; call {@link syncRewardDefaults} (or rely on ModelCreateForm doing so on
	 * mount) to fill it. An empty payload makes the backend fall back to the code defaults, so
	 * the code stays the single source of truth even if the fetch hasn't completed.
	 */
	rw = $state<Record<string, number>>({ ...rewardState.defaults });

	constructor(caps: RunConfigCaps = {}) {
		this.caps = caps;
	}

	/** (Re)seed reward weights from the backend-loaded defaults — discards local edits. */
	syncRewardDefaults() {
		this.rw = { ...rewardState.defaults };
	}

	// ── builders ───────────────────────────────────────────────────────────────
	buildStop(): StopCondition | null {
		if (this.stopKind === 'steps') {
			return this.timesteps > 0 ? { kind: 'steps', value: this.timesteps } : null;
		}
		if (this.stopKind === 'duration') {
			const secs = (Number(this.durH) || 0) * 3600 + (Number(this.durM) || 0) * 60;
			return secs >= 1 ? { kind: 'duration', value: secs } : null;
		}
		return this.untilEpoch != null && this.untilEpoch * 1000 > Date.now()
			? { kind: 'until', value: this.untilEpoch }
			: null;
	}

	parseNetArch(): number[] {
		return this.netArchStr
			.split(',')
			.map((s) => parseInt(s.trim(), 10))
			.filter((n) => Number.isInteger(n) && n > 0);
	}

	trainPayload(): LaunchConfig {
		const p: LaunchConfig = {
			stage: this.stage,
			timesteps: this.timesteps,
			n_envs: this.n_envs,
			seed: this.seed,
			domain_rand: this.domain_rand,
			stop: this.buildStop() ?? undefined,
			target: this.target,
			viz: this.viz
		};
		if (this.distributed) {
			const sync_every = Math.max(1, Math.floor(this.distSyncEvery));
			const perDevice = this.distDevices.filter((d) => d.target && d.n_envs >= 1);
			if (perDevice.length > 1) {
				p.distributed = {
					sync_every,
					devices: perDevice.map((d) => ({ target: d.target, n_envs: Math.floor(d.n_envs) }))
				};
			} else if (this.distShards > 1) {
				p.distributed = { shards: Math.max(2, Math.floor(this.distShards)), sync_every };
			}
		}
		if (this.cont && this.srcRun && this.srcCkpt) {
			p.resume_from = { run: this.srcRun, checkpoint: this.srcCkpt };
		}
		if (this.caps.identity) {
			p.name = this.name.trim();
			p.version = this.version.trim();
		}
		if (this.caps.advanced) {
			p.hyperparams = { ...this.hp };
			p.net_arch = this.parseNetArch();
			p.reward_weights = { ...this.rw };
			p.save_step_checkpoints = this.saveStepCheckpoints;
		}
		return p;
	}

	matchConfig(): MatchConfig {
		return {
			policyA: { run: this.runA, checkpoint: this.ckptA },
			policyB: { run: this.runB, checkpoint: this.ckptB },
			seed: this.matchSeed,
			manualRed: this.manualRed
		};
	}

	// ── validity ─────────────────────────────────────────────────────────────────
	get stopValid(): boolean {
		return this.mode === 'match' ? true : this.buildStop() !== null;
	}
	get archValid(): boolean {
		return !this.caps.advanced || this.parseNetArch().length > 0;
	}
	get nameValid(): boolean {
		return !this.caps.identity || (this.name.trim().length > 0 && this.version.trim().length > 0);
	}
	get configValid(): boolean {
		if (this.mode === 'match') {
			return !!this.runA && !!this.ckptA && !!this.runB && !!this.ckptB;
		}
		return (
			this.nameValid &&
			this.stopValid &&
			this.archValid &&
			(!this.cont || (!!this.srcRun && !!this.srcCkpt))
		);
	}
	/** Immediate "Launch now" only runs in-process on the server. */
	get targetIsRemote(): boolean {
		return this.mode === 'train' && this.target !== 'any' && this.target !== 'server';
	}

	// ── chip summaries ─────────────────────────────────────────────────────────
	get stageSummary(): string {
		return STAGES.find((s) => s.value === this.stage)?.label ?? this.stage;
	}
	get stopSummary(): string {
		if (this.stopKind === 'steps') return `${fmtCount(this.timesteps)} steps`;
		if (this.stopKind === 'duration') return `${this.durH || 0}h ${this.durM || 0}m`;
		return this.untilEpoch != null ? `until ${fmtTime(this.untilEpoch)}` : 'until …';
	}
	get parallelismSummary(): string {
		return `${this.n_envs} envs · seed ${this.seed}`;
	}
	/** Enabled per-device shard assignments (target set, ≥1 env). */
	get distAssignments(): { target: string; n_envs: number }[] {
		return this.distDevices.filter((d) => d.target && d.n_envs >= 1);
	}
	get distAssignmentsSummary(): string {
		const a = this.distAssignments;
		return `${a.length} devices · ${a.map((d) => d.n_envs).join('+')} envs`;
	}
	get optionsSummary(): string {
		const bits = [`domain rand ${this.domain_rand ? 'on' : 'off'}`];
		if (this.viz) bits.push('watch live');
		if (this.distributed) {
			bits.push(
				this.distAssignments.length > 1
					? this.distAssignmentsSummary
					: `${Math.max(2, this.distShards)} shards`
			);
		}
		if (this.caps.advanced && this.saveStepCheckpoints) bits.push('step ckpts');
		return bits.join(' · ');
	}
	get resumeSummary(): string {
		if (!this.cont || !this.srcRun) return 'off';
		return `${this.srcRun} / ${this.srcCkpt.replace(/\.zip$/, '')}`;
	}
	get scheduleSummary(): string {
		return fmtTime(this.startEpoch);
	}
	get advancedSummary(): string {
		return `arch ${this.parseNetArch().join('·') || '—'} · PPO`;
	}
	get rewardsSummary(): string {
		return `${rewardState.fields.length} weights`;
	}
	get botASummary(): string {
		return this.runA ? `${this.runA} / ${this.ckptA.replace(/\.zip$/, '')}` : 'pick a policy';
	}
	get botBSummary(): string {
		return this.runB ? `${this.runB} / ${this.ckptB.replace(/\.zip$/, '')}` : 'pick a policy';
	}
}
