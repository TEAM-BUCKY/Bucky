import type {
	Hyperparams,
	LaunchConfig,
	MatchConfig,
	StopCondition,
	StopKind
} from './simulation.svelte.js';

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

export const REWARD_FIELDS: { key: string; label: string }[] = [
	{ key: 'w_approach', label: 'approach' },
	{ key: 'w_ball_to_goal', label: 'ball→goal' },
	{ key: 'w_possession', label: 'possession' },
	{ key: 'w_front_align', label: 'front align' },
	{ key: 'w_goal', label: 'goal' },
	{ key: 'w_goal_against', label: 'goal against' },
	{ key: 'w_out_of_bounds', label: 'out of bounds' },
	{ key: 'w_lack_of_progress', label: 'lack of progress' },
	{ key: 'w_defective', label: 'defective' },
	{ key: 'w_spin', label: 'spin' },
	// Skilled play (kicker + opponent-aware) — let the policy learn to shoot, not just dribble.
	{ key: 'w_steal', label: 'steal' },
	{ key: 'w_blocked_shot', label: 'blocked shot' },
	{ key: 'w_kick_goal', label: 'kick goal' },
	{ key: 'w_bank_shot', label: 'bank shot' },
	{ key: 'w_risky_shot', label: 'risky shot' },
	{ key: 'w_kick_lost', label: 'kick lost' },
	{ key: 'w_time', label: 'time' },
	{ key: 'w_action_mag', label: 'action mag' }
];

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
	rw = $state<Record<string, number>>({
		w_approach: 0.5,
		w_ball_to_goal: 2.5,
		w_possession: 1,
		w_front_align: 0.3,
		w_goal: 20,
		w_goal_against: -20,
		w_out_of_bounds: -10,
		w_lack_of_progress: -2,
		w_defective: -10,
		w_spin: -0.2,
		// Skilled play (kicker + opponent-aware) — defaults mirror the backend RewardConfig.
		w_steal: 3,
		w_blocked_shot: 5,
		w_kick_goal: 6,
		w_bank_shot: 4,
		w_risky_shot: 2,
		w_kick_lost: -6,
		w_time: -0.001,
		w_action_mag: -0.005
	});

	constructor(caps: RunConfigCaps = {}) {
		this.caps = caps;
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
		if (this.distributed && this.distShards > 1) {
			p.distributed = {
				shards: Math.max(2, Math.floor(this.distShards)),
				sync_every: Math.max(1, Math.floor(this.distSyncEvery))
			};
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
			seed: this.matchSeed
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
	get optionsSummary(): string {
		const bits = [`domain rand ${this.domain_rand ? 'on' : 'off'}`];
		if (this.viz) bits.push('watch live');
		if (this.distributed) bits.push(`${Math.max(2, this.distShards)} shards`);
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
		return `${REWARD_FIELDS.length} weights`;
	}
	get botASummary(): string {
		return this.runA ? `${this.runA} / ${this.ckptA.replace(/\.zip$/, '')}` : 'pick a policy';
	}
	get botBSummary(): string {
		return this.runB ? `${this.runB} / ${this.ckptB.replace(/\.zip$/, '')}` : 'pick a policy';
	}
}
