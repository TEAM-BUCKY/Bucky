<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte';
	import SoccerField from './SoccerField.svelte';
	import RewardBreakdown from './RewardBreakdown.svelte';
	import ObservationInspector from './ObservationInspector.svelte';
	import TrainingMetrics from './TrainingMetrics.svelte';
	import TrainingControls from './TrainingControls.svelte';
	import LineChart from './LineChart.svelte';
	import * as Card from '$lib/components/ui/card';
	import { Input } from '$lib/components/ui/input';

	let wsUrl = $state('ws://localhost:8765');

	onMount(() => {
		simulation.connect(wsUrl);
		return () => simulation.disconnect();
	});

	// Physics (m) → field (mm). Field is rendered rotated 90°, and physics x is
	// forward / y is lateral, so map row→y and col→x as the original viewer did.
	const SCALE_X = 610 / 0.9;
	const SCALE_Y = 915 / 1.2;

	const fieldData = $derived(
		simulation.frame
			? {
					allies: [
						{
							x: simulation.frame.robot_pos[1] * SCALE_X,
							y: simulation.frame.robot_pos[0] * SCALE_Y,
							theta: simulation.frame.robot_heading - Math.PI / 2
						}
					],
					ball: {
						x: simulation.frame.ball_pos[1] * SCALE_X,
						y: simulation.frame.ball_pos[0] * SCALE_Y
					}
				}
			: { allies: undefined, ball: undefined }
	);

	// ── Status presentation ──────────────────────────────────────────────────
	const hub = $derived.by(() => {
		if (simulation.live) return { label: 'Hub live', color: '#34d399', pulse: true };
		if (simulation.connected) return { label: 'Hub idle', color: '#fbbf24', pulse: false };
		return { label: 'Reconnecting…', color: '#f87171', pulse: true };
	});

	const TRAIN_COLORS: Record<string, string> = {
		idle: '#94a3b8',
		launching: '#fbbf24',
		running: '#34d399',
		stopping: '#fbbf24',
		exited: '#5a8cff',
		unknown: '#94a3b8'
	};
	const train = $derived.by(() => {
		const s = simulation.status;
		const color = TRAIN_COLORS[s.state] ?? '#94a3b8';
		let label: string = s.state;
		if (s.state === 'exited') label = `exited${s.code != null ? ` (${s.code})` : ''}`;
		return { label, color, pulse: s.state === 'running' || s.state === 'launching' };
	});

	// Live frames carry the freshest count; status only updates at lifecycle edges,
	// so take whichever is larger to keep the odometer moving during a run.
	const numTimesteps = $derived(
		Math.max(simulation.frame?.num_timesteps ?? 0, simulation.status.num_timesteps ?? 0)
	);
	const totalTimesteps = $derived(simulation.status.timesteps ?? 0);
	const progress = $derived(
		totalTimesteps > 0 ? Math.min(numTimesteps / totalTimesteps, 1) : 0
	);

	// Curated metric charts in display order; only those received are shown.
	const CHARTS: { key: string; label: string; color: string }[] = [
		{ key: 'rollout/ep_rew_mean', label: 'ep reward mean', color: '#34d399' },
		{ key: 'train/loss', label: 'loss', color: '#f87171' },
		{ key: 'train/approx_kl', label: 'approx kl', color: '#fbbf24' },
		{ key: 'train/entropy_loss', label: 'entropy', color: '#a78bfa' },
		{ key: 'train/value_loss', label: 'value loss', color: '#22d3ee' },
		{ key: 'train/policy_gradient_loss', label: 'pg loss', color: '#5a8cff' },
		{ key: 'train/clip_fraction', label: 'clip frac', color: '#f0abfc' },
		{ key: 'rollout/ep_len_mean', label: 'ep len mean', color: '#94a3b8' }
	];
	const activeCharts = $derived(CHARTS.filter((c) => (simulation.metrics[c.key]?.length ?? 0) > 0));

	function fmtInt(n: number) {
		return n.toLocaleString('en-US');
	}
</script>

<div class="flex h-full flex-col gap-4 overflow-y-auto p-4">
	<!-- ── Command bar ─────────────────────────────────────────────────────── -->
	<Card.Root>
		<Card.Content class="px-4 py-3">
			<div class="flex flex-wrap items-center justify-between gap-3">
				<div class="flex items-baseline gap-3">
					<h1 class="font-mono text-base font-bold tracking-tight text-foreground">
						BUCKY<span class="text-muted-foreground">·</span>RL
					</h1>
					<span class="font-mono text-xs text-muted-foreground">
						{simulation.status.run_name ?? 'no active run'}
						{#if simulation.status.phase}<span class="text-foreground/70"> · {simulation.status.phase}</span>{/if}
					</span>
				</div>

				<div class="flex items-center gap-2">
					<!-- status pills — kept custom: dynamic runtime colours -->
					<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
						<span
							class="size-2 rounded-full {hub.pulse ? 'animate-pulse' : ''}"
							style="background: {hub.color}"
						></span>
						<span class="font-mono text-[11px] text-muted-foreground">{hub.label}</span>
					</span>
					<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
						<span
							class="size-2 rounded-full {train.pulse ? 'animate-pulse' : ''}"
							style="background: {train.color}"
						></span>
						<span class="font-mono text-[11px] capitalize text-muted-foreground">{train.label}</span>
					</span>

					<Input
						type="text"
						bind:value={wsUrl}
						onkeydown={(e) => e.key === 'Enter' && simulation.connect(wsUrl)}
						spellcheck={false}
						class="hidden w-44 font-mono text-[11px] text-muted-foreground sm:block"
					/>
				</div>
			</div>

			<!-- timestep odometer — progress bar kept custom for dynamic train.color -->
			<div class="mt-2.5 flex items-center gap-3">
				<div class="font-mono text-xs tabular-nums text-foreground">
					{fmtInt(numTimesteps)}<span class="text-muted-foreground">{totalTimesteps ? ` / ${fmtInt(totalTimesteps)}` : ''} steps</span>
				</div>
				<div class="h-1.5 flex-1 overflow-hidden rounded-full bg-muted">
					<div
						class="h-full rounded-full transition-[width] duration-500"
						style="width: {progress * 100}%; background: {train.color}"
					></div>
				</div>
				<div class="w-10 text-right font-mono text-[11px] tabular-nums text-muted-foreground">
					{(progress * 100).toFixed(0)}%
				</div>
			</div>
		</Card.Content>
	</Card.Root>

	<!-- ── Main: controls · field · telemetry ──────────────────────────────── -->
	<div class="grid grid-cols-1 items-start gap-4 lg:grid-cols-[18rem_minmax(0,1fr)_20rem]">
		<div class="flex flex-col gap-4">
			<TrainingControls />
		</div>

		<Card.Root class="flex h-[58vh] min-w-0 items-center justify-center overflow-hidden p-3">
			<SoccerField
				fit
				allies={fieldData.allies ?? []}
				enemies={[]}
				ball={fieldData.ball}
				showBall={!!simulation.frame}
				rotation={90}
				class="border-0"
			/>
		</Card.Root>

		<div class="flex flex-col gap-4">
			<TrainingMetrics frame={simulation.frame} episodeReturns={simulation.episodeReturns} />
			<RewardBreakdown
				terms={simulation.frame?.reward_terms ?? null}
				total={simulation.frame?.reward_total ?? null}
			/>
			<ObservationInspector frame={simulation.frame} />
		</div>
	</div>

	<!-- ── Training scalars ─────────────────────────────────────────────────── -->
	<Card.Root>
		<Card.Header>
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Training scalars
			</Card.Title>
		</Card.Header>
		<Card.Content>
			{#if activeCharts.length}
				<div class="grid grid-cols-2 gap-2.5 md:grid-cols-3 xl:grid-cols-4">
					{#each activeCharts as c (c.key)}
						<LineChart label={c.label} points={simulation.metrics[c.key]} color={c.color} />
					{/each}
				</div>
			{:else}
				<p class="font-mono text-xs text-muted-foreground">
					Scalars appear after the first PPO update completes.
				</p>
			{/if}
		</Card.Content>
	</Card.Root>
</div>
