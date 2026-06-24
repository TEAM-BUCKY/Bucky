<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import SoccerField from '../SoccerField.svelte';
	import RewardBreakdown from './panels/RewardBreakdown.svelte';
	import ObservationInspector from './panels/ObservationInspector.svelte';
	import TrainingMetrics from './panels/TrainingMetrics.svelte';
	import Controls from './VisualizerControls.svelte';
	import QueuePanel from './QueuePanel.svelte';
	import ActiveRunsPanel from './ActiveRunsPanel.svelte';
	import ManualControl from './ManualControl.svelte';
	import LoginControl from './LoginControl.svelte';
	import * as Card from '$lib/components/ui/card';
	import DialogsState from '$lib/state/dialog.svelte.js';
	import TrainingScalarsDialog from "$lib/components/dialog/simulation/TrainingScalarsDialog.svelte";
	import {Button} from "$lib/components/ui/button";
	import { ChartLine, Boxes, Gamepad2, FlaskConical } from '@lucide/svelte';


	onMount(() => {
		simulation.connect();
		return () => simulation.disconnect();
	});

	// Physics (m) → field (mm). The physics now uses the official RoboCup playfield
	// (1.83 m goal-to-goal × 1.22 m), so the scale is uniform 1000 mm/m (no distortion).
	// Field is rendered rotated 90°: screen X = physics y (lateral, ±0.61 m → ±610 mm),
	// screen Y = physics x (goal axis, ±0.915 m → ±915 mm).
	const SCALE_X = 1000;
	const SCALE_Y = 1000;

	// Axes are swapped (screen X = physics y, screen Y = physics x), a reflection, so
	// heading maps as (pi/2 - h). Robot B (match/self-play frames) renders as an enemy.
	const fieldData = $derived.by(() => {
		const f = simulation.frame;
		if (!f) return { allies: undefined, enemies: [], ball: undefined };
		const allies = [
			{ x: f.robot_pos[1] * SCALE_X, y: f.robot_pos[0] * SCALE_Y, theta: Math.PI / 2 - f.robot_heading }
		];
		const enemies = f.robot2_pos
				? [
					{
						x: f.robot2_pos[1] * SCALE_X,
						y: f.robot2_pos[0] * SCALE_Y,
						theta: Math.PI / 2 - (f.robot2_heading ?? 0)
					}
				]
				: [];
		const ball = { x: f.ball_pos[1] * SCALE_X, y: f.ball_pos[0] * SCALE_Y };
		return { allies, enemies, ball };
	});

	const score = $derived(simulation.frame?.score ?? null);

	// ── Match state (clock / half / referee status) ───────────────────────────
	function fmtClock(sec: number) {
		const s = Math.max(0, Math.floor(sec));
		return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
	}
	const matchInfo = $derived.by(() => {
		const f = simulation.frame;
		if (!f || f.mode !== 'play' || f.clock == null) return null;
		const half = f.match_over ? 'FULL TIME' : `1ST HALF`.replace('1ST', f.half === 2 ? '2ND' : '1ST');
		return { clock: fmtClock(f.clock), half, over: !!f.match_over };
	});
	function statusLabel(s: { suspended: boolean; defective: boolean; penalty_remaining: number } | undefined) {
		if (!s || (!s.suspended && !s.defective)) return null;
		const kind = s.defective ? 'DEFECTIVE' : 'SUSPENDED';
		return `${kind} ${Math.ceil(s.penalty_remaining)}s`;
	}
	const statusA = $derived(statusLabel(simulation.frame?.status?.a));
	const statusB = $derived(statusLabel(simulation.frame?.status?.b));

	// Live streams by source device — lets the viewer follow one when several
	// devices (the server + guest workers) train concurrently. '' = auto (latest).
	const deviceName = (id: string) =>
		id === 'server' ? 'Server' : (simulation.devices.find((d) => d.id === id)?.name ?? id);
	const streamOptions = $derived(
		simulation.streamingDevices.map((id) => ({ id, label: deviceName(id) }))
	);

	const conn = $derived.by(() => {
		if (simulation.live) return { label: 'Live', color: '#34d399', pulse: true };
		if (simulation.connected) return { label: 'Connected', color: '#fbbf24', pulse: false };
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

	// A run stops either by step count or by wall-clock (duration/until). Time-based runs
	// have no step target, so progress + remaining are measured against the deadline.
	const isTimeBased = $derived(
		(simulation.status.stop_kind === 'duration' || simulation.status.stop_kind === 'until') &&
			!!simulation.status.deadline
	);
	const progress = $derived.by(() => {
		const s = simulation.status;
		if (isTimeBased && s.started_at && s.deadline) {
			const total = s.deadline - s.started_at;
			if (total <= 0) return 1;
			const nowSec = simulation.now ? simulation.now / 1000 : Date.now() / 1000;
			const ref = s.state === 'exited' ? (s.ended_at ?? nowSec) : nowSec;
			return Math.min(Math.max((ref - s.started_at) / total, 0), 1);
		}
		return totalTimesteps > 0 ? Math.min(numTimesteps / totalTimesteps, 1) : 0;
	});


	// Elapsed / remaining wall-clock time. `started_at`/`ended_at`/`deadline` come from the
	// backend (epoch seconds), so elapsed survives reloads. For time-based runs remaining is
	// exact (deadline − now); for step-based runs it's extrapolated from step progress.
	function fmtDuration(s: number) {
		if (!isFinite(s) || s < 0) return '—';
		s = Math.floor(s);
		const h = Math.floor(s / 3600);
		const m = Math.floor((s % 3600) / 60);
		const sec = s % 60;
		const pad = (n: number) => String(n).padStart(2, '0');
		return h > 0 ? `${h}:${pad(m)}:${pad(sec)}` : `${m}:${pad(sec)}`;
	}
	const timing = $derived.by(() => {
		const s = simulation.status;
		if (!s.started_at) return null;
		const active = s.state === 'running' || s.state === 'launching' || s.state === 'stopping';
		const nowSec = simulation.now ? simulation.now / 1000 : Date.now() / 1000;
		const endRef = active ? nowSec : (s.ended_at ?? nowSec);
		const elapsed = Math.max(0, endRef - s.started_at);
		let remaining: number | null = null;
		if (active && isTimeBased && s.deadline) {
			remaining = Math.max(0, s.deadline - nowSec);
		} else if (active && progress > 0 && progress < 1 && elapsed > 1) {
			remaining = (elapsed * (1 - progress)) / progress;
		}
		return { elapsed, remaining };
	});

	let mode = $state<'train' | 'match'>(simulation.status.run_type || 'train');
	// Field DOM wrapper — ManualControl reads the rendered <svg> from it to map the cursor to
	// the field and to capture pointer aiming/kicks.
	let fieldEl = $state<HTMLElement | null>(null);
	const manualRun = $derived(simulation.manualMatchRun);

	$effect(() => {
		if (simulation.status.phase && simulation.status.phase != "done" && simulation.status.run_type)
			mode = simulation.status.run_type;
	});

	function fmtInt(n: number) {
		return n.toLocaleString('en-US');
	}
</script>

<div class="flex h-full flex-col gap-4 p-4">
	<Card.Root>
		<Card.Content class="px-4 py-3">
			<div class="flex flex-wrap items-center justify-between gap-3">
				<div class="flex items-baseline gap-3">
					<h1 class="font-mono text-base font-bold tracking-tight text-foreground">
						BUCKY
					</h1>
					<span class="font-mono text-xs text-muted-foreground">
						{simulation.status.run_name ?? 'no active run'}
						{#if simulation.status.phase}<span class="text-foreground/70"> · {simulation.status.phase}</span>{/if}
					</span>
				</div>

				<div class="flex items-center gap-2">
					<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
						<span
								class="size-2 rounded-full {conn.pulse ? 'animate-pulse' : ''}"
								style="background: {conn.color}"
						></span>
						<span class="font-mono text-[11px] text-muted-foreground">{conn.label}</span>
					</span>
					<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
						<span
								class="size-2 rounded-full {train.pulse ? 'animate-pulse' : ''}"
								style="background: {train.color}"
						></span>
						<span class="font-mono text-[11px] capitalize text-muted-foreground">{train.label}</span>
					</span>

					{#if streamOptions.length > 1}
						<select
							bind:value={simulation.selectedDevice}
							class="h-7 rounded-md border border-border bg-background/60 px-2 font-mono text-[11px] text-muted-foreground outline-none"
							title="Which device's live stream to watch"
						>
							<option value="">Auto (latest)</option>
							{#each streamOptions as o (o.id)}
								<option value={o.id}>{o.label}</option>
							{/each}
						</select>
					{/if}

					<Button href="/eval" variant="ghost" size="xs" class="font-mono text-[11px]">
						<FlaskConical class="size-3" />Eval
					</Button>
					<Button href="/play" variant="ghost" size="xs" class="font-mono text-[11px]">
						<Gamepad2 class="size-3" />Play a friend
					</Button>
					<Button href="/overview" variant="ghost" size="xs" class="font-mono text-[11px]">
						<Boxes class="size-3" />Overview
					</Button>
					<LoginControl />
				</div>
			</div>

			<div class="mt-2.5 flex items-center gap-3">
				<div class="font-mono text-xs tabular-nums text-foreground">
					{fmtInt(numTimesteps)}<span class="text-muted-foreground">{totalTimesteps ? ` / ${fmtInt(totalTimesteps)}` : ''} steps</span>
				</div>
				<div class="h-1.5 flex-1 rounded-full bg-muted">
					<div
							class="h-full rounded-full transition-[width] duration-500"
							style="width: {progress * 100}%; background: {train.color}"
					></div>
				</div>
				<div class="w-10 text-right font-mono text-[11px] tabular-nums text-muted-foreground">
					{(progress * 100).toFixed(0)}%
				</div>
			</div>

			{#if timing}
				<div class="mt-1.5 flex items-center gap-4 font-mono text-[11px] tabular-nums text-muted-foreground">
					<span>elapsed <span class="text-foreground">{fmtDuration(timing.elapsed)}</span></span>
					<span>
						remaining
						<span class="text-foreground">
							{timing.remaining != null ? `~${fmtDuration(timing.remaining)}` : '—'}
						</span>
					</span>
				</div>
			{/if}
		</Card.Content>
	</Card.Root>

	<div class="grid min-h-0 flex-1 grid-cols-1 gap-4 lg:grid-cols-[20rem_minmax(0,1fr)_20rem]">
		<div class="flex flex-col gap-2 lg:min-h-0 lg:overflow-y-auto">
			<div class="shrink-0"><Controls bind:mode={mode} /></div>

			<div class="shrink-0"><ActiveRunsPanel /></div>

			<div class="shrink-0"><QueuePanel /></div>
		</div>

		<div class="flex min-w-0 flex-col gap-4 lg:min-h-0">
			<Card.Root class="relative mx-auto flex aspect-[303/242] w-full max-w-2xl shrink-0 items-center justify-center p-3">
				{#if score}
					<div class="absolute left-1/2 top-3 z-10 flex -translate-x-1/2 flex-col items-center gap-1">
						<div class="flex items-center gap-3 rounded-full border border-border bg-background/80 px-4 py-1.5 backdrop-blur">
							<span class="font-mono text-[10px] uppercase tracking-wider" style="color:#3c78dc">A</span>
							<span class="font-mono text-lg font-bold tabular-nums" style="color:#3c78dc">{score.a}</span>
							<span class="font-mono text-sm text-muted-foreground">:</span>
							<span class="font-mono text-lg font-bold tabular-nums" style="color:#dc3c3c">{score.b}</span>
							<span class="font-mono text-[10px] uppercase tracking-wider" style="color:#dc3c3c">B</span>
							{#if matchInfo}
								<span class="ml-2 border-l border-border pl-3 font-mono text-sm font-bold tabular-nums text-foreground">{matchInfo.clock}</span>
								<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">{matchInfo.half}</span>
							{/if}
						</div>
						{#if statusA || statusB}
							<div class="flex items-center gap-2">
								{#if statusA}
									<span class="rounded-full bg-amber-500/90 px-2 py-0.5 font-mono text-[9px] font-bold uppercase tracking-wider text-black">A {statusA}</span>
								{/if}
								{#if statusB}
									<span class="rounded-full bg-amber-500/90 px-2 py-0.5 font-mono text-[9px] font-bold uppercase tracking-wider text-black">B {statusB}</span>
								{/if}
							</div>
						{/if}
					</div>
				{/if}
				<div
					bind:this={fieldEl}
					class="flex h-full w-full items-center justify-center {manualRun ? 'cursor-crosshair' : ''}"
				>
					<SoccerField
							fit
							allies={fieldData.allies ?? []}
							enemies={fieldData.enemies ?? []}
							ball={fieldData.ball}
							showBall={!!simulation.frame}
							rotation={90}
							class="border-0"
					/>
				</div>
			</Card.Root>


		</div>

		{#if mode === 'train'}
			<div class="flex flex-col gap-4 lg:min-h-0 w-full">
				<Button
						variant="outline"
						size="sm"
						class="shrink-0 font-mono text-[11px]"
						onclick={async () => await DialogsState.open({
						component: TrainingScalarsDialog,
						data: {}
					})}
				>
					<ChartLine class="size-3.5" />Open training scalars
				</Button>

				<TrainingMetrics frame={simulation.frame} episodeReturns={simulation.episodeReturns} />

				<RewardBreakdown
						terms={simulation.frame?.reward_terms ?? null}
						total={simulation.frame?.reward_total ?? null}
				/>
				<ObservationInspector frame={simulation.frame} />
			</div>
		{:else if mode === 'match'}
			<div class="flex flex-col gap-4 lg:min-h-0 w-full">
				{#if manualRun}
					<ManualControl {fieldEl} />
				{:else}
					<p class="font-mono text-[11px] text-muted-foreground">
						Tick "Drive yourself" in the match config to control red against the AI.
					</p>
				{/if}
			</div>
		{/if}
	</div>
</div>
