<script lang="ts">
	import {
		simulation,
		type LaunchConfig,
		type StopCondition,
		type StopKind
	} from '$lib/state/simulation.svelte.js';
	import { Button } from '$lib/components/ui/button';
	import * as Card from '$lib/components/ui/card';
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import DateTimePicker from './DateTimePicker.svelte';
	import { Play, Square, Loader2, CalendarPlus } from '@lucide/svelte';

	const STAGES = [
		{ value: 'APPROACH_STATIC_BALL', label: 'Approach static ball', stub: false },
		{ value: 'PUSH_TO_EMPTY_GOAL', label: 'Push to empty goal', stub: false },
		{ value: 'SELF_PLAY_1V1', label: 'Self-play 1v1', stub: false },
		{ value: 'SCRIPTED_OPPONENT', label: 'Scripted opponent (stub)', stub: true },
		{ value: 'SELF_PLAY_2V2', label: 'Self-play 2v2 (stub)', stub: true }
	];

	let cfg = $state<LaunchConfig>({
		stage: 'APPROACH_STATIC_BALL',
		timesteps: 200000,
		n_envs: 16,
		seed: 0,
		domain_rand: true
	});

	// Stop condition: by steps, by a duration, or until an absolute clock time.
	let stopKind = $state<StopKind>('steps');
	let durH = $state(2);
	let durM = $state(0);
	let untilEpoch = $state<number | null>(null); // epoch seconds

	// Optional scheduled start for queued runs (epoch seconds); null = start ASAP.
	let startEpoch = $state<number | null>(null);

	let cont = $state(false);
	let srcRun = $state('');
	let srcCkpt = $state('');

	/** 'train' = single-agent training; 'match' = run two saved policies against each other. */
	let { mode = $bindable<'train' | 'match'>('train') } = $props();

	// Match-mode policy selections (Bot A vs Bot B).
	let runA = $state('');
	let ckptA = $state('');
	let runB = $state('');
	let ckptB = $state('');
	let matchSeed = $state(0);

	const runs = $derived(simulation.runs);
	const hasRuns = $derived(runs.length > 0);
	const srcCheckpoints = $derived(runs.find((r) => r.run === srcRun)?.checkpoints ?? []);
	const ckptsA = $derived(runs.find((r) => r.run === runA)?.checkpoints ?? []);
	const ckptsB = $derived(runs.find((r) => r.run === runB)?.checkpoints ?? []);

	const pickFinal = (cks: string[]) => cks.find((c) => c === 'final_model.zip') ?? cks[0] ?? '';

	$effect(() => {
		if (!cont || !hasRuns) return;
		if (!runs.some((r) => r.run === srcRun)) srcRun = runs[runs.length - 1].run;
	});
	$effect(() => {
		if (!cont) return;
		if (!srcCheckpoints.includes(srcCkpt)) srcCkpt = pickFinal(srcCheckpoints);
	});
	$effect(() => {
		if (mode !== 'match' || !hasRuns) return;
		if (!runs.some((r) => r.run === runA)) runA = runs[0].run;
		if (!runs.some((r) => r.run === runB)) runB = runs[runs.length - 1].run;
	});
	$effect(() => {
		if (mode !== 'match') return;
		if (!ckptsA.includes(ckptA)) ckptA = pickFinal(ckptsA);
	});
	$effect(() => {
		if (mode !== 'match') return;
		if (!ckptsB.includes(ckptB)) ckptB = pickFinal(ckptsB);
	});

	const busy = $derived(['launching', 'running', 'stopping'].includes(simulation.status.state));

	// Build the stop condition from the current selection (null = invalid input).
	function buildStop(): StopCondition | null {
		if (stopKind === 'steps') {
			return cfg.timesteps > 0 ? { kind: 'steps', value: cfg.timesteps } : null;
		}
		if (stopKind === 'duration') {
			const secs = (Number(durH) || 0) * 3600 + (Number(durM) || 0) * 60;
			return secs >= 1 ? { kind: 'duration', value: secs } : null;
		}
		return untilEpoch != null && untilEpoch * 1000 > Date.now()
			? { kind: 'until', value: untilEpoch }
			: null;
	}

	const stopValid = $derived(mode === 'match' ? true : buildStop() !== null);
	const configValid = $derived(
		mode === 'train'
			? (!cont || (!!srcRun && !!srcCkpt)) && stopValid
			: !!runA && !!ckptA && !!runB && !!ckptB
	);

	const canLaunch = $derived(simulation.connected && !busy && configValid);
	const canQueue = $derived(simulation.connected && configValid);
	const canKill = $derived(
		simulation.connected && ['launching', 'running'].includes(simulation.status.state)
	);

	function trainPayload(): LaunchConfig {
		const payload: LaunchConfig = { ...cfg, stop: buildStop() ?? undefined };
		if (cont && srcRun && srcCkpt) payload.resume_from = { run: srcRun, checkpoint: srcCkpt };
		return payload;
	}

	function launch() {
		if (!canLaunch) return;
		if (mode === 'match') {
			simulation.match({
				policyA: { run: runA, checkpoint: ckptA },
				policyB: { run: runB, checkpoint: ckptB },
				seed: matchSeed
			});
			return;
		}
		simulation.launch(trainPayload());
	}

	function addToQueue() {
		if (!canQueue) return;
		const startAt = startEpoch;
		if (mode === 'match') {
			simulation.enqueue(
				{
					policyA: { run: runA, checkpoint: ckptA },
					policyB: { run: runB, checkpoint: ckptB },
					seed: matchSeed
				},
				startAt
			);
		} else {
			simulation.enqueue(trainPayload(), startAt);
		}
		startEpoch = null;
	}

	const STOP_TABS: { value: StopKind; label: string }[] = [
		{ value: 'steps', label: 'Steps' },
		{ value: 'duration', label: 'Duration' },
		{ value: 'until', label: 'Until' }
	];
</script>

<Card.Root>
	<Card.Header class="pb-3 pt-4">
		<div class="flex items-center justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				{mode === 'match' ? 'Match config' : 'Run config'}
			</Card.Title>
			{#if busy}
				<span class="flex items-center gap-1 font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
					<Loader2 class="size-3 animate-spin" />running
				</span>
			{/if}
		</div>
	</Card.Header>

	<Card.Content class="flex flex-col gap-3">
		<!-- Mode toggle: Train a single agent, or run two saved policies head-to-head. -->
		<div class="grid grid-cols-2 gap-1 rounded-md border border-border/70 bg-input/20 p-1">
			<Button
				variant={mode === 'train' ? 'default' : 'ghost'}
				size="sm"
				class="h-7 font-mono text-[11px] uppercase tracking-wider"
				onclick={() => (mode = 'train')}
			>
				Train
			</Button>
			<Button
				variant={mode === 'match' ? 'default' : 'ghost'}
				size="sm"
				class="h-7 font-mono text-[11px] uppercase tracking-wider"
				onclick={() => (mode = 'match')}
			>
				Match
			</Button>
		</div>

		{#if mode === 'train'}
			<div class="flex flex-col gap-1">
				<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Stage</Label>
				<Combobox bind:value={cfg.stage} items={STAGES} searchPlaceholder="Search stages…" />
			</div>

			<!-- Stop condition -->
			<div class="flex flex-col gap-1.5">
				<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
					Stop after
				</Label>
				<div class="grid grid-cols-3 gap-1 rounded-md border border-border/70 bg-input/20 p-1">
					{#each STOP_TABS as t (t.value)}
						<Button
							variant={stopKind === t.value ? 'default' : 'ghost'}
							size="sm"
							class="h-7 font-mono text-[11px] uppercase tracking-wider"
							onclick={() => (stopKind = t.value)}
						>
							{t.label}
						</Button>
					{/each}
				</div>

				{#if stopKind === 'steps'}
					<Input type="number" min={1000} step={50000} bind:value={cfg.timesteps} class="font-mono" />
				{:else if stopKind === 'duration'}
					<div class="grid grid-cols-2 gap-2">
						<div class="flex items-center gap-1">
							<Input type="number" min={0} bind:value={durH} class="font-mono" />
							<span class="font-mono text-[11px] text-muted-foreground">h</span>
						</div>
						<div class="flex items-center gap-1">
							<Input type="number" min={0} max={59} bind:value={durM} class="font-mono" />
							<span class="font-mono text-[11px] text-muted-foreground">m</span>
						</div>
					</div>
				{:else}
					<DateTimePicker bind:value={untilEpoch} />
					{#if untilEpoch != null && !stopValid}
						<span class="font-mono text-[11px] text-destructive">Pick a time in the future.</span>
					{/if}
				{/if}
			</div>

			<div class="grid grid-cols-2 gap-2">
				<div class="flex flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Envs</Label>
					<Input type="number" min={1} max={32} bind:value={cfg.n_envs} class="font-mono" />
				</div>
				<div class="flex flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</Label>
					<Input type="number" bind:value={cfg.seed} class="font-mono" />
				</div>
			</div>

			<div class="flex items-center gap-2">
				<Checkbox id="domain-rand" bind:checked={cfg.domain_rand} />
				<Label for="domain-rand" class="cursor-pointer font-mono text-xs text-muted-foreground">
					Domain randomization
				</Label>
			</div>

			<div class="flex items-center gap-2 {hasRuns ? '' : 'opacity-50'}" title={hasRuns ? '' : 'No saved checkpoints yet'}>
				<Checkbox id="cont" bind:checked={cont} disabled={!hasRuns} />
				<Label for="cont" class="{hasRuns ? 'cursor-pointer' : ''} font-mono text-xs text-muted-foreground">
					Continue from checkpoint
				</Label>
			</div>

			{#if cont && hasRuns}
				<div class="grid grid-cols-1 gap-2 rounded-md border border-border/70 bg-input/20 p-2">
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Source run</Label>
						<Combobox
							bind:value={srcRun}
							items={runs.map((r) => ({ value: r.run, label: r.run }))}
							size="sm"
							searchPlaceholder="Search runs…"
						/>
					</div>
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Checkpoint</Label>
						<Combobox
							bind:value={srcCkpt}
							items={srcCheckpoints.map((c) => ({ value: c, label: c.replace(/\.zip$/, '') }))}
							size="sm"
							searchPlaceholder="Search checkpoints…"
						/>
					</div>
				</div>
			{/if}
		{/if}

		{#if mode === 'match'}
			{#if hasRuns}
				<!-- Bot A -->
				<div class="grid grid-cols-1 gap-2 rounded-md border border-border/70 bg-input/20 p-2">
					<Label class="font-mono text-[10px] font-semibold uppercase tracking-wider" style="color:#3c78dc">
						Bot A network
					</Label>
					<Combobox
						bind:value={runA}
						items={runs.map((r) => ({ value: r.run, label: r.run }))}
						size="sm"
						searchPlaceholder="Search runs…"
					/>
					<Combobox
						bind:value={ckptA}
						items={ckptsA.map((c) => ({ value: c, label: c.replace(/\.zip$/, '') }))}
						size="sm"
						searchPlaceholder="Search checkpoints…"
					/>
				</div>
				<!-- Bot B -->
				<div class="grid grid-cols-1 gap-2 rounded-md border border-border/70 bg-input/20 p-2">
					<Label class="font-mono text-[10px] font-semibold uppercase tracking-wider" style="color:#dc3c3c">
						Bot B network
					</Label>
					<Combobox
						bind:value={runB}
						items={runs.map((r) => ({ value: r.run, label: r.run }))}
						size="sm"
						searchPlaceholder="Search runs…"
					/>
					<Combobox
						bind:value={ckptB}
						items={ckptsB.map((c) => ({ value: c, label: c.replace(/\.zip$/, '') }))}
						size="sm"
						searchPlaceholder="Search checkpoints…"
					/>
				</div>
				<div class="flex flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</Label>
					<Input type="number" bind:value={matchSeed} class="font-mono" />
				</div>
			{:else}
				<p class="font-mono text-[11px] text-muted-foreground">
					No saved checkpoints yet — train a policy first, then pick one for each bot.
				</p>
			{/if}
		{/if}

		<!-- Schedule: optional start time, applied to "Add to queue". -->
		<div class="flex flex-col gap-1">
			<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
				Queue start time (optional)
			</Label>
			<DateTimePicker bind:value={startEpoch} placeholder="ASAP" />
		</div>

		<div class="mt-1 flex flex-col gap-2">
			<Button disabled={!canLaunch} onclick={launch}>
				<Play class="size-4" />{mode === 'match' ? 'Start now' : 'Launch now'}
			</Button>
			<Button variant="secondary" disabled={!canQueue} onclick={addToQueue}>
				<CalendarPlus class="size-4" />Add to queue
			</Button>
			<Button variant="destructive" disabled={!canKill} onclick={() => simulation.kill()}>
				<Square class="size-4" />Kill current run
			</Button>
		</div>

		{#if simulation.status.message}
			<p class="font-mono text-[11px] text-muted-foreground">{simulation.status.message}</p>
		{/if}
	</Card.Content>
</Card.Root>
