<script lang="ts">
	import { simulation, type LaunchConfig } from '$lib/state/simulation.svelte';
	import { Button } from '$lib/components/ui/button';
	import * as Card from '$lib/components/ui/card';
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import { Play, Square, Loader2 } from '@lucide/svelte';

	const STAGES = [
		{ value: 'APPROACH_STATIC_BALL', label: 'Approach static ball', stub: false },
		{ value: 'PUSH_TO_EMPTY_GOAL', label: 'Push to empty goal', stub: false },
		{ value: 'SCRIPTED_OPPONENT', label: 'Scripted opponent (stub)', stub: true },
		{ value: 'SELF_PLAY_2V2', label: 'Self-play 2v2 (stub)', stub: true }
	];

	let cfg = $state<LaunchConfig>({
		stage: 'APPROACH_STATIC_BALL',
		timesteps: 200000,
		n_envs: 8,
		seed: 0,
		domain_rand: true
	});

	let cont = $state(false);
	let srcRun = $state('');
	let srcCkpt = $state('');

	const runs = $derived(simulation.runs);
	const hasRuns = $derived(runs.length > 0);
	const srcCheckpoints = $derived(runs.find((r) => r.run === srcRun)?.checkpoints ?? []);

	$effect(() => {
		if (!cont || !hasRuns) return;
		if (!runs.some((r) => r.run === srcRun)) srcRun = runs[runs.length - 1].run;
	});
	$effect(() => {
		if (!cont) return;
		if (!srcCheckpoints.includes(srcCkpt)) {
			srcCkpt = srcCheckpoints.find((c) => c === 'final_model.zip') ?? srcCheckpoints[0] ?? '';
		}
	});

	const busy = $derived(['launching', 'running', 'stopping'].includes(simulation.status.state));
	const canLaunch = $derived(
		simulation.connected && !busy && (!cont || (!!srcRun && !!srcCkpt))
	);
	const canKill = $derived(
		simulation.connected && ['launching', 'running'].includes(simulation.status.state)
	);

	function launch() {
		if (!canLaunch) return;
		const payload: LaunchConfig = { ...cfg };
		if (cont && srcRun && srcCkpt) payload.resume_from = { run: srcRun, checkpoint: srcCkpt };
		simulation.launch(payload);
	}
</script>

<Card.Root>
	<Card.Header class="pb-3 pt-4">
		<div class="flex items-center justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Run config
			</Card.Title>
			{#if busy}
				<span class="flex items-center gap-1 font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
					<Loader2 class="size-3 animate-spin" />locked
				</span>
			{/if}
		</div>
	</Card.Header>

	<Card.Content class="flex flex-col gap-3">
		<div class="flex flex-col gap-1">
			<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Stage</Label>
			<Combobox bind:value={cfg.stage} items={STAGES} disabled={busy} searchPlaceholder="Search stages…" />
		</div>

		<div class="grid grid-cols-3 gap-2">
			<div class="flex flex-col gap-1">
				<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Steps</Label>
				<Input
					type="number"
					min={1000}
					step={50000}
					bind:value={cfg.timesteps}
					disabled={busy}
					class="font-mono"
				/>
			</div>
			<div class="flex flex-col gap-1">
				<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Envs</Label>
				<Input
					type="number"
					min={1}
					max={32}
					bind:value={cfg.n_envs}
					disabled={busy}
					class="font-mono"
				/>
			</div>
			<div class="flex flex-col gap-1">
				<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</Label>
				<Input
					type="number"
					bind:value={cfg.seed}
					disabled={busy}
					class="font-mono"
				/>
			</div>
		</div>

		<div class="flex items-center gap-2">
			<Checkbox id="domain-rand" bind:checked={cfg.domain_rand} disabled={busy} />
			<Label for="domain-rand" class="cursor-pointer font-mono text-xs text-muted-foreground">
				Domain randomization
			</Label>
		</div>

		<div class="flex items-center gap-2 {hasRuns ? '' : 'opacity-50'}" title={hasRuns ? '' : 'No saved checkpoints yet'}>
			<Checkbox id="cont" bind:checked={cont} disabled={busy || !hasRuns} />
			<Label
				for="cont"
				class="{hasRuns ? 'cursor-pointer' : ''} font-mono text-xs text-muted-foreground"
			>
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
						disabled={busy}
						size="sm"
						searchPlaceholder="Search runs…"
					/>
				</div>
				<div class="flex flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Checkpoint</Label>
					<Combobox
						bind:value={srcCkpt}
						items={srcCheckpoints.map((c) => ({ value: c, label: c.replace(/\.zip$/, '') }))}
						disabled={busy}
						size="sm"
						searchPlaceholder="Search checkpoints…"
					/>
				</div>
				<p class="font-mono text-[10px] leading-snug text-muted-foreground">
					Seeds weights from this checkpoint into a new <span class="text-foreground/80">…_cont</span> run.
					Stage above is the target — different stages give curriculum transfer.
				</p>
			</div>
		{/if}

		<div class="mt-1 flex gap-2">
			<Button class="flex-1" disabled={!canLaunch} onclick={launch}>
				<Play class="size-4" />Launch
			</Button>
			<Button variant="destructive" class="flex-1" disabled={!canKill} onclick={() => simulation.kill()}>
				<Square class="size-4" />Kill
			</Button>
		</div>

		{#if simulation.status.message}
			<p class="font-mono text-[11px] text-muted-foreground">{simulation.status.message}</p>
		{/if}
	</Card.Content>
</Card.Root>
