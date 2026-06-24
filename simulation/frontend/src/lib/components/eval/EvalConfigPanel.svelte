<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { Play, Square } from '@lucide/svelte';

	// Drills = curriculum stages, each a concrete eval scenario.
	const DRILLS = [
		{ value: 'APPROACH_STATIC_BALL', label: 'Approach static ball' },
		{ value: 'PUSH_TO_EMPTY_GOAL', label: 'Push to empty goal' },
		{ value: 'AIM_AND_KICK', label: 'Aim & kick (shooting)' },
		{ value: 'SELF_PLAY_1V1', label: 'Self-play 1v1' }
	];

	let run = $state('');
	let checkpoint = $state('');
	let stage = $state(DRILLS[0].value);
	let nEpisodes = $state(10);
	let seed = $state(999);
	let deterministic = $state(true);

	// Keep the checkpoint valid when the run changes; default to a sensible file.
	const checkpoints = $derived(simulation.runs.find((r) => r.run === run)?.checkpoints ?? []);
	$effect(() => {
		if (run && !checkpoints.includes(checkpoint)) {
			checkpoint =
				checkpoints.find((c) => c === 'final_model.zip') ??
				checkpoints.find((c) => c === 'best_model.zip') ??
				checkpoints[0] ??
				'';
		}
	});

	const running = $derived(simulation.evalStatus.running);
	const canStart = $derived(!!run && !!checkpoint && !running && simulation.hasCredentials);

	async function start() {
		await simulation.startEval({ run, checkpoint, stage, nEpisodes, seed, deterministic });
	}
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
			Evaluation drill
		</Card.Title>
	</Card.Header>
	<Card.Content class="flex flex-col gap-3">
		<label class="flex flex-col gap-1">
			<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Model</span>
			<select
				bind:value={run}
				disabled={running}
				class="h-8 rounded-md border border-border bg-background/60 px-2 font-mono text-xs outline-none"
			>
				<option value="" disabled>Select a run…</option>
				{#each simulation.runs as r (r.run)}
					<option value={r.run}>{r.run}</option>
				{/each}
			</select>
		</label>

		<label class="flex flex-col gap-1">
			<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Checkpoint</span>
			<select
				bind:value={checkpoint}
				disabled={running || !run}
				class="h-8 rounded-md border border-border bg-background/60 px-2 font-mono text-xs outline-none"
			>
				{#each checkpoints as c (c)}
					<option value={c}>{c}</option>
				{/each}
			</select>
		</label>

		<label class="flex flex-col gap-1">
			<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Drill</span>
			<select
				bind:value={stage}
				disabled={running}
				class="h-8 rounded-md border border-border bg-background/60 px-2 font-mono text-xs outline-none"
			>
				{#each DRILLS as d (d.value)}
					<option value={d.value}>{d.label}</option>
				{/each}
			</select>
		</label>

		<div class="flex gap-3">
			<label class="flex flex-1 flex-col gap-1">
				<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Episodes</span>
				<input
					type="number"
					min="1"
					max="100"
					bind:value={nEpisodes}
					disabled={running}
					class="h-8 rounded-md border border-border bg-background/60 px-2 font-mono text-xs tabular-nums outline-none"
				/>
			</label>
			<label class="flex flex-1 flex-col gap-1">
				<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</span>
				<input
					type="number"
					bind:value={seed}
					disabled={running}
					class="h-8 rounded-md border border-border bg-background/60 px-2 font-mono text-xs tabular-nums outline-none"
				/>
			</label>
		</div>

		<label class="flex items-center gap-2">
			<input type="checkbox" bind:checked={deterministic} disabled={running} />
			<span class="font-mono text-[11px] text-muted-foreground">Deterministic policy</span>
		</label>

		{#if running}
			<Button variant="destructive" size="sm" class="font-mono text-[11px]" onclick={() => simulation.stopEval()}>
				<Square class="size-3" />Stop evaluation
			</Button>
		{:else}
			<Button size="sm" class="font-mono text-[11px]" disabled={!canStart} onclick={start}>
				<Play class="size-3" />Run drill
			</Button>
		{/if}

		{#if !simulation.hasCredentials}
			<p class="font-mono text-[11px] text-amber-500">Log in to run evaluations.</p>
		{:else if simulation.evalError}
			<p class="font-mono text-[11px] text-red-400">{simulation.evalError}</p>
		{/if}
	</Card.Content>
</Card.Root>
