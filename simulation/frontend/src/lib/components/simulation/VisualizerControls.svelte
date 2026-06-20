<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import { RunConfig } from '$lib/state/runConfig.svelte.js';
	import { Button } from '$lib/components/ui/button';
	import * as Card from '$lib/components/ui/card';
	import RunConfigChips from './RunConfigChips.svelte';
	import { Play, Square, Loader2, CalendarPlus } from '@lucide/svelte';

	/** 'train' = single-agent training; 'match' = run two saved policies against each other. */
	let { mode = $bindable('train') }: { mode?: 'train' | 'match' } = $props();

	const config = new RunConfig({ match: true });
	// Keep the shared config's mode in sync with the bindable prop (also driven by the
	// live status in the parent), and reflect toggle clicks back out via the prop.
	$effect(() => {
		config.mode = mode;
	});

	const busy = $derived(['launching', 'running', 'stopping'].includes(simulation.status.state));
	const canLaunch = $derived(
		simulation.connected && !busy && config.configValid && !config.targetIsRemote
	);
	const canQueue = $derived(simulation.connected && config.configValid);
	const canKill = $derived(
		simulation.connected && ['launching', 'running'].includes(simulation.status.state)
	);

	function launch() {
		if (!canLaunch) return;
		if (config.mode === 'match') simulation.match(config.matchConfig());
		else simulation.launch(config.trainPayload());
	}

	function addToQueue() {
		if (!canQueue) return;
		const startAt = config.startEpoch;
		if (config.mode === 'match') simulation.enqueue(config.matchConfig(), startAt);
		else simulation.enqueue(config.trainPayload(), startAt);
		config.startEpoch = null;
	}
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

		<RunConfigChips {config} />

		<div class="mt-1 flex flex-col gap-2">
			<Button
				disabled={!canLaunch}
				title={config.targetIsRemote ? 'Remote-device targets run via the queue' : ''}
				onclick={launch}
			>
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
