<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import { RunConfig, loadRewardDefaults } from '$lib/state/runConfig.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import RunConfigChips from '$lib/components/simulation/RunConfigChips.svelte';
	import { Play, CalendarPlus, Sparkles } from '@lucide/svelte';

	let { onCreated = () => {} }: { onCreated?: () => void } = $props();

	const config = new RunConfig({ advanced: true, identity: true });

	// Seed the reward editor from the backend RewardConfig (single source of truth) so edits to
	// bucky/rewards.py show up here and the form doesn't ship stale weights that override the code.
	onMount(async () => {
		await loadRewardDefaults();
		config.syncRewardDefaults();
	});

	const busy = $derived(['launching', 'running', 'stopping'].includes(simulation.status.state));
	const valid = $derived(config.configValid);
	const canLaunch = $derived(
		simulation.connected && simulation.hasCredentials && valid && !busy && !config.targetIsRemote
	);
	const canQueue = $derived(simulation.connected && simulation.hasCredentials && valid);

	function create() {
		if (!canLaunch) return;
		simulation.createModel(config.trainPayload());
		onCreated();
	}
	function queue() {
		if (!canQueue) return;
		simulation.queueModel(config.trainPayload(), config.startEpoch);
		config.startEpoch = null;
		onCreated();
	}
</script>

<Card.Root class="max-w-2xl">
	<Card.Header class="pb-3 pt-4">
		<Card.Title class="flex items-center gap-2 font-mono text-xs font-semibold uppercase tracking-widest">
			<Sparkles class="size-4" />Create a model
		</Card.Title>
	</Card.Header>

	<Card.Content class="flex flex-col gap-3">
		<RunConfigChips {config} />

		{#if !simulation.hasCredentials}
			<p class="font-mono text-[11px] text-muted-foreground">Log in to create models</p>
		{/if}

		<div class="mt-1 flex flex-col gap-2">
			<Button
				disabled={!canLaunch}
				title={config.targetIsRemote ? 'Remote-device targets run via the queue' : ''}
				onclick={create}
			>
				<Play class="size-4" />Create &amp; launch now
			</Button>
			<Button variant="secondary" disabled={!canQueue} onclick={queue}>
				<CalendarPlus class="size-4" />Add to queue
			</Button>
		</div>

		{#if simulation.status.message}
			<p class="font-mono text-[11px] text-muted-foreground">{simulation.status.message}</p>
		{/if}
	</Card.Content>
</Card.Root>
