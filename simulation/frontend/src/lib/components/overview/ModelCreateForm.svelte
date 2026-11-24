<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import { RunConfig, loadRewardDefaults, ALGOS, PWS_ALGOS } from '$lib/state/runConfig.svelte.js';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import { Label } from '$lib/components/ui/label';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import RunConfigChips from '$lib/components/simulation/RunConfigChips.svelte';
	import { Play, CalendarPlus, Sparkles, Layers } from '@lucide/svelte';

	let { onCreated = () => {} }: { onCreated?: () => void } = $props();

	const config = new RunConfig({ advanced: true, identity: true });

	// Seed the reward editor from the backend RewardConfig (single source of truth) so edits to
	// bucky/rewards.py show up here and the form doesn't ship stale weights that override the code.
	onMount(async () => {
		await loadRewardDefaults(config.rewardProfile);
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
	// ── "Train all PWS algorithms": one queued model per ticked algo, same settings and weights.
	// Queued (not launched directly) so they start as soon as training slots free up: in
	// parallel when the server/devices have room, one after another otherwise, and never
	// pushing an already running trainer off its slot.
	const algoLabel = (a: string) => ALGOS.find((x) => x.value === a)?.label ?? a;
	let batchBusy = $state(false);
	const canBatch = $derived(canQueue && config.batchAlgos.length > 0 && !batchBusy);
	function toggleBatchAlgo(algo: string, on: boolean) {
		const rest = config.batchAlgos.filter((a) => a !== algo);
		config.batchAlgos = on ? PWS_ALGOS.filter((a) => a === algo || rest.includes(a)) : rest;
	}
	async function queueBatch() {
		if (!canBatch) return;
		batchBusy = true;
		try {
			for (const payload of config.batchPayloads()) {
				await simulation.queueModel(payload, config.startEpoch);
			}
		} finally {
			batchBusy = false;
		}
		config.startEpoch = null;
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
			<Sparkles class="size-4" />New model
		</Card.Title>
	</Card.Header>

	<Card.Content class="flex flex-col gap-3">
		<RunConfigChips {config} />

		<div class="mt-1 grid grid-cols-2 gap-2">
			<Button
				disabled={!canLaunch}
				title={config.targetIsRemote ? 'Remote targets run via the queue' : ''}
				onclick={create}
			>
				<Play class="size-4" />Launch
			</Button>
			<Button variant="secondary" disabled={!canQueue} onclick={queue}>
				<CalendarPlus class="size-4" />Queue
			</Button>
		</div>

		<!-- PWS bake-off: one queued model per ticked algorithm, same settings and weights. -->
		<div class="mt-1 flex flex-col gap-2 border-t border-border/70 pt-3">
			<Label
				class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground"
				title="One queued model per algorithm, named <name>-<algo>"
			>
				Train all
			</Label>
			<div class="grid grid-cols-2 gap-1.5 sm:grid-cols-3">
				{#each PWS_ALGOS as algo (algo)}
					<div class="flex items-center gap-2">
						<Checkbox
							id="batch-{algo}"
							checked={config.batchAlgos.includes(algo)}
							onCheckedChange={(v) => toggleBatchAlgo(algo, v === true)}
						/>
						<Label for="batch-{algo}" class="cursor-pointer font-mono text-[11px] text-muted-foreground">
							{algoLabel(algo)}
						</Label>
					</div>
				{/each}
			</div>
			<Button variant="secondary" disabled={!canBatch} onclick={queueBatch}>
				<Layers class="size-4" />Queue {config.batchAlgos.length} models
			</Button>
		</div>

		{#if !simulation.hasCredentials}
			<p class="font-mono text-[11px] text-muted-foreground">Log in to create</p>
		{/if}
		{#if simulation.status.message}
			<p class="font-mono text-[11px] text-muted-foreground">{simulation.status.message}</p>
		{/if}
	</Card.Content>
</Card.Root>
