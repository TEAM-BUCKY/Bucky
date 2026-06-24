<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import {
		RunConfig,
		STAGES,
		STOP_TABS,
		HP_FIELDS,
		rewardState
	} from '$lib/state/runConfig.svelte.js';
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Button } from '$lib/components/ui/button';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import ConfigChip from '$lib/components/ui/config-chip/ConfigChip.svelte';
	import DateTimePicker from './DateTimePicker.svelte';
	import SplitBar from './SplitBar.svelte';

	let { config }: { config: RunConfig } = $props();

	const runs = $derived(simulation.runs);
	const hasRuns = $derived(runs.length > 0);
	const devices = $derived(simulation.devices);
	const runItems = $derived(runs.map((r) => ({ value: r.run, label: r.run })));
	const ckptItems = (cks: string[]) =>
		cks.map((c) => ({ value: c, label: c.replace(/\.zip$/, '') }));

	const srcCheckpoints = $derived(runs.find((r) => r.run === config.srcRun)?.checkpoints ?? []);
	const ckptsA = $derived(runs.find((r) => r.run === config.runA)?.checkpoints ?? []);
	const ckptsB = $derived(runs.find((r) => r.run === config.runB)?.checkpoints ?? []);

	const pickFinal = (cks: string[]) => cks.find((c) => c === 'final_model.zip') ?? cks[0] ?? '';

	// Auto-pick sensible defaults as the available runs/checkpoints change.
	$effect(() => {
		if (!config.cont || !hasRuns) return;
		if (!runs.some((r) => r.run === config.srcRun)) config.srcRun = runs[runs.length - 1].run;
	});
	$effect(() => {
		if (!config.cont) return;
		if (!srcCheckpoints.includes(config.srcCkpt)) config.srcCkpt = pickFinal(srcCheckpoints);
	});
	$effect(() => {
		if (config.mode !== 'match' || !hasRuns) return;
		if (!runs.some((r) => r.run === config.runA)) config.runA = runs[0].run;
		if (!runs.some((r) => r.run === config.runB)) config.runB = runs[runs.length - 1].run;
	});
	$effect(() => {
		if (config.mode !== 'match') return;
		if (!ckptsA.includes(config.ckptA)) config.ckptA = pickFinal(ckptsA);
	});
	$effect(() => {
		if (config.mode !== 'match') return;
		if (!ckptsB.includes(config.ckptB)) config.ckptB = pickFinal(ckptsB);
	});

	const TARGET_TABS = [
		{ value: 'any', label: 'Any' },
		{ value: 'server', label: 'Server' },
		{ value: 'device', label: 'Device' }
	];
	// 'device' tab is selected whenever target is a concrete device id.
	const targetTab = $derived(
		config.target === 'any' || config.target === 'server' ? config.target : 'device'
	);
	function pickTargetTab(tab: string) {
		if (tab === 'any' || tab === 'server') config.target = tab;
		else config.target = devices[0]?.id ?? '';
	}
	const targetSummary = $derived(
		config.target === 'any'
			? 'Any available'
			: config.target === 'server'
				? 'Server only'
				: (devices.find((d) => d.id === config.target)?.name ?? 'pick a device')
	);

	// ── per-device FedAvg distribution ──────────────────────────────────────────
	// Assignable shard hosts: the local server plus every registered device.
	const distHosts = $derived([
		{ id: 'server', name: 'Server (local)', online: true },
		...devices.map((d) => ({ id: d.id, name: d.online ? d.name : `${d.name} (offline)`, online: d.online }))
	]);
	// Mode toggle: 'even' = N identical shards, 'device' = per-host env counts.
	let distMode = $state<'even' | 'device'>('even');
	$effect(() => {
		// Per-device assignments only take effect in 'device' mode; clear them otherwise
		// so trainPayload() falls back to even shards.
		if (distMode === 'even' && config.distDevices.length > 0) config.distDevices = [];
	});
	const distEnvsOf = (target: string) =>
		config.distDevices.find((d) => d.target === target)?.n_envs ?? 0;
	function toggleDistHost(target: string, on: boolean) {
		const rest = config.distDevices.filter((d) => d.target !== target);
		config.distDevices = on ? [...rest, { target, n_envs: config.n_envs }] : rest;
	}
	function setDistEnvs(target: string, n: number) {
		const envs = Math.max(1, Math.floor(n) || 1);
		config.distDevices = config.distDevices.map((d) =>
			d.target === target ? { ...d, n_envs: envs } : d
		);
	}
</script>

{#if config.caps.identity}
	<!-- Identity stays inline — it's the model's primary label, not a tucked-away option. -->
	<div class="grid grid-cols-3 gap-2">
		<div class="col-span-2 flex flex-col gap-1">
			<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Name</Label>
			<Input bind:value={config.name} placeholder="e.g. striker" class="font-mono" />
		</div>
		<div class="flex flex-col gap-1">
			<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Version</Label>
			<Input bind:value={config.version} placeholder="1" class="font-mono" />
		</div>
	</div>
{/if}

{#if config.mode === 'train'}
	<div class="grid grid-cols-2 gap-2">
		<!-- Stage -->
		<ConfigChip label="Stage" summary={config.stageSummary}>
			{#snippet body()}
				<div class="flex flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Stage</Label>
					<Combobox bind:value={config.stage} items={STAGES} searchPlaceholder="Search stages…" />
				</div>
				{#if config.stage === 'FULL_TRAINING'}
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
							Phase budget split
						</Label>
						<SplitBar bind:values={config.fullSplit} labels={['Approach', 'Push', 'Kick', 'Self-play']} />
						<p class="font-mono text-[10px] text-muted-foreground">
							Drag the dividers to set each phase's share of the total budget.
						</p>
					</div>
				{/if}
			{/snippet}
		</ConfigChip>

		<!-- Stop condition -->
		<ConfigChip label="Stop" summary={config.stopSummary} invalid={!config.stopValid} title="Stop after">
			{#snippet body()}
				<div class="grid grid-cols-3 gap-1 rounded-md border border-border/70 bg-input/20 p-1">
					{#each STOP_TABS as t (t.value)}
						<Button
							variant={config.stopKind === t.value ? 'default' : 'ghost'}
							size="sm"
							class="h-7 font-mono text-[11px] uppercase tracking-wider"
							onclick={() => (config.stopKind = t.value)}>{t.label}</Button
						>
					{/each}
				</div>
				{#if config.stopKind === 'steps'}
					<Input type="number" min={1000} step={50000} bind:value={config.timesteps} class="font-mono" />
				{:else if config.stopKind === 'duration'}
					<div class="grid grid-cols-2 gap-2">
						<div class="flex items-center gap-1">
							<Input type="number" min={0} bind:value={config.durH} class="font-mono" />
							<span class="font-mono text-[11px] text-muted-foreground">h</span>
						</div>
						<div class="flex items-center gap-1">
							<Input type="number" min={0} max={59} bind:value={config.durM} class="font-mono" />
							<span class="font-mono text-[11px] text-muted-foreground">m</span>
						</div>
					</div>
				{:else}
					<DateTimePicker bind:value={config.untilEpoch} />
					{#if config.untilEpoch != null && !config.stopValid}
						<span class="font-mono text-[11px] text-destructive">Pick a time in the future.</span>
					{/if}
				{/if}
			{/snippet}
		</ConfigChip>

		<!-- Parallelism -->
		<ConfigChip label="Parallelism" summary={config.parallelismSummary}>
			{#snippet body()}
				<div class="grid grid-cols-2 gap-2">
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Envs</Label>
						<Input type="number" min={1} max={32} bind:value={config.n_envs} class="font-mono" />
					</div>
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</Label>
						<Input type="number" bind:value={config.seed} class="font-mono" />
					</div>
				</div>
			{/snippet}
		</ConfigChip>

		<!-- Options -->
		<ConfigChip label="Options" summary={config.optionsSummary}>
			{#snippet body()}
				<div class="flex items-center gap-2">
					<Checkbox id="opt-dr" bind:checked={config.domain_rand} />
					<Label for="opt-dr" class="cursor-pointer font-mono text-xs text-muted-foreground">
						Domain randomization
					</Label>
				</div>
				<div class="flex items-center gap-2">
					<Checkbox id="opt-viz" bind:checked={config.viz} />
					<Label for="opt-viz" class="cursor-pointer font-mono text-xs text-muted-foreground">
						Watch live (slower)
					</Label>
				</div>
				<div class="flex items-center gap-2">
					<Checkbox id="opt-dist" bind:checked={config.distributed} />
					<Label for="opt-dist" class="cursor-pointer font-mono text-xs text-muted-foreground">
						Distribute across devices (FedAvg)
					</Label>
				</div>
				{#if config.distributed}
					<div class="ml-6 flex flex-col gap-2">
						<!-- Even N shards vs. per-device env counts. -->
						<div class="grid grid-cols-2 gap-1 rounded-md border border-border/70 bg-input/20 p-1">
							<Button
								variant={distMode === 'even' ? 'default' : 'ghost'}
								size="sm"
								class="h-7 font-mono text-[11px] uppercase tracking-wider"
								onclick={() => (distMode = 'even')}>Even shards</Button
							>
							<Button
								variant={distMode === 'device' ? 'default' : 'ghost'}
								size="sm"
								class="h-7 font-mono text-[11px] uppercase tracking-wider"
								onclick={() => (distMode = 'device')}>Per-device</Button
							>
						</div>

						{#if distMode === 'even'}
							<div class="flex items-center gap-1.5">
								<Label for="dist-shards" class="font-mono text-[10px] text-muted-foreground">shards</Label>
								<Input
									id="dist-shards"
									type="number"
									min="2"
									class="h-7 w-16 font-mono text-xs"
									bind:value={config.distShards}
								/>
							</div>
						{:else}
							<!-- One shard per enabled host, sized to its env count. -->
							<div class="flex flex-col gap-1">
								{#each distHosts as host (host.id)}
									{@const enabled = config.distDevices.some((d) => d.target === host.id)}
									<div class="flex items-center gap-2">
										<Checkbox
											id="dist-{host.id}"
											checked={enabled}
											onCheckedChange={(v) => toggleDistHost(host.id, !!v)}
										/>
										<Label
											for="dist-{host.id}"
											class="flex-1 cursor-pointer font-mono text-xs text-muted-foreground"
										>
											{host.name}
										</Label>
										<Input
											type="number"
											min="1"
											max="32"
											disabled={!enabled}
											value={enabled ? distEnvsOf(host.id) : ''}
											oninput={(e) => setDistEnvs(host.id, +e.currentTarget.value)}
											placeholder="envs"
											class="h-7 w-16 font-mono text-xs"
										/>
									</div>
								{/each}
								{#if config.distDevices.filter((d) => d.n_envs >= 1).length < 2}
									<span class="font-mono text-[11px] text-muted-foreground">
										Enable at least 2 hosts to distribute.
									</span>
								{/if}
							</div>
						{/if}

						<div class="flex items-center gap-1.5">
							<Label for="dist-every" class="font-mono text-[10px] text-muted-foreground">sync every</Label>
							<Input
								id="dist-every"
								type="number"
								min="1000"
								step="1000"
								class="h-7 w-24 font-mono text-xs"
								bind:value={config.distSyncEvery}
							/>
						</div>
					</div>
				{/if}
				{#if config.caps.advanced}
					<div class="flex items-center gap-2">
						<Checkbox id="opt-steps" bind:checked={config.saveStepCheckpoints} />
						<Label for="opt-steps" class="cursor-pointer font-mono text-xs text-muted-foreground">
							Save periodic step checkpoints
						</Label>
					</div>
				{/if}
			{/snippet}
		</ConfigChip>

		<!-- Resume -->
		<ConfigChip label="Resume" summary={config.resumeSummary}>
			{#snippet body()}
				<div class="flex items-center gap-2 {hasRuns ? '' : 'opacity-50'}">
					<Checkbox id="resume-cont" bind:checked={config.cont} disabled={!hasRuns} />
					<Label for="resume-cont" class="{hasRuns ? 'cursor-pointer' : ''} font-mono text-xs text-muted-foreground">
						Continue from checkpoint
					</Label>
				</div>
				{#if !hasRuns}
					<span class="font-mono text-[11px] text-muted-foreground">No saved checkpoints yet.</span>
				{/if}
				{#if config.cont && hasRuns}
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Source run</Label>
						<Combobox bind:value={config.srcRun} items={runItems} size="sm" searchPlaceholder="Search runs…" />
					</div>
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Checkpoint</Label>
						<Combobox
							bind:value={config.srcCkpt}
							items={ckptItems(srcCheckpoints)}
							size="sm"
							searchPlaceholder="Search checkpoints…"
						/>
					</div>
				{/if}
			{/snippet}
		</ConfigChip>

		<!-- Target device -->
		<ConfigChip label="Target" summary={targetSummary} invalid={targetTab === 'device' && !config.target}>
			{#snippet body()}
				<div class="grid grid-cols-3 gap-1 rounded-md border border-border/70 bg-input/20 p-1">
					{#each TARGET_TABS as t (t.value)}
						<Button
							variant={targetTab === t.value ? 'default' : 'ghost'}
							size="sm"
							class="h-7 font-mono text-[11px] uppercase tracking-wider"
							onclick={() => pickTargetTab(t.value)}>{t.label}</Button
						>
					{/each}
				</div>
				{#if targetTab === 'device'}
					{#if devices.length > 0}
						<Combobox
							bind:value={config.target}
							items={devices.map((d) => ({ value: d.id, label: d.online ? d.name : `${d.name} (offline)` }))}
							size="sm"
							searchPlaceholder="Search devices…"
						/>
					{:else}
						<span class="font-mono text-[11px] text-muted-foreground">No devices registered yet.</span>
					{/if}
				{/if}
				<p class="font-mono text-[10px] text-muted-foreground">
					Remote-device targets run via the queue, not "Launch now".
				</p>
			{/snippet}
		</ConfigChip>

		{#if config.caps.advanced}
			<!-- Advanced: net arch + PPO hyperparameters -->
			<ConfigChip label="Advanced" summary={config.advancedSummary} invalid={!config.archValid} title="Advanced">
				{#snippet body()}
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
							Network architecture (comma-separated hidden sizes)
						</Label>
						<Input bind:value={config.netArchStr} placeholder="64,64" class="font-mono" />
						{#if !config.archValid}
							<span class="font-mono text-[11px] text-destructive">Enter at least one positive integer.</span>
						{/if}
					</div>
					<div class="rounded-md border border-border/70 bg-input/10 p-2">
						<Label class="mb-1.5 block font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
							PPO hyperparameters
						</Label>
						<div class="grid grid-cols-2 gap-2 sm:grid-cols-3">
							{#each HP_FIELDS as f (f.key)}
								<div class="flex flex-col gap-0.5">
									<Label class="font-mono text-[10px] text-muted-foreground">{f.label}</Label>
									<Input type="number" step={f.step} bind:value={config.hp[f.key]} class="h-8 font-mono text-xs" />
								</div>
							{/each}
						</div>
					</div>
				{/snippet}
			</ConfigChip>

			<!-- Reward weights -->
			<ConfigChip label="Rewards" summary={config.rewardsSummary} title="Reward weights">
				{#snippet body()}
					<div class="grid grid-cols-2 gap-2 sm:grid-cols-3">
						{#each rewardState.fields as f (f.key)}
							<div class="flex flex-col gap-0.5">
								<Label class="font-mono text-[10px] text-muted-foreground">{f.label}</Label>
								<Input type="number" step={0.1} bind:value={config.rw[f.key]} class="h-8 font-mono text-xs" />
							</div>
						{/each}
					</div>
				{/snippet}
			</ConfigChip>
		{/if}

		<!-- Queue start time -->
		<ConfigChip label="Schedule" summary={config.scheduleSummary} title="Queue start time">
			{#snippet body()}
				<p class="font-mono text-[10px] text-muted-foreground">Applies to "Add to queue" only.</p>
				<DateTimePicker bind:value={config.startEpoch} placeholder="ASAP" />
			{/snippet}
		</ConfigChip>
	</div>
{:else}
	<!-- Match mode -->
	{#if hasRuns}
		<div class="grid grid-cols-2 gap-2">
			<ConfigChip label="Bot A" summary={config.botASummary} invalid={!config.runA || !config.ckptA}>
				{#snippet body()}
					<Label class="font-mono text-[10px] font-semibold uppercase tracking-wider" style="color:#3c78dc">
						Bot A network
					</Label>
					<Combobox bind:value={config.runA} items={runItems} size="sm" searchPlaceholder="Search runs…" />
					<Combobox bind:value={config.ckptA} items={ckptItems(ckptsA)} size="sm" searchPlaceholder="Search checkpoints…" />
				{/snippet}
			</ConfigChip>

			<ConfigChip label="Bot B" summary={config.botBSummary} invalid={!config.runB || !config.ckptB}>
				{#snippet body()}
					<Label class="font-mono text-[10px] font-semibold uppercase tracking-wider" style="color:#dc3c3c">
						Bot B network
					</Label>
					<Combobox bind:value={config.runB} items={runItems} size="sm" searchPlaceholder="Search runs…" />
					<Combobox bind:value={config.ckptB} items={ckptItems(ckptsB)} size="sm" searchPlaceholder="Search checkpoints…" />
				{/snippet}
			</ConfigChip>

			<ConfigChip label="Seed" summary={String(config.matchSeed)}>
				{#snippet body()}
					<div class="flex flex-col gap-1">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Seed</Label>
						<Input type="number" bind:value={config.matchSeed} class="font-mono" />
					</div>
				{/snippet}
			</ConfigChip>

			<ConfigChip label="Schedule" summary={config.scheduleSummary} title="Queue start time">
				{#snippet body()}
					<p class="font-mono text-[10px] text-muted-foreground">Applies to "Add to queue" only.</p>
					<DateTimePicker bind:value={config.startEpoch} placeholder="ASAP" />
				{/snippet}
			</ConfigChip>

			<!-- Test mode: drive Bot A (red) yourself against Bot B's AI (mouse aim + WASD + kick). -->
			<div class="col-span-2 flex items-center gap-2 rounded-md border border-border/70 bg-input/20 px-2 py-1.5">
				<Checkbox id="opt-manual-red" bind:checked={config.manualRed} />
				<Label for="opt-manual-red" class="cursor-pointer font-mono text-xs text-muted-foreground">
					Drive yourself
				</Label>
			</div>
		</div>
	{:else}
		<p class="font-mono text-[11px] text-muted-foreground">
			No saved checkpoints yet — train a policy first, then pick one for each bot.
		</p>
	{/if}
{/if}
