<script lang="ts">
	import { simulation, type ModelInfo } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import { Input } from '$lib/components/ui/input';
	import { Button } from '$lib/components/ui/button';
	import ConfirmDialog from './ConfirmDialog.svelte';
	import { Download, Trash2, ChevronRight, Cpu, Eraser } from '@lucide/svelte';

	let search = $state('');
	let sort = $state<'newest' | 'name'>('newest');
	// run names the user has expanded to reveal individual checkpoints
	let expanded = $state<Record<string, boolean>>({});

	const canControl = $derived(simulation.hasCredentials);

	// Filter, then group runs by model name (each run = one version of a model).
	const groups = $derived.by(() => {
		const q = search.trim().toLowerCase();
		const filtered = simulation.models.filter(
			(m) =>
				!q ||
				m.name.toLowerCase().includes(q) ||
				m.run.toLowerCase().includes(q) ||
				(m.stage ?? '').toLowerCase().includes(q)
		);
		const byName = new Map<string, ModelInfo[]>();
		for (const m of filtered) {
			const arr = byName.get(m.name) ?? [];
			arr.push(m);
			byName.set(m.name, arr);
		}
		const list = [...byName.entries()].map(([name, versions]) => ({
			name,
			versions,
			newest: Math.max(...versions.map((v) => v.created_at ?? 0))
		}));
		list.sort((a, b) =>
			sort === 'name' ? a.name.localeCompare(b.name) : b.newest - a.newest
		);
		return list;
	});

	function fmtBytes(n: number): string {
		if (n < 1024) return `${n} B`;
		if (n < 1024 * 1024) return `${(n / 1024).toFixed(0)} KB`;
		return `${(n / 1024 / 1024).toFixed(1)} MB`;
	}
	function fmtDate(epoch: number | null): string {
		if (!epoch) return '—';
		return new Date(epoch * 1000).toLocaleString();
	}
	function statusColor(s: string | null): string {
		if (s === 'training') return '#fbbf24';
		if (s === 'done') return '#34d399';
		if (s === 'interrupted') return '#f87171';
		return '#94a3b8';
	}

	// Confirmation dialog state — a pending action is shown in ConfirmDialog and run
	// only if the user confirms.
	let confirmOpen = $state(false);
	let pending = $state<{
		title: string;
		description: string;
		confirmLabel: string;
		action: () => void;
	} | null>(null);

	/** Number of periodic *_steps.zip snapshots across a model's versions. */
	function stepCount(versions: ModelInfo[]): number {
		return versions.reduce(
			(n, v) => n + v.checkpoints.filter((c) => c.file.endsWith('_steps.zip')).length,
			0
		);
	}

	function confirmDeleteModel(name: string, versions: ModelInfo[]) {
		pending = {
			title: 'Delete entire model',
			description: `Delete all ${versions.length} version${versions.length === 1 ? '' : 's'} of "${name}" and every checkpoint? This cannot be undone.`,
			confirmLabel: 'Delete model',
			action: () => versions.forEach((v) => simulation.deleteModel(v.run))
		};
		confirmOpen = true;
	}

	function confirmPruneSteps(name: string, versions: ModelInfo[]) {
		const n = stepCount(versions);
		pending = {
			title: 'Remove step checkpoints',
			description: `Remove ${n} periodic step snapshot${n === 1 ? '' : 's'} from "${name}"? The best and final checkpoints are kept.`,
			confirmLabel: 'Remove',
			action: () => versions.forEach((v) => simulation.pruneStepCheckpoints(v.run))
		};
		confirmOpen = true;
	}

	function confirmDeleteRun(run: string) {
		pending = {
			title: 'Delete model version',
			description: `Delete "${run}" and all its checkpoints? This cannot be undone.`,
			confirmLabel: 'Delete',
			action: () => simulation.deleteModel(run)
		};
		confirmOpen = true;
	}
	function confirmDeleteCkpt(run: string, file: string) {
		pending = {
			title: 'Delete checkpoint',
			description: `Delete checkpoint "${file}" from "${run}"?`,
			confirmLabel: 'Delete',
			action: () => simulation.deleteCheckpoint(run, file)
		};
		confirmOpen = true;
	}
</script>

<div class="flex flex-col gap-4">
	<div class="flex flex-wrap items-center gap-2">
		<Input
			bind:value={search}
			placeholder="Search models, runs, stages…"
			class="h-8 max-w-xs font-mono text-xs"
		/>
		<div class="ml-auto flex items-center gap-1 rounded-md border border-border/70 bg-input/20 p-1">
			<Button
				variant={sort === 'newest' ? 'default' : 'ghost'}
				size="sm"
				class="h-6 font-mono text-[10px] uppercase"
				onclick={() => (sort = 'newest')}>Newest</Button
			>
			<Button
				variant={sort === 'name' ? 'default' : 'ghost'}
				size="sm"
				class="h-6 font-mono text-[10px] uppercase"
				onclick={() => (sort = 'name')}>Name</Button
			>
		</div>
	</div>

	{#if !canControl}
		<p class="font-mono text-[11px] text-muted-foreground">
			Log in to download or delete models
		</p>
	{/if}

	{#if groups.length === 0}
		<Card.Root>
			<Card.Content class="py-10 text-center font-mono text-xs text-muted-foreground">
				{simulation.models.length === 0
					? 'No models yet. Create one in the Create tab.'
					: 'No models match your search.'}
			</Card.Content>
		</Card.Root>
	{/if}

	{#each groups as group (group.name)}
		<Card.Root>
			<Card.Header class="pb-2 pt-3">
				<div class="flex items-center gap-2">
					<Cpu class="size-4 text-muted-foreground" />
					<Card.Title class="font-mono text-sm font-semibold">{group.name}</Card.Title>
					<span class="font-mono text-[10px] text-muted-foreground">
						{group.versions.length} version{group.versions.length === 1 ? '' : 's'}
					</span>
					<div class="ml-auto flex items-center gap-1">
						{#if stepCount(group.versions) > 0}
							<Button
								variant="ghost"
								size="xs"
								class="font-mono text-[10px]"
								disabled={!canControl}
								title="Delete the periodic *_steps.zip snapshots (keep best + final)"
								onclick={() => confirmPruneSteps(group.name, group.versions)}
							>
								<Eraser class="size-3" />Remove step ckpts ({stepCount(group.versions)})
							</Button>
						{/if}
						<Button
							variant="ghost"
							size="xs"
							class="font-mono text-[10px] text-destructive hover:text-destructive"
							disabled={!canControl}
							title="Delete every version of this model"
							onclick={() => confirmDeleteModel(group.name, group.versions)}
						>
							<Trash2 class="size-3" />Delete model
						</Button>
					</div>
				</div>
			</Card.Header>
			<Card.Content class="flex flex-col gap-2">
				{#each group.versions as m (m.run)}
					<div class="rounded-md border border-border/70 bg-input/10">
						<button
							class="flex w-full items-center gap-2 px-3 py-2 text-left"
							onclick={() => (expanded[m.run] = !expanded[m.run])}
						>
							<ChevronRight
								class="size-3.5 shrink-0 text-muted-foreground transition-transform {expanded[m.run]
									? 'rotate-90'
									: ''}"
							/>
							<span class="font-mono text-xs font-semibold">
								{m.version ? `v${m.version}` : m.run}
							</span>
							<span
								class="font-mono text-[10px] uppercase"
								style="color:{statusColor(m.status)}"
							>
								● {m.status ?? 'unknown'}
							</span>
							{#if m.stage}
								<span class="font-mono text-[10px] text-muted-foreground">{m.stage}</span>
							{/if}
							<span class="ml-auto flex items-center gap-3 font-mono text-[10px] text-muted-foreground">
								{#if m.best_eval != null}<span>eval {m.best_eval.toFixed(2)}</span>{/if}
								{#if m.timesteps_trained != null}<span>{m.timesteps_trained.toLocaleString()} steps</span>{/if}
								<span>{m.checkpoints.length} ckpt</span>
							</span>
						</button>

						{#if expanded[m.run]}
							<div class="border-t border-border/50 px-3 py-2">
								<div class="mb-2 flex flex-wrap items-center gap-x-4 gap-y-1 font-mono text-[10px] text-muted-foreground">
									<span>run: {m.run}</span>
									<span>created: {fmtDate(m.created_at)}</span>
									{#if m.created_by}<span>by: {m.created_by}</span>{/if}
									{#if m.parent}<span>from: {m.parent}</span>{/if}
									{#if !m.has_meta}<span class="text-amber-500">legacy (no metadata)</span>{/if}
								</div>

								{#if m.config}
									<details class="mb-2">
										<summary class="cursor-pointer font-mono text-[10px] uppercase text-muted-foreground">
											config
										</summary>
										<pre class="mt-1 overflow-x-auto rounded bg-background/60 p-2 font-mono text-[10px] leading-tight">{JSON.stringify(
												m.config,
												null,
												2
											)}</pre>
									</details>
								{/if}

								<div class="flex flex-col gap-1">
									{#each m.checkpoints as ck (ck.file)}
										<div class="flex items-center gap-2 rounded bg-background/40 px-2 py-1">
											<span class="font-mono text-[11px]">{ck.file}</span>
											<span class="font-mono text-[10px] text-muted-foreground">{fmtBytes(ck.size)}</span>
											<div class="ml-auto flex items-center gap-1">
												<Button
													variant="ghost"
													size="xs"
													class="font-mono text-[10px]"
													disabled={!canControl}
													onclick={() => simulation.downloadModel(m.run, ck.file)}
												>
													<Download class="size-3" />Download
												</Button>
												<Button
													variant="ghost"
													size="xs"
													class="font-mono text-[10px] text-destructive hover:text-destructive"
													disabled={!canControl}
													onclick={() => confirmDeleteCkpt(m.run, ck.file)}
												>
													<Trash2 class="size-3" />
												</Button>
											</div>
										</div>
									{/each}
								</div>

								<div class="mt-2 flex justify-end">
									<Button
										variant="outline"
										size="xs"
										class="font-mono text-[10px] text-destructive hover:text-destructive"
										disabled={!canControl}
										onclick={() => confirmDeleteRun(m.run)}
									>
										<Trash2 class="size-3" />Delete version
									</Button>
								</div>
							</div>
						{/if}
					</div>
				{/each}
			</Card.Content>
		</Card.Root>
	{/each}
</div>

<ConfirmDialog
	bind:open={confirmOpen}
	title={pending?.title ?? ''}
	description={pending?.description ?? ''}
	confirmLabel={pending?.confirmLabel ?? 'Confirm'}
	destructive
	onconfirm={() => pending?.action()}
/>
