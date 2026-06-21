<script lang="ts">
	import { simulation, type ActiveRun } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { Square, Cpu, Server } from '@lucide/svelte';

	const runs = $derived(simulation.activeRuns);
	const loggedIn = $derived(simulation.username.length > 0);

	function deviceLabel(run: ActiveRun): string {
		if (run.device === 'server') return 'server';
		const dev = simulation.devices.find((d) => d.id === run.device);
		return dev?.name ?? run.device;
	}

	function title(run: ActiveRun): string {
		if (run.run_type === 'match') return 'Match';
		return run.model_name || run.stage || run.run_name;
	}
</script>

<Card.Root>
	<Card.Header class="pb-2 pt-4">
		<div class="flex items-center justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Active runs{runs.length ? ` · ${runs.length}` : ''}
			</Card.Title>
			<span class="font-mono text-[10px] text-muted-foreground">
				server {simulation.localActive}/{simulation.localSlots}
			</span>
		</div>
	</Card.Header>

	<Card.Content class="flex flex-col gap-1.5">
		{#if runs.length === 0}
			<p class="font-mono text-[11px] text-muted-foreground">No active runs.</p>
		{:else}
			{#each runs as run (run.run_name)}
				<div
					class="flex items-center gap-2 rounded-md border border-border/70 bg-input/20 px-2.5 py-1.5"
				>
					<div class="min-w-0 flex-1">
						<div class="flex items-center gap-1.5">
							<span class="truncate font-mono text-xs text-foreground">{title(run)}</span>
							{#if run.num_timesteps}
								<span class="font-mono text-[10px] text-muted-foreground">
									· {run.num_timesteps.toLocaleString('en-US')} steps
								</span>
							{/if}
						</div>
						<div class="flex items-center gap-2 font-mono text-[10px] text-muted-foreground">
							<span class="flex items-center gap-0.5">
								{#if run.device === 'server'}
									<Server class="size-2.5" />
								{:else}
									<Cpu class="size-2.5" />
								{/if}
								{deviceLabel(run)}
							</span>
							{#if run.dist_group}
								<span class="text-sky-400" title="Federated shard of {run.dist_group}">⇆ shard</span>
							{/if}
							{#if run.cancel_requested}
								<span class="text-amber-400">stopping…</span>
							{:else if run.phase}
								<span class="text-emerald-400">{run.phase}</span>
							{/if}
						</div>
					</div>
					{#if loggedIn}
						<Button
							variant="ghost"
							size="xs"
							class="size-6 p-0"
							title="Stop this run"
							disabled={run.cancel_requested || run.state === 'stopping'}
							onclick={() => simulation.stopRun(run.run_name)}
						>
							<Square class="size-3" />
						</Button>
					{/if}
				</div>
			{/each}
		{/if}
	</Card.Content>
</Card.Root>
