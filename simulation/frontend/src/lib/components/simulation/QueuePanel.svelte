<script lang="ts">
	import { simulation, type QueueItem } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { X, Trash2, Clock } from '@lucide/svelte';

	const queue = $derived(simulation.queue);
	const loggedIn = $derived(simulation.username.length > 0);

	function fmtStop(item: QueueItem): string {
		const stop = item.config?.stop;
		if (item.mode === 'play') return 'match';
		if (!stop) return 'steps';
		if (stop.kind === 'steps') return `${stop.value.toLocaleString('en-US')} steps`;
		if (stop.kind === 'duration') {
			const m = Math.round(stop.value / 60);
			const h = Math.floor(m / 60);
			return h > 0 ? `${h}h ${m % 60}m` : `${m}m`;
		}
		return `until ${fmtTime(stop.value)}`;
	}

	function fmtTime(epochSec: number): string {
		const d = new Date(epochSec * 1000);
		return d.toLocaleString(undefined, {
			month: 'short',
			day: 'numeric',
			hour: '2-digit',
			minute: '2-digit'
		});
	}

	function title(item: QueueItem): string {
		if (item.mode === 'play') return 'Match';
		return (item.config?.stage as string) ?? 'train';
	}
</script>

<Card.Root>
	<Card.Header class="pb-2 pt-4">
		<div class="flex items-center justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Queue{queue.length ? ` · ${queue.length}` : ''}
			</Card.Title>
			{#if loggedIn && queue.length > 0}
				<Button
					variant="ghost"
					size="xs"
					class="font-mono text-[11px]"
					onclick={() => simulation.clearQueue()}
				>
					<Trash2 class="size-3" />Clear
				</Button>
			{/if}
		</div>
	</Card.Header>

	<Card.Content class="flex flex-col gap-1.5">
		{#if queue.length === 0}
			<p class="font-mono text-[11px] text-muted-foreground">
				No queued runs.
			</p>
		{:else}
			{#each queue as item, i (item.id)}
				<div
					class="flex items-center gap-2 rounded-md border border-border/70 bg-input/20 px-2.5 py-1.5"
				>
					<span class="font-mono text-[11px] tabular-nums text-muted-foreground">{i + 1}</span>
					<div class="min-w-0 flex-1">
						<div class="flex items-center gap-1.5">
							<span class="truncate font-mono text-xs text-foreground">{title(item)}</span>
							<span class="font-mono text-[10px] text-muted-foreground">· {fmtStop(item)}</span>
						</div>
						<div class="flex items-center gap-2 font-mono text-[10px] text-muted-foreground">
							{#if item.start_at}
								<span class="flex items-center gap-0.5">
									<Clock class="size-2.5" />{fmtTime(item.start_at)}
								</span>
							{:else}
								<span>starts when idle</span>
							{/if}
							{#if item.status === 'running'}
								<span class="text-emerald-400">launching…</span>
							{:else if item.status === 'failed'}
								<span class="text-destructive">failed{item.message ? `: ${item.message}` : ''}</span>
							{/if}
						</div>
					</div>
					{#if loggedIn}
						<Button
							variant="ghost"
							size="xs"
							class="size-6 p-0"
							title="Remove from queue"
							onclick={() => simulation.dequeue(item.id)}
						>
							<X class="size-3" />
						</Button>
					{/if}
				</div>
			{/each}
		{/if}
	</Card.Content>
</Card.Root>
