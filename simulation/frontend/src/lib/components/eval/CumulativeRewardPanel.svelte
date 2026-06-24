<script lang="ts">
	import type { MetricPoint } from '$lib/state/simulation.svelte';
	import * as Card from '$lib/components/ui/card';
	import LineChart from '$lib/components/LineChart.svelte';

	let {
		cumulative,
		total,
		history,
		episode,
		step
	}: {
		cumulative: Record<string, number> | null;
		total: number | null;
		history: MetricPoint[];
		episode: number | null;
		step: number | null;
	} = $props();

	const POS = '#34d399';
	const NEG = '#f87171';

	// Show every non-trivial term, largest contributor first — the build-up makes which
	// terms dominate obvious without a fixed term list (rewards have 28 terms).
	const rows = $derived.by(() => {
		if (!cumulative) return [];
		return Object.entries(cumulative)
			.filter(([, v]) => Math.abs(v) > 1e-6)
			.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
	});
	const scale = $derived(Math.max(0.02, ...rows.map(([, v]) => Math.abs(v))));
	function pct(v: number): number {
		return Math.min((Math.abs(v) / scale) * 50, 50);
	}
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<div class="flex items-baseline justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Cumulative reward
			</Card.Title>
			<span class="font-mono text-[10px] text-muted-foreground tabular-nums">
				{#if episode !== null}ep {episode}{/if}{#if step !== null} · step {step}{/if}
			</span>
		</div>
	</Card.Header>
	<Card.Content class="flex flex-col gap-3">
		<LineChart label="Return this episode" points={history} color={(total ?? 0) >= 0 ? POS : NEG} />

		{#if rows.length > 0}
			<div class="flex flex-col gap-1.5">
				{#each rows as [key, val] (key)}
					<div>
						<div class="flex justify-between font-mono text-[11px]">
							<span class="text-muted-foreground">{key}</span>
							<span class="tabular-nums" style="color: {val >= 0 ? POS : NEG}">
								{val >= 0 ? '+' : ''}{val.toFixed(3)}
							</span>
						</div>
						<div class="relative mt-0.5 h-1.5 w-full rounded-sm bg-muted/60">
							<div class="absolute inset-y-0 left-1/2 w-px bg-border"></div>
							{#if val >= 0}
								<div class="absolute inset-y-0 left-1/2 rounded-r-sm" style="width: {pct(val)}%; background: {POS}"></div>
							{:else}
								<div class="absolute inset-y-0 rounded-l-sm" style="right: 50%; width: {pct(val)}%; background: {NEG}"></div>
							{/if}
						</div>
					</div>
				{/each}
			</div>
		{:else}
			<p class="font-mono text-xs text-muted-foreground">Waiting for the drill to start…</p>
		{/if}
	</Card.Content>
</Card.Root>
