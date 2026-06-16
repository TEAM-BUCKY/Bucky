<script lang="ts">
	import type { SimFrame } from '$lib/state/simulation.svelte';

	let {
		frame,
		episodeReturns,
		connected
	}: { frame: SimFrame | null; episodeReturns: number[]; connected: boolean } = $props();

	const W = 200;
	const H = 40;

	const maxReturn = $derived(Math.max(...episodeReturns, 1));
	const minReturn = $derived(Math.min(...episodeReturns, 0));
	const range = $derived(maxReturn - minReturn || 1);

	function sparkY(val: number): number {
		return H - ((val - minReturn) / range) * H;
	}

	const points = $derived(
		episodeReturns
			.map((v, i) => `${(i / Math.max(episodeReturns.length - 1, 1)) * W},${sparkY(v)}`)
			.join(' ')
	);
</script>

<div class="rounded-lg border border-border bg-card p-3 text-xs font-mono space-y-1">
	<div class="flex items-center gap-2 text-sm font-semibold text-foreground">
		Training Metrics
		<span class="h-2 w-2 rounded-full {connected ? 'bg-green-400' : 'bg-red-400'}"></span>
	</div>
	{#if frame}
		<div class="flex gap-4 text-muted-foreground">
			<span>Ep <span class="text-foreground font-bold">{frame.episode}</span></span>
			<span>Step <span class="text-foreground">{frame.step}</span></span>
			<span>
				Return
				<span class={frame.total_return >= 0 ? 'text-green-400' : 'text-red-400'}>
					{frame.total_return.toFixed(2)}
				</span>
			</span>
		</div>
		{#if episodeReturns.length > 1}
			<svg width={W} height={H} class="mt-1 w-full">
				<polyline
					{points}
					fill="none"
					stroke="currentColor"
					stroke-width="1.5"
					class="text-primary"
				/>
			</svg>
		{/if}
	{:else}
		<p class="text-muted-foreground">Not connected</p>
	{/if}
</div>
