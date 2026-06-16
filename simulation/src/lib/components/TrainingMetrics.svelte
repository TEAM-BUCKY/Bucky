<script lang="ts">
	import type { SimFrame } from '$lib/state/simulation.svelte';
	import * as Card from '$lib/components/ui/card';

	let {
		frame,
		episodeReturns
	}: { frame: SimFrame | null; episodeReturns: number[] } = $props();

	const VW = 240;
	const VH = 48;
	const PAD = 2;

	const stats = $derived.by(() => {
		if (episodeReturns.length === 0) return null;
		const lo = Math.min(...episodeReturns);
		const hi = Math.max(...episodeReturns);
		return { lo, hi, span: hi - lo || Math.abs(hi) || 1 };
	});

	function px(i: number, n: number) {
		return PAD + (n === 1 ? 0 : (i / (n - 1)) * (VW - 2 * PAD));
	}
	function py(v: number) {
		if (!stats) return VH / 2;
		return PAD + (1 - (v - stats.lo) / stats.span) * (VH - 2 * PAD);
	}

	const line = $derived(
		episodeReturns.map((v, i) => `${i === 0 ? 'M' : 'L'}${px(i, episodeReturns.length).toFixed(1)},${py(v).toFixed(1)}`).join(' ')
	);
	const area = $derived(
		episodeReturns.length > 1
			? `${line} L${px(episodeReturns.length - 1, episodeReturns.length).toFixed(1)},${VH} L${PAD},${VH} Z`
			: ''
	);
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<div class="flex items-baseline justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Episode return
			</Card.Title>
			<span class="font-mono text-[10px] text-muted-foreground">last {episodeReturns.length}</span>
		</div>
	</Card.Header>
	<Card.Content>
		{#if frame}
			<div class="flex items-end justify-between font-mono">
				<div class="flex gap-4 text-[11px] text-muted-foreground">
					<span>ep <span class="text-foreground">{frame.episode}</span></span>
					<span>step <span class="text-foreground">{frame.step}</span></span>
				</div>
				<span
					class="text-lg leading-none tabular-nums"
					style="color: {frame.total_return >= 0 ? '#34d399' : '#f87171'}"
				>
					{frame.total_return >= 0 ? '+' : ''}{frame.total_return.toFixed(2)}
				</span>
			</div>

			{#if episodeReturns.length > 1}
				<svg viewBox="0 0 {VW} {VH}" class="mt-2 h-12 w-full" preserveAspectRatio="none" aria-hidden="true">
					<defs>
						<linearGradient id="ret-fill" x1="0" y1="0" x2="0" y2="1">
							<stop offset="0%" stop-color="#5a8cff" stop-opacity="0.35" />
							<stop offset="100%" stop-color="#5a8cff" stop-opacity="0" />
						</linearGradient>
					</defs>
					<path d={area} fill="url(#ret-fill)" />
					<path
						d={line}
						fill="none"
						stroke="#5a8cff"
						stroke-width="1.5"
						stroke-linejoin="round"
						stroke-linecap="round"
						vector-effect="non-scaling-stroke"
					/>
				</svg>
			{/if}
		{:else}
			<p class="font-mono text-xs text-muted-foreground">No live rollout yet.</p>
		{/if}
	</Card.Content>
</Card.Root>
