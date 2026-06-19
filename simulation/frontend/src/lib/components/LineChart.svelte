<script lang="ts">
	import type { MetricPoint } from '$lib/state/simulation.svelte';

	let {
		label,
		points,
		color = '#5a8cff',
		precision = 3
	}: { label: string; points: MetricPoint[]; color?: string; precision?: number } = $props();

	const VW = 240;
	const VH = 56;
	const PAD = 3;

	const stats = $derived.by(() => {
		if (points.length === 0) return null;
		let lo = Infinity;
		let hi = -Infinity;
		for (const p of points) {
			if (p.y < lo) lo = p.y;
			if (p.y > hi) hi = p.y;
		}
		const span = hi - lo || Math.abs(hi) || 1;
		return { lo, hi, span, last: points[points.length - 1].y };
	});

	const path = $derived.by(() => {
		if (!stats || points.length === 0) return '';
		const n = points.length;
		return points
			.map((p, i) => {
				const x = PAD + (n === 1 ? 0 : (i / (n - 1)) * (VW - 2 * PAD));
				const y = PAD + (1 - (p.y - stats.lo) / stats.span) * (VH - 2 * PAD);
				return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`;
			})
			.join(' ');
	});

	// Baseline at y=0 if the series straddles zero — gives the curve a reference.
	const zeroY = $derived(
		stats && stats.lo < 0 && stats.hi > 0
			? PAD + (1 - (0 - stats.lo) / stats.span) * (VH - 2 * PAD)
			: null
	);

	function fmt(v: number): string {
		const a = Math.abs(v);
		if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(1);
		return v.toFixed(precision);
	}
</script>

<div class="rounded-md border border-border/70 bg-card/40 p-2.5">
	<div class="flex items-baseline justify-between gap-2">
		<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">{label}</span>
		<span class="font-mono text-xs tabular-nums" style="color: {color}">
			{stats ? fmt(stats.last) : '—'}
		</span>
	</div>
	<svg viewBox="0 0 {VW} {VH}" class="mt-1.5 h-12 w-full" preserveAspectRatio="none" aria-hidden="true">
		{#if zeroY !== null}
			<line
				x1={PAD}
				y1={zeroY}
				x2={VW - PAD}
				y2={zeroY}
				stroke="currentColor"
				class="text-border"
				stroke-width="1"
				stroke-dasharray="2 3"
				vector-effect="non-scaling-stroke"
			/>
		{/if}
		{#if path}
			<path
				d={path}
				fill="none"
				stroke={color}
				stroke-width="1.5"
				stroke-linejoin="round"
				stroke-linecap="round"
				vector-effect="non-scaling-stroke"
			/>
		{/if}
	</svg>
</div>
