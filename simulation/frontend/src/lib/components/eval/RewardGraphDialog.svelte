<script lang="ts">
	import * as Dialog from '$lib/components/ui/dialog';
	import { Button } from '$lib/components/ui/button';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import type { EvalStepFrame } from '$lib/state/simulation.svelte';

	// `close` is supplied by DialogContainer so bits-ui's dismissal (outside-click/Escape/X) resolves
	// the DialogsState popup. Optional so the component still satisfies DialogsState's Component type.
	let { close }: { close?: () => void } = $props();

	const frames = $derived(simulation.evalFrames);
	const cursor = $derived(simulation.evalCursor);

	// Term options: the synthetic `total` plus every per-term key present in the buffer.
	const termItems = $derived.by(() => {
		const keys = frames.length ? Object.keys(frames[0].reward_terms ?? {}) : [];
		return [
			{ value: 'total', label: 'total' },
			...keys.map((k) => ({ value: k, label: k }))
		];
	});

	let term = $state('total');
	let mode = $state<'instant' | 'cumulative'>('instant');

	// Pick the value for a frame under the current term + mode. `total` maps to the step reward
	// (instant) or the episode return so far (cumulative); named terms read the matching dict.
	function valueAt(f: EvalStepFrame): number {
		if (term === 'total') return mode === 'cumulative' ? f.total_return : f.reward_total;
		const src = mode === 'cumulative' ? f.reward_cumulative : f.reward_terms;
		return src?.[term] ?? 0;
	}

	const points = $derived(
		frames.map((f, i) => ({ x: i, y: valueAt(f), ep: f.episode, step: f.step }))
	);

	// --- chart geometry (viewBox stretched to element width via preserveAspectRatio="none") ---
	const W = 800;
	const H = 180;
	const PAD = 6;

	const stats = $derived.by(() => {
		if (points.length === 0) return null;
		let lo = Infinity;
		let hi = -Infinity;
		for (const p of points) {
			if (p.y < lo) lo = p.y;
			if (p.y > hi) hi = p.y;
		}
		const span = hi - lo || Math.abs(hi) || 1;
		return { lo, hi, span };
	});

	const n = $derived(points.length);
	function xFor(i: number): number {
		return PAD + (n <= 1 ? 0 : (i / (n - 1)) * (W - 2 * PAD));
	}
	function yFor(v: number): number {
		if (!stats) return H / 2;
		return PAD + (1 - (v - stats.lo) / stats.span) * (H - 2 * PAD);
	}

	const path = $derived.by(() => {
		if (!stats || n === 0) return '';
		return points
			.map((p, i) => `${i === 0 ? 'M' : 'L'}${xFor(i).toFixed(1)},${yFor(p.y).toFixed(1)}`)
			.join(' ');
	});

	// Dashed reference at y=0 when the series straddles zero.
	const zeroY = $derived(stats && stats.lo < 0 && stats.hi > 0 ? yFor(0) : null);

	// Vertical ticks where the episode index changes — frame the per-episode segments.
	const boundaries = $derived(
		points.filter((p, i) => i > 0 && p.ep !== points[i - 1].ep).map((p) => p.x)
	);

	let hover = $state<number | null>(null);

	// Map a pointer x (client space) back to a buffer index through the stretched viewBox.
	function indexFromEvent(e: MouseEvent): number | null {
		const rect = (e.currentTarget as SVGElement).getBoundingClientRect();
		if (!rect.width || n <= 1) return n === 1 ? 0 : null;
		const vbX = ((e.clientX - rect.left) / rect.width) * W;
		const frac = (vbX - PAD) / (W - 2 * PAD);
		return Math.max(0, Math.min(n - 1, Math.round(frac * (n - 1))));
	}

	function onMove(e: MouseEvent) {
		hover = indexFromEvent(e);
	}
	function onClick(e: MouseEvent) {
		const idx = indexFromEvent(e);
		if (idx !== null) simulation.evalSeek(idx);
	}

	// The frame currently described in the readout: hovered if any, else the playhead.
	const readoutIdx = $derived(hover ?? (cursor >= 0 ? cursor : null));
	const readout = $derived(readoutIdx !== null ? points[readoutIdx] ?? null : null);

	const ACCENT = '#5a8cff';
	function fmt(v: number): string {
		const a = Math.abs(v);
		if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(1);
		return v.toFixed(3);
	}
</script>

<Dialog.Root open={true} onOpenChange={(o) => { if (!o) close?.(); }}>
	<Dialog.Content class="w-full max-w-[94vw] sm:max-w-[60rem]">
		<Dialog.Header>
			<Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Reward graph
				<span class="ml-2 font-normal text-muted-foreground">click a point to seek</span>
			</Dialog.Title>
		</Dialog.Header>

		{#if n === 0}
			<p class="font-mono text-xs text-muted-foreground">Run a drill to plot rewards.</p>
		{:else}
			<div class="flex flex-col gap-3">
				<!-- controls -->
				<div class="flex flex-wrap items-end gap-3">
					<div class="flex flex-col gap-1.5">
						<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Reward term</span>
						<div class="w-56">
							<Combobox bind:value={term} items={termItems} size="sm"
								placeholder="Select a term…" searchPlaceholder="Search terms…" />
						</div>
					</div>
					<div class="flex flex-col gap-1.5">
						<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Signal</span>
						<div class="flex items-center gap-1">
							<Button variant={mode === 'instant' ? 'default' : 'outline'} size="xs"
								class="font-mono text-[10px]" onclick={() => (mode = 'instant')}>
								instant
							</Button>
							<Button variant={mode === 'cumulative' ? 'default' : 'outline'} size="xs"
								class="font-mono text-[10px]" onclick={() => (mode = 'cumulative')}>
								cumulative
							</Button>
						</div>
					</div>
				</div>

				<!-- chart: click/hover seek is an enhancement over the keyboard-accessible scrubber -->
				<!-- svelte-ignore a11y_click_events_have_key_events -->
				<!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
				<svg
					viewBox="0 0 {W} {H}"
					class="h-44 w-full cursor-crosshair rounded-md border border-border/70 bg-card/40"
					preserveAspectRatio="none"
					role="img"
					aria-label="Reward over frames"
					onmousemove={onMove}
					onmouseleave={() => (hover = null)}
					onclick={onClick}
				>
					{#if zeroY !== null}
						<line x1={PAD} y1={zeroY} x2={W - PAD} y2={zeroY} stroke="currentColor"
							class="text-border" stroke-width="1" stroke-dasharray="2 3"
							vector-effect="non-scaling-stroke" />
					{/if}

					{#each boundaries as bx (bx)}
						<line x1={xFor(bx)} y1={PAD} x2={xFor(bx)} y2={H - PAD} stroke="currentColor"
							class="text-border/60" stroke-width="1" vector-effect="non-scaling-stroke" />
					{/each}

					{#if cursor >= 0 && cursor < n}
						<line x1={xFor(cursor)} y1={0} x2={xFor(cursor)} y2={H} stroke="#34d399"
							stroke-width="1.5" vector-effect="non-scaling-stroke" />
					{/if}

					{#if hover !== null}
						<line x1={xFor(hover)} y1={0} x2={xFor(hover)} y2={H} stroke="currentColor"
							class="text-muted-foreground/50" stroke-width="1" vector-effect="non-scaling-stroke" />
						<circle cx={xFor(hover)} cy={yFor(points[hover].y)} r="3" fill={ACCENT}
							vector-effect="non-scaling-stroke" />
					{/if}

					{#if path}
						<path d={path} fill="none" stroke={ACCENT} stroke-width="1.5"
							stroke-linejoin="round" stroke-linecap="round" vector-effect="non-scaling-stroke" />
					{/if}
				</svg>

				<!-- readout -->
				<div class="flex items-center justify-between font-mono text-[11px] tabular-nums text-muted-foreground">
					<span>
						{#if readout}
							frame <span class="text-foreground">{(readoutIdx ?? 0) + 1}</span>/{n}
							· ep <span class="text-foreground">{readout.ep}</span>
							· step <span class="text-foreground">{readout.step}</span>
							{#if hover !== null}<span class="ml-1 text-muted-foreground/70">(hover)</span>{/if}
						{:else}
							hover the graph to inspect a frame
						{/if}
					</span>
					{#if readout}
						<span style="color: {ACCENT}">{term} · {mode}: {fmt(readout.y)}</span>
					{/if}
				</div>
			</div>
		{/if}
	</Dialog.Content>
</Dialog.Root>
