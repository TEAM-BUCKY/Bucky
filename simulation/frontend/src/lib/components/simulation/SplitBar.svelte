<script lang="ts">
	/**
	 * A horizontal divider bar split into N segments whose widths are editable by dragging the
	 * handles between them. ``values`` are percentages that always sum to 100; dragging a handle
	 * transfers budget between the two adjacent segments only (the rest stay put).
	 */
	interface Props {
		/** Percentages per segment; kept summing to 100. Bindable. */
		values: number[];
		/** Label shown on each segment. */
		labels: string[];
		/** Optional per-segment accent colors. */
		colors?: string[];
		/** Minimum percentage a segment may shrink to. */
		min?: number;
	}

	let {
		values = $bindable(),
		labels,
		colors = ['#3c78dc', '#d9a441', '#3cba6b'],
		min = 4
	}: Props = $props();

	let bar: HTMLDivElement;
	let active = $state<number | null>(null); // index of the handle being dragged

	const cum = (i: number) => values.slice(0, i).reduce((a, b) => a + b, 0);

	function onDown(i: number, e: PointerEvent) {
		active = i;
		(e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
		e.preventDefault();
	}

	function onMove(i: number, e: PointerEvent) {
		if (active !== i || !bar) return;
		const rect = bar.getBoundingClientRect();
		const pair = values[i] + values[i + 1]; // budget shared by the two adjacent segments
		const pos = ((e.clientX - rect.left) / rect.width) * 100; // 0..100 across the whole bar
		let left = pos - cum(i); // desired size of segment i
		left = Math.max(min, Math.min(pair - min, left));
		const next = [...values];
		next[i] = Math.round(left);
		next[i + 1] = pair - next[i]; // keep the pair (and therefore the total) exact
		values = next;
	}

	function onUp() {
		active = null;
	}
</script>

<div class="flex flex-col gap-1.5">
	<div
		bind:this={bar}
		class="relative flex h-8 w-full select-none overflow-hidden rounded-md border border-border/70 bg-input/20"
	>
		{#each values as v, i (i)}
			<div
				class="flex min-w-0 items-center justify-center overflow-hidden"
				style="width:{v}%; background:{colors[i % colors.length]}33;"
				title="{labels[i]}: {v}%"
			>
				<span class="truncate px-1 font-mono text-[10px] font-semibold text-foreground/80">{v}%</span>
			</div>
		{/each}

		<!-- Draggable handles between adjacent segments. -->
		{#each values.slice(0, -1) as _, i (i)}
			<button
				type="button"
				class="absolute top-0 z-10 h-full w-3 -translate-x-1/2 cursor-ew-resize touch-none"
				style="left:{cum(i + 1)}%"
				aria-label="Adjust {labels[i]} / {labels[i + 1]} split"
				onpointerdown={(e) => onDown(i, e)}
				onpointermove={(e) => onMove(i, e)}
				onpointerup={onUp}
				onpointercancel={onUp}
			>
				<span class="mx-auto block h-full w-0.5 rounded bg-foreground/50"></span>
			</button>
		{/each}
	</div>

	<!-- Legend -->
	<div class="flex flex-wrap gap-x-3 gap-y-0.5">
		{#each labels as label, i (i)}
			<span class="flex items-center gap-1 font-mono text-[10px] text-muted-foreground">
				<span class="inline-block h-2 w-2 rounded-sm" style="background:{colors[i % colors.length]}"></span>
				{label} {values[i]}%
			</span>
		{/each}
	</div>
</div>
