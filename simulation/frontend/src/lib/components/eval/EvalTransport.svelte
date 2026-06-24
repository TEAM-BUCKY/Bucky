<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import { Button } from '$lib/components/ui/button';
	import { Slider } from '$lib/components/ui/slider';
	import { SkipBack, ChevronLeft, ChevronRight, Play, Pause } from '@lucide/svelte';

	const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];

	const running = $derived(simulation.evalStatus.running);
	const frames = $derived(simulation.evalFrames);
	const playing = $derived(simulation.evalPlaying);
	const view = $derived(simulation.evalView);
	const atEdge = $derived(simulation.evalAtLiveEdge);
	const canSeek = $derived(frames.length > 1);
	const live = $derived(running && playing && atEdge);
</script>

<div class="rounded-lg border border-border bg-background/85 px-3 py-2 shadow-lg backdrop-blur">
	<div class="flex items-center justify-between gap-3">
		<!-- transport -->
		<div class="flex items-center gap-1">
			<Button variant="ghost" size="xs" title="Jump to episode start"
				disabled={!frames.length} onclick={() => simulation.evalRewind()}>
				<SkipBack class="size-3.5" />
			</Button>
			<Button variant="ghost" size="xs" title="Step back one frame"
				disabled={!frames.length} onclick={() => simulation.evalStepBack()}>
				<ChevronLeft class="size-3.5" />
			</Button>
			<Button variant={playing ? 'default' : 'outline'} size="xs"
				title={playing ? 'Pause' : 'Play'} disabled={!running}
				onclick={() => simulation.setEvalPlaying(!playing)}>
				{#if playing}<Pause class="size-3.5" />{:else}<Play class="size-3.5" />{/if}
			</Button>
			<Button variant="ghost" size="xs" title="Step forward one frame"
				disabled={!frames.length || (atEdge && !running)} onclick={() => simulation.evalStepForward()}>
				<ChevronRight class="size-3.5" />
			</Button>
		</div>

		<!-- position / state -->
		<div class="flex items-center gap-2 font-mono text-[11px] tabular-nums text-muted-foreground">
			{#if view}
				<span>ep <span class="text-foreground">{view.episode}</span></span>
				<span>step <span class="text-foreground">{view.step}</span></span>
			{/if}
			<span
				class="rounded-full px-1.5 py-0.5 text-[9px] font-semibold uppercase tracking-wider"
				class:bg-emerald-500={live}
				class:text-black={live}
				class:bg-muted={!live}
			>
				{live ? 'Live' : running ? 'Paused' : 'Review'}
			</span>
		</div>

		<!-- speed -->
		<div class="flex items-center gap-1">
			{#each SPEEDS as s (s)}
				<Button
					variant={simulation.evalSpeed === s ? 'default' : 'outline'}
					size="xs"
					class="px-1.5 font-mono text-[10px] tabular-nums"
					disabled={!running}
					onclick={() => simulation.setEvalSpeed(s)}
				>
					{s}×
				</Button>
			{/each}
		</div>
	</div>

	<!-- scrubber -->
	<div class="mt-2 flex items-center gap-2">
		<Slider
			type="single"
			min={0}
			max={Math.max(0, frames.length - 1)}
			step={1}
			value={simulation.evalCursor < 0 ? 0 : simulation.evalCursor}
			disabled={!canSeek}
			onValueChange={(v: number) => simulation.evalSeek(v)}
			class="flex-1"
		/>
		<span class="w-16 shrink-0 text-right font-mono text-[10px] tabular-nums text-muted-foreground">
			{frames.length ? simulation.evalCursor + 1 : 0}/{frames.length}
		</span>
	</div>
</div>
