<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import { Button } from '$lib/components/ui/button';
	import { Gauge } from '@lucide/svelte';

	const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];
	const running = $derived(simulation.evalStatus.running);
</script>

<div class="flex items-center gap-2">
	<Gauge class="size-3.5 text-muted-foreground" />
	<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Speed</span>
	<div class="flex items-center gap-1">
		{#each SPEEDS as s (s)}
			<Button
				variant={simulation.evalSpeed === s ? 'default' : 'outline'}
				size="xs"
				class="font-mono text-[11px] tabular-nums"
				disabled={!running}
				onclick={() => simulation.setEvalSpeed(s)}
			>
				{s}×
			</Button>
		{/each}
	</div>
</div>
