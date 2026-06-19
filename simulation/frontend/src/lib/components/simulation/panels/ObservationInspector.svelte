<script lang="ts">
	import type { SimFrame } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';

	let { frame }: { frame: SimFrame | null } = $props();

	const LABELS = [
		'ball bearing sin',
		'ball bearing cos',
		'ball distance',
		'ball vel vx',
		'ball vel vy',
		'own vel vx',
		'own vel vy',
		'own omega',
		'goal heading sin',
		'goal heading cos',
		'edge bearing sin',
		'edge bearing cos',
		'edge proximity',
		'over goal',
		'teammate x',
		'teammate y',
		'teammate has_ball',
		'sonar forward',
		'sonar left',
		'sonar right',
		'sonar back'
	];

	const obs = $derived(frame?.obs ?? []);

	function pct(val: number): number {
		return Math.min((Math.abs(val) / 3) * 50, 50);
	}
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
			Observation · {obs.length}d
		</Card.Title>
	</Card.Header>
	<Card.Content>
		{#if frame && obs.length}
			<div class="flex flex-col gap-1">
				{#each obs as v, i}
					{@const label = LABELS[i] ?? `dim ${i}`}
					<div class="flex items-center gap-2">
						<span class="w-4 shrink-0 text-right font-mono text-[10px] text-muted-foreground/60">{i}</span>
						<span class="flex-1 truncate font-mono text-[11px] text-muted-foreground">{label}</span>
						<div class="relative h-1 w-12 shrink-0 rounded-sm bg-muted/60">
							<div class="absolute inset-y-0 left-1/2 w-px bg-border"></div>
							<div
								class="absolute inset-y-0 rounded-sm bg-foreground/55"
								style={v >= 0 ? `left:50%; width:${pct(v)}%` : `right:50%; width:${pct(v)}%`}
							></div>
						</div>
						<span class="w-12 shrink-0 text-right font-mono text-[11px] tabular-nums text-foreground">
							{v.toFixed(2)}
						</span>
					</div>
				{/each}
			</div>
		{:else}
			<p class="font-mono text-xs text-muted-foreground">Waiting…</p>
		{/if}
	</Card.Content>
</Card.Root>
