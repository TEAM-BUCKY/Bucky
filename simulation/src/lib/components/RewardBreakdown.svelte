<script lang="ts">
	import type { RewardTerms } from '$lib/state/simulation.svelte';

	let { terms }: { terms: RewardTerms | null } = $props();

	const TERM_ORDER = [
		'ball_to_goal',
		'approach',
		'possession',
		'goal',
		'out_of_bounds',
		'spin',
		'time_penalty',
		'action_magnitude'
	];

	function barWidth(val: number): string {
		const pct = Math.min(Math.abs(val) * 200, 100);
		return `${pct}%`;
	}
</script>

<div class="rounded-lg border border-border bg-card p-3 text-xs font-mono">
	<div class="mb-2 text-sm font-semibold text-foreground">Reward Terms</div>
	{#if terms}
		{#each TERM_ORDER as key}
			{@const val = terms[key] ?? 0}
			<div class="mb-1">
				<div class="flex justify-between text-muted-foreground">
					<span>{key}</span>
					<span class={val >= 0 ? 'text-green-400' : 'text-red-400'}>
						{val >= 0 ? '+' : ''}{val.toFixed(4)}
					</span>
				</div>
				<div class="mt-0.5 h-1.5 w-full rounded bg-muted">
					<div
						class="h-full rounded {val >= 0 ? 'bg-green-500' : 'bg-red-500'}"
						style="width: {barWidth(val)}"
					></div>
				</div>
			</div>
		{/each}
	{:else}
		<p class="text-muted-foreground">Waiting for data…</p>
	{/if}
</div>
