<script lang="ts">
	import type { RewardTerms } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';

	let { terms, total }: { terms: RewardTerms | null; total: number | null } = $props();

	const TERM_ORDER = [
		'ball_to_goal',
		'approach',
		'speed',
		'possession',
		'front_alignment',
		'goal',
		'out_of_bounds',
		'ball_out',
		'spin',
		'time_penalty',
		'action_smoothness'
	];

	const POS = '#34d399';
	const NEG = '#f87171';

	const scale = $derived(
		terms ? Math.max(0.02, ...TERM_ORDER.map((k) => Math.abs(terms[k] ?? 0))) : 1
	);

	function pct(val: number): number {
		return Math.min((Math.abs(val) / scale) * 50, 50);
	}
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<div class="flex items-baseline justify-between">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Reward
			</Card.Title>
			{#if total !== null}
				<span class="font-mono text-xs tabular-nums" style="color: {total >= 0 ? POS : NEG}">
					Σ {total >= 0 ? '+' : ''}{total.toFixed(3)}
				</span>
			{/if}
		</div>
	</Card.Header>
	<Card.Content>
		{#if terms}
			<div class="flex flex-col gap-1.5">
				{#each TERM_ORDER as key}
					{@const val = terms[key] ?? 0}
					<div>
						<div class="flex justify-between font-mono text-[11px]">
							<span class="text-muted-foreground">{key}</span>
							<span class="tabular-nums" style="color: {val >= 0 ? POS : NEG}">
								{val >= 0 ? '+' : ''}{val.toFixed(4)}
							</span>
						</div>
						<div class="relative mt-0.5 h-1.5 w-full rounded-sm bg-muted/60">
							<div class="absolute inset-y-0 left-1/2 w-px bg-border"></div>
							{#if val >= 0}
								<div
									class="absolute inset-y-0 left-1/2 rounded-r-sm"
									style="width: {pct(val)}%; background: {POS}"
								></div>
							{:else}
								<div
									class="absolute inset-y-0 rounded-l-sm"
									style="right: 50%; width: {pct(val)}%; background: {NEG}"
								></div>
							{/if}
						</div>
					</div>
				{/each}
			</div>
		{:else}
			<p class="font-mono text-xs text-muted-foreground">Waiting for the live rollout…</p>
		{/if}
	</Card.Content>
</Card.Root>
