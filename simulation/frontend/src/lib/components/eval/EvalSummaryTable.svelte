<script lang="ts">
	import type { EvalSummary } from '$lib/state/simulation.svelte';
	import * as Card from '$lib/components/ui/card';

	let { summary }: { summary: EvalSummary | null } = $props();

	const POS = '#34d399';
	const NEG = '#f87171';

	// Per-term means, largest magnitude first — the headline of what the policy farmed.
	const terms = $derived.by(() => {
		if (!summary) return [];
		return Object.entries(summary.per_term_mean)
			.filter(([, v]) => Math.abs(v) > 1e-6)
			.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]))
			.map(([k, mean]) => ({ key: k, mean, std: summary.per_term_std[k] ?? 0 }));
	});
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
			Aggregate · {summary ? `${summary.n_episodes} episodes` : '—'}
		</Card.Title>
	</Card.Header>
	<Card.Content class="flex flex-col gap-3">
		{#if !summary}
			<p class="font-mono text-xs text-muted-foreground">
				Runs to completion to show mean return, success rate and per-term totals.
			</p>
		{:else}
			<div class="grid grid-cols-2 gap-2">
				<div class="rounded-md border border-border/70 bg-card/40 p-2.5">
					<div class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Mean return</div>
					<div class="font-mono text-sm tabular-nums" style="color: {summary.return_mean >= 0 ? POS : NEG}">
						{summary.return_mean >= 0 ? '+' : ''}{summary.return_mean.toFixed(2)}
						<span class="text-muted-foreground">± {summary.return_std.toFixed(2)}</span>
					</div>
				</div>
				<div class="rounded-md border border-border/70 bg-card/40 p-2.5">
					<div class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Success rate</div>
					<div class="font-mono text-sm tabular-nums text-foreground">
						{(summary.success_rate * 100).toFixed(0)}%
					</div>
				</div>
			</div>

			<div class="flex flex-wrap gap-x-4 gap-y-1 font-mono text-[10px] text-muted-foreground">
				<span>{summary.deterministic ? 'deterministic' : 'stochastic'}</span>
				{#if summary.opponent}<span>opponent: {summary.opponent}</span>{/if}
			</div>

			<div class="flex flex-col gap-1">
				<div class="flex justify-between font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
					<span>Term</span><span>mean ± std / ep</span>
				</div>
				{#each terms as t (t.key)}
					<div class="flex justify-between font-mono text-[11px]">
						<span class="text-muted-foreground">{t.key}</span>
						<span class="tabular-nums" style="color: {t.mean >= 0 ? POS : NEG}">
							{t.mean >= 0 ? '+' : ''}{t.mean.toFixed(3)}
							<span class="text-muted-foreground">± {t.std.toFixed(3)}</span>
						</span>
					</div>
				{/each}
			</div>
		{/if}
	</Card.Content>
</Card.Root>
