<script lang="ts">
	// Result of a best-fit search (params set to "*"): the best values and the top candidates tried.
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import type { OptimizeResult, ParamValue } from '$lib/lab/api';

	let {
		opt,
		onApply
	}: {
		opt: OptimizeResult;
		/** Write the best values into the param form (replacing the "*"). */
		onApply: (best: Record<string, ParamValue>) => void;
	} = $props();

	const names = $derived(opt.axes.map((a) => a.name));
	const top = $derived(opt.history.slice(0, 10));
	const startScore = $derived(
		opt.history.find((h) => names.every((n) => h.params[n] === opt.start[n]))?.score ?? null
	);
	const range = (a: OptimizeResult['axes'][number]) =>
		a.choices ? a.choices.map(String).join(' / ') : `${a.lo} … ${a.hi}`;
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="flex items-baseline justify-between font-mono text-xs font-semibold uppercase tracking-widest">
			Best fit
			<span class="text-[10px] font-normal tracking-normal normal-case text-muted-foreground tabular-nums">
				{opt.history.length} tried
			</span>
		</Card.Title>
	</Card.Header>
	<Card.Content class="flex flex-col gap-3">
		<table class="w-full font-mono text-[11px] tabular-nums">
			<thead class="text-muted-foreground">
				<tr>
					<th class="py-1 pr-2 text-left font-normal">param</th>
					<th class="px-1 py-1 text-right font-normal">searched</th>
					<th class="px-1 py-1 text-right font-normal">start</th>
					<th class="px-1 py-1 text-right font-normal">best</th>
				</tr>
			</thead>
			<tbody>
				{#each opt.axes as a (a.name)}
					<tr class="border-t border-border">
						<td class="py-1 pr-2 text-left">{a.name}</td>
						<td class="px-1 py-1 text-right text-muted-foreground">{range(a)}</td>
						<td class="px-1 py-1 text-right text-muted-foreground">{String(opt.start[a.name])}</td>
						<td class="px-1 py-1 text-right font-semibold">{String(opt.best[a.name])}</td>
					</tr>
				{/each}
				<tr class="border-t border-border">
					<td class="py-1 pr-2 text-left">{opt.objective}</td>
					<td></td>
					<td class="px-1 py-1 text-right text-muted-foreground">{startScore === null ? '–' : startScore.toFixed(2)}</td>
					<td class="px-1 py-1 text-right font-semibold">{opt.best_score.toFixed(2)}</td>
				</tr>
			</tbody>
		</table>
		<Button size="xs" variant="outline" class="self-start font-mono text-[11px]" onclick={() => onApply(opt.best)}>
			Use best
		</Button>
	</Card.Content>
</Card.Root>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">Top {top.length}</Card.Title>
	</Card.Header>
	<Card.Content class="overflow-x-auto">
		<table class="w-full font-mono text-[11px] tabular-nums">
			<thead class="text-muted-foreground">
				<tr>
					{#each names as n (n)}<th class="px-1 py-1 text-right font-normal">{n}</th>{/each}
					<th class="px-1 py-1 text-right font-normal">{opt.objective}</th>
				</tr>
			</thead>
			<tbody>
				{#each top as h, i (i)}
					<tr class="border-t border-border {i === 0 ? 'font-semibold' : ''}">
						{#each names as n (n)}<td class="px-1 py-0.5 text-right">{String(h.params[n])}</td>{/each}
						<td class="px-1 py-0.5 text-right">{h.score.toFixed(2)}</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</Card.Content>
</Card.Root>
