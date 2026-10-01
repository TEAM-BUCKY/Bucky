<script lang="ts">
	// Tables for a finished sweep: variants side by side, per start angle, and the worst cases.
	import * as Card from '$lib/components/ui/card';
	import type { SweepRecord, SweepResult } from '$lib/lab/api';
	import { METRICS, metricValue } from '$lib/lab/metrics';

	let {
		result,
		variantIdx = $bindable(0),
		onReplay
	}: {
		result: SweepResult;
		variantIdx?: number;
		onReplay: (rec: SweepRecord) => void;
	} = $props();

	const COLS = METRICS.filter((m) =>
		['success', 'time_s', 'wrong_touch', 'path_eff', 'out_of_bounds', 'score'].includes(m.key)
	);
	const variant = $derived(result.variants[variantIdx] ?? result.variants[0]);

	function fmt(key: string, row: Record<string, unknown>): string {
		const v = metricValue(row as never, key);
		return v === null ? '–' : COLS.find((c) => c.key === key)!.format(v);
	}

	function angleLabel(a: number): string {
		const names: Record<number, string> = { 0: 'behind', 90: 'right', 180: 'in front', 270: 'left' };
		return names[a] ? `${a}° · ${names[a]}` : `${a}°`;
	}
</script>

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">Variants</Card.Title>
		<Card.Description class="font-mono text-[11px]">
			{result.n_scenarios} start positions · {result.elapsed_s} s · click a row to inspect it
		</Card.Description>
	</Card.Header>
	<Card.Content class="overflow-x-auto">
		<table class="w-full font-mono text-[11px] tabular-nums">
			<thead class="text-muted-foreground">
				<tr>
					<th class="py-1 pr-2 text-left font-normal">variant</th>
					{#each COLS as c (c.key)}<th class="px-1 py-1 text-right font-normal">{c.label}</th>{/each}
				</tr>
			</thead>
			<tbody>
				{#each result.variants as v, i (v.label)}
					<tr
						class="cursor-pointer border-t border-border hover:bg-muted/50 {i === variantIdx ? 'bg-muted' : ''}"
						onclick={() => (variantIdx = i)}
					>
						<td class="py-1 pr-2 text-left">{v.label}</td>
						{#each COLS as c (c.key)}<td class="px-1 py-1 text-right">{fmt(c.key, v.overall)}</td>{/each}
					</tr>
				{/each}
			</tbody>
		</table>
	</Card.Content>
</Card.Root>

{#if variant.by_angle?.length}
	<Card.Root>
		<Card.Header class="pb-2">
			<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				By start angle · {variant.label}
			</Card.Title>
			<Card.Description class="font-mono text-[11px]">Where the robot starts, seen from the ball</Card.Description>
		</Card.Header>
		<Card.Content class="overflow-x-auto">
			<table class="w-full font-mono text-[11px] tabular-nums">
				<thead class="text-muted-foreground">
					<tr>
						<th class="py-1 pr-2 text-left font-normal">angle</th>
						{#each COLS as c (c.key)}<th class="px-1 py-1 text-right font-normal">{c.label}</th>{/each}
					</tr>
				</thead>
				<tbody>
					{#each variant.by_angle as row (row.angle_deg)}
						<tr class="border-t border-border">
							<td class="py-0.5 pr-2 text-left">{angleLabel(row.angle_deg)}</td>
							{#each COLS as c (c.key)}<td class="px-1 py-0.5 text-right">{fmt(c.key, row)}</td>{/each}
						</tr>
					{/each}
				</tbody>
			</table>
		</Card.Content>
	</Card.Root>
{/if}

<Card.Root>
	<Card.Header class="pb-2">
		<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
			Worst cases · {variant.label}
		</Card.Title>
		<Card.Description class="font-mono text-[11px]">Click to replay</Card.Description>
	</Card.Header>
	<Card.Content class="overflow-x-auto">
		<table class="w-full font-mono text-[11px] tabular-nums">
			<thead class="text-muted-foreground">
				<tr>
					<th class="py-1 pr-2 text-left font-normal">#</th>
					<th class="px-1 py-1 text-right font-normal">ball cm</th>
					<th class="px-1 py-1 text-right font-normal">robot cm</th>
					<th class="px-1 py-1 text-right font-normal">score</th>
					<th class="px-1 py-1 text-left font-normal">why</th>
				</tr>
			</thead>
			<tbody>
				{#each variant.worst as rec (rec.scenario.id)}
					{@const m = rec.metrics}
					<tr class="cursor-pointer border-t border-border hover:bg-muted/50" onclick={() => onReplay(rec)}>
						<td class="py-0.5 pr-2 text-left">{rec.scenario.id}</td>
						<td class="px-1 py-0.5 text-right">
							{(-rec.scenario.ball[1] * 100).toFixed(0)}, {(rec.scenario.ball[0] * 100).toFixed(0)}
						</td>
						<td class="px-1 py-0.5 text-right">
							{(-rec.scenario.robot[1] * 100).toFixed(0)}, {(rec.scenario.robot[0] * 100).toFixed(0)}
						</td>
						<td class="px-1 py-0.5 text-right">{Number(m.score).toFixed(1)}</td>
						<td class="px-1 py-0.5 text-left text-muted-foreground">
							{[
								!m.success && 'not behind',
								m.wrong_touch && 'touched ball',
								m.out_of_bounds && 'out',
								m.wall_hit && 'wall',
								m.own_goal && 'own goal',
								m.ball_out && 'ball out'
							]
								.filter(Boolean)
								.join(', ') || 'slow'}
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</Card.Content>
</Card.Root>
