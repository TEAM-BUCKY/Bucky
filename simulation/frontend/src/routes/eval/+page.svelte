<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import SoccerField from '$lib/components/SoccerField.svelte';
	import RewardBreakdown from '$lib/components/simulation/panels/RewardBreakdown.svelte';
	import LoginControl from '$lib/components/simulation/LoginControl.svelte';
	import EvalConfigPanel from '$lib/components/eval/EvalConfigPanel.svelte';
	import CumulativeRewardPanel from '$lib/components/eval/CumulativeRewardPanel.svelte';
	import EvalSummaryTable from '$lib/components/eval/EvalSummaryTable.svelte';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { ChartLine, Boxes, Gamepad2 } from '@lucide/svelte';

	onMount(() => {
		simulation.connect();
		return () => simulation.disconnect();
	});

	// Physics (m) → field (mm), same transform as the training viewer: screen X = physics y,
	// screen Y = physics x, heading (pi/2 - h). Robot B (self-play) renders as an enemy.
	const SCALE = 1000;
	const fieldData = $derived.by(() => {
		const f = simulation.evalFrame;
		if (!f) return { allies: [], enemies: [], ball: undefined };
		const allies = [
			{ x: f.robot_pos[1] * SCALE, y: f.robot_pos[0] * SCALE, theta: Math.PI / 2 - f.robot_heading }
		];
		const enemies = f.robot2_pos
			? [{ x: f.robot2_pos[1] * SCALE, y: f.robot2_pos[0] * SCALE, theta: Math.PI / 2 - (f.robot2_heading ?? 0) }]
			: [];
		const ball = { x: f.ball_pos[1] * SCALE, y: f.ball_pos[0] * SCALE };
		return { allies, enemies, ball };
	});

	const running = $derived(simulation.evalStatus.running);
	const conn = $derived.by(() => {
		if (simulation.connected) return { label: 'Connected', color: '#34d399' };
		return { label: 'Reconnecting…', color: '#f87171' };
	});
</script>

<svelte:head>
	<title>Bucky · Eval</title>
</svelte:head>

<div class="h-screen bg-background text-foreground">
	<div class="flex h-full flex-col gap-4 p-4">
		<Card.Root>
			<Card.Content class="px-4 py-3">
				<div class="flex flex-wrap items-center justify-between gap-3">
					<div class="flex items-baseline gap-3">
						<h1 class="font-mono text-base font-bold tracking-tight text-foreground">BUCKY · EVAL</h1>
						<span class="font-mono text-xs text-muted-foreground">
							{simulation.evalRun ?? 'no evaluation'}
							{#if running}<span class="text-foreground/70"> · {simulation.evalStatus.phase ?? 'running'}</span>{/if}
						</span>
					</div>
					<div class="flex items-center gap-2">
						<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
							<span class="size-2 rounded-full" style="background: {conn.color}"></span>
							<span class="font-mono text-[11px] text-muted-foreground">{conn.label}</span>
						</span>
						<Button href="/viz" variant="ghost" size="xs" class="font-mono text-[11px]">
							<ChartLine class="size-3" />Train
						</Button>
						<Button href="/play" variant="ghost" size="xs" class="font-mono text-[11px]">
							<Gamepad2 class="size-3" />Play a friend
						</Button>
						<Button href="/overview" variant="ghost" size="xs" class="font-mono text-[11px]">
							<Boxes class="size-3" />Overview
						</Button>
						<LoginControl />
					</div>
				</div>
			</Card.Content>
		</Card.Root>

		<div class="grid min-h-0 flex-1 grid-cols-1 gap-4 lg:grid-cols-[20rem_minmax(0,1fr)_22rem]">
			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto">
				<EvalConfigPanel />
				<EvalSummaryTable summary={simulation.evalSummary} />
			</div>

			<div class="flex min-w-0 flex-col gap-4 lg:min-h-0">
				<Card.Root class="relative mx-auto flex aspect-[303/242] w-full max-w-2xl shrink-0 items-center justify-center p-3">
					<div class="flex h-full w-full items-center justify-center">
						<SoccerField
							fit
							allies={fieldData.allies}
							enemies={fieldData.enemies}
							ball={fieldData.ball}
							showBall={!!simulation.evalFrame}
							rotation={90}
							class="border-0"
						/>
					</div>
				</Card.Root>
				<RewardBreakdown
					terms={simulation.evalFrame?.reward_terms ?? null}
					total={simulation.evalFrame?.reward_total ?? null}
				/>
			</div>

			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto">
				<CumulativeRewardPanel
					cumulative={simulation.evalFrame?.reward_cumulative ?? null}
					total={simulation.evalFrame?.total_return ?? null}
					history={simulation.evalCumulativeHistory}
					episode={simulation.evalFrame?.episode ?? null}
					step={simulation.evalFrame?.step ?? null}
				/>
			</div>
		</div>
	</div>
</div>
