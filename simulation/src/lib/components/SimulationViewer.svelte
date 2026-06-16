<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte';
	import SoccerField from './SoccerField.svelte';
	import RewardBreakdown from './RewardBreakdown.svelte';
	import ObservationInspector from './ObservationInspector.svelte';
	import TrainingMetrics from './TrainingMetrics.svelte';

	let wsUrl = $state('ws://localhost:8765');

	const SCALE_X = 610 / 0.9;   // physics lateral (m) → field x (mm)
	const SCALE_Y = 915 / 1.2;   // physics forward (m) → field y (mm)

	const fieldData = $derived(
		simulation.frame
			? {
					allies: [
						{
							x: simulation.frame.robot_pos[1] * SCALE_X,
							y: simulation.frame.robot_pos[0] * SCALE_Y,
							theta: simulation.frame.robot_heading - Math.PI / 2
						}
					],
					ball: {
						x: simulation.frame.ball_pos[1] * SCALE_X,
						y: simulation.frame.ball_pos[0] * SCALE_Y
					}
				}
			: { allies: undefined, ball: undefined }
	);
</script>

<div class="flex h-full flex-col gap-3 p-4">
	<!-- Connection bar -->
	<div class="flex items-center gap-2">
		<input
			type="text"
			bind:value={wsUrl}
			class="rounded border border-border bg-muted px-2 py-1 font-mono text-sm text-foreground w-52"
		/>
		<button
			class="rounded bg-primary px-3 py-1 text-sm text-primary-foreground hover:opacity-90"
			onclick={() => simulation.connect(wsUrl)}
		>
			{simulation.connected ? 'Reconnect' : 'Connect'}
		</button>
		{#if !simulation.connected && simulation.error}
			<span class="text-xs text-destructive">{simulation.error}</span>
		{/if}
		{#if simulation.connected}
			<span class="text-xs text-green-400">● Connected</span>
		{/if}
	</div>

	<!-- Main layout: field + side panel -->
	<div class="flex flex-1 gap-4 overflow-hidden min-h-0">
		<!-- Soccer field -->
		<div class="flex-1 min-w-0">
			<SoccerField
				allies={fieldData.allies}
				ball={fieldData.ball}
				rotation={90}
				class="h-full w-full"
			/>
		</div>

		<!-- Right panel -->
		<div class="flex w-64 shrink-0 flex-col gap-3 overflow-y-auto">
			<TrainingMetrics
				frame={simulation.frame}
				episodeReturns={simulation.episodeReturns}
				connected={simulation.connected}
			/>
			<RewardBreakdown terms={simulation.frame?.reward_terms ?? null} />
			<ObservationInspector frame={simulation.frame} />
		</div>
	</div>
</div>
