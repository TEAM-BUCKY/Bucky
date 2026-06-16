<script lang="ts">
	import type { SimFrame } from '$lib/state/simulation.svelte';

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
		'teammate has_ball'
	];

	const FIELD_DIAG = Math.sqrt(2.4 ** 2 + 1.8 ** 2);
	const FIELD_W = 2.4;

	const obs = $derived((): number[] => {
		if (!frame) return Array(17).fill(0);
		const dx = frame.ball_pos[0] - frame.robot_pos[0];
		const dy = frame.ball_pos[1] - frame.robot_pos[1];
		const bearing = Math.atan2(dy, dx) - frame.robot_heading;
		const dist = Math.sqrt(dx * dx + dy * dy) / FIELD_DIAG;

		const goalDx = FIELD_W / 2 - frame.robot_pos[0];
		const goalDy = -frame.robot_pos[1];
		const headingToGoal = Math.atan2(goalDy, goalDx) - frame.robot_heading;

		return [
			Math.sin(bearing),
			Math.cos(bearing),
			dist,
			0, 0, // ball vel (not in frame)
			0, 0, // own vel (not in frame)
			0,    // own omega (not in frame)
			Math.sin(headingToGoal),
			Math.cos(headingToGoal),
			0, 0, 0, // edge (not in frame)
			0,       // over goal
			0, 0, 0  // teammate
		];
	});
</script>

<div class="rounded-lg border border-border bg-card p-3 text-xs font-mono">
	<div class="mb-2 text-sm font-semibold text-foreground">Observation Vector</div>
	{#if frame}
		{#each LABELS as label, i}
			<div class="mb-0.5 flex justify-between gap-2">
				<span class="text-muted-foreground">[{i}] {label}</span>
				<span class="text-foreground">{obs()[i].toFixed(3)}</span>
			</div>
		{/each}
	{:else}
		<p class="text-muted-foreground">Waiting…</p>
	{/if}
</div>
