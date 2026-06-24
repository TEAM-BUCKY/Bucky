<script lang="ts">
	import * as Dialog from '$lib/components/ui/dialog';
	import { simulation } from '$lib/state/simulation.svelte.js';

	// `close` is supplied by DialogContainer so bits-ui's dismissal (outside-click/Escape/X) resolves
	// the DialogsState popup. Optional so the component still satisfies DialogsState's Component type.
	let { close }: { close?: () => void } = $props();

	// The current observation layout (bucky/obs.py). Single-agent is the first 39; self-play
	// appends the 4 sonar dims (43 total). Kept in sync with build_observation().
	const OBS_LABELS = [
		'ball bearing sin', 'ball bearing cos', 'ball distance',
		'ball vel vx', 'ball vel vy', 'own vel vx', 'own vel vy', 'own omega',
		'goal heading sin', 'goal heading cos',
		'edge bearing sin', 'edge bearing cos', 'edge proximity', 'over goal area',
		'teammate x', 'teammate y', 'teammate has_ball', 'kick ready',
		'ball line dist', 'ball vel→line', 'respawn rel x', 'respawn rel y', 'ball out flag',
		'kick1 is_goal', 'kick1 is_robot', 'kick1 is_wall', 'kick1 impact x', 'kick1 impact y', 'kick1 dist',
		'kick2 is_goal', 'kick2 is_robot', 'kick2 is_wall', 'kick2 impact x', 'kick2 impact y', 'kick2 dist',
		'ball→opp goal x', 'ball→opp goal y', 'ball→own goal x', 'ball→own goal y',
		'sonar forward', 'sonar left', 'sonar right', 'sonar back'
	];

	const view = $derived(simulation.evalView);
	const obs = $derived(view?.obs ?? []);
	const rewardInputs = $derived(view?.reward_inputs ?? {});
	const inputRows = $derived(Object.entries(rewardInputs).sort((a, b) => a[0].localeCompare(b[0])));

	// The reward fn's state args s0 (pre-step) and s1 (post-step), flattened to comparable rows.
	const states = $derived(view?.reward_states ?? null);
	const stateRows = $derived.by(() => {
		const s = states;
		if (!s) return [];
		return [
			['robot x', s.s0.robot_pos[0], s.s1.robot_pos[0]],
			['robot y', s.s0.robot_pos[1], s.s1.robot_pos[1]],
			['robot vx', s.s0.robot_vel[0], s.s1.robot_vel[0]],
			['robot vy', s.s0.robot_vel[1], s.s1.robot_vel[1]],
			['heading', s.s0.robot_heading, s.s1.robot_heading],
			['omega', s.s0.robot_omega, s.s1.robot_omega],
			['ball x', s.s0.ball_pos[0], s.s1.ball_pos[0]],
			['ball y', s.s0.ball_pos[1], s.s1.ball_pos[1]],
			['ball vx', s.s0.ball_vel[0], s.s1.ball_vel[0]],
			['ball vy', s.s0.ball_vel[1], s.s1.ball_vel[1]]
		] as [string, number, number][];
	});
	const ACTION_LABELS = ['vx', 'vy', 'ω', 'kick'];
	const actionRows = $derived<{ name: string; vec: number[] }[]>(
		states ? [
			{ name: 'action', vec: states.action },
			{ name: 'prev_action', vec: states.prev_action }
		] : []
	);

	const POS = '#34d399';
	const NEG = '#f87171';
	function pct(v: number): number {
		return Math.min((Math.abs(v) / 3) * 50, 50);
	}
	function isTruthy(v: number | boolean | string[]): boolean {
		return v === true || (typeof v === 'number' && v !== 0) || (Array.isArray(v) && v.length > 0);
	}
</script>

<Dialog.Root open={true} onOpenChange={(o) => { if (!o) close?.(); }}>
	<Dialog.Content class="max-h-[88vh] w-full max-w-[94vw] overflow-hidden sm:max-w-[72rem]">
		<Dialog.Header>
			<Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				Frame inspector
				{#if view}<span class="ml-2 font-normal text-muted-foreground">ep {view.episode} · step {view.step}</span>{/if}
			</Dialog.Title>
		</Dialog.Header>

		{#if !view}
			<p class="font-mono text-xs text-muted-foreground">Run a drill to inspect a frame.</p>
		{:else}
			<div class="grid grid-cols-1 gap-5 overflow-y-auto lg:grid-cols-3">
				<!-- observation (two columns so 43 dims stay compact and labels have room) -->
				<div class="lg:col-span-2">
					<div class="mb-2 font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
						Observation · {obs.length}d
					</div>
					<div class="grid grid-cols-1 gap-x-6 gap-y-0.5 sm:grid-cols-2">
						{#each obs as v, i (i)}
							<div class="flex items-center gap-2">
								<span class="w-5 shrink-0 text-right font-mono text-[10px] text-muted-foreground/60">{i}</span>
								<span class="flex-1 font-mono text-[11px] text-muted-foreground">{OBS_LABELS[i] ?? `dim ${i}`}</span>
								<div class="relative h-1 w-14 shrink-0 rounded-sm bg-muted/60">
									<div class="absolute inset-y-0 left-1/2 w-px bg-border"></div>
									<div class="absolute inset-y-0 rounded-sm" style="{v >= 0 ? `left:50%; width:${pct(v)}%; background:${POS}` : `right:50%; width:${pct(v)}%; background:${NEG}`}"></div>
								</div>
								<span class="w-12 shrink-0 text-right font-mono text-[11px] tabular-nums text-foreground">{v.toFixed(3)}</span>
							</div>
						{/each}
					</div>
				</div>

				<!-- reward fn state args + reward inputs -->
				<div class="flex flex-col gap-4 lg:col-span-1">
					{#if states}
						<div>
							<div class="mb-2 font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
								Reward fn args · states (world frame)
							</div>
							<div class="grid grid-cols-[1fr_auto_auto] gap-x-3 gap-y-0.5">
								<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground/70"></span>
								<span class="text-right font-mono text-[10px] uppercase tracking-wider text-muted-foreground/70">s0</span>
								<span class="text-right font-mono text-[10px] uppercase tracking-wider text-muted-foreground/70">s1</span>
								{#each stateRows as [label, v0, v1] (label)}
									<span class="font-mono text-[11px] text-muted-foreground">{label}</span>
									<span class="w-14 text-right font-mono text-[11px] tabular-nums text-muted-foreground/80">{v0.toFixed(3)}</span>
									<span class="w-14 text-right font-mono text-[11px] tabular-nums {v1 !== v0 ? 'text-foreground' : 'text-muted-foreground/80'}">{v1.toFixed(3)}</span>
								{/each}
							</div>
							<div class="mt-2 flex flex-col gap-0.5">
								{#each actionRows as { name, vec } (name)}
									<div class="flex items-center justify-between gap-2">
										<span class="font-mono text-[11px] text-muted-foreground">{name}</span>
										<span class="font-mono text-[11px] tabular-nums text-foreground">
											{#each vec as a, i (i)}<span class="text-muted-foreground/60">{ACTION_LABELS[i]}</span> {a.toFixed(2)}{i < vec.length - 1 ? '  ' : ''}{/each}
										</span>
									</div>
								{/each}
							</div>
						</div>
					{/if}

					<div>
					<div class="mb-2 font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
						Reward inputs · what compute_rewards saw
					</div>
					{#if inputRows.length}
						<div class="flex flex-col gap-0.5">
							{#each inputRows as [key, val] (key)}
								<div class="flex items-center justify-between gap-2 rounded-sm px-1 py-0.5 odd:bg-muted/30">
									<span class="font-mono text-[11px] text-muted-foreground">{key}</span>
									{#if typeof val === 'boolean'}
										<span class="rounded-full px-1.5 py-0.5 font-mono text-[9px] font-semibold uppercase {val ? 'bg-emerald-500 text-black' : 'bg-muted text-muted-foreground'}">{val}</span>
									{:else if Array.isArray(val)}
										<span class="max-w-[60%] truncate text-right font-mono text-[11px] text-foreground">{val.join(', ') || '—'}</span>
									{:else}
										<span class="font-mono text-[11px] tabular-nums {isTruthy(val) ? 'text-foreground' : 'text-muted-foreground'}">{val}</span>
									{/if}
								</div>
							{/each}
						</div>
					{:else}
						<p class="font-mono text-xs text-muted-foreground">No reward signals this frame.</p>
					{/if}
				</div>
			</div>
		</div>
		{/if}
	</Dialog.Content>
</Dialog.Root>
