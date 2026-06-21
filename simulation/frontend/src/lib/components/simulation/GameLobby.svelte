<script lang="ts">
	// "Play a friend by game code" lobby + live play view.
	//   • No game yet → Create (casual/full-match) or Join (enter a code).
	//   • In a game   → the live field + a "You are BLUE/RED" banner + the control overlay.
	// Open guest play: no login. The per-side token (held in state) authorises control.
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import SoccerField from '../SoccerField.svelte';
	import ManualControl from './ManualControl.svelte';
	import { Button } from '$lib/components/ui/button';
	import { Input } from '$lib/components/ui/input';
	import * as Card from '$lib/components/ui/card';
	import { Copy, LogOut, Gamepad2, Users, Loader } from '@lucide/svelte';

	// Physics (m) → field (mm), matching SimulationViewer: rendered rotated 90°, uniform 1000 mm/m.
	const SCALE = 1000;
	const fieldData = $derived.by(() => {
		const f = simulation.gameFrame;
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
	const score = $derived(simulation.gameFrame?.score ?? null);

	// Lobby form state.
	let mode = $state<'casual' | 'match'>('casual');
	let joinCode = $state('');
	let busy = $state(false);

	const game = $derived(simulation.game);
	const playing = $derived(!!game && !game.spectator && !!game.side);
	// You are BLUE (side a, attacks the yellow goal) / RED (side b, attacks the blue goal).
	const youColor = $derived(game?.side === 'a' ? '#3c78dc' : '#dc3c3c');
	const youLabel = $derived(
		game?.side === 'a' ? 'BLUE' : game?.side === 'b' ? 'RED' : null
	);
	const attackGoal = $derived(game?.side === 'a' ? 'yellow' : 'blue');

	let fieldEl = $state<HTMLElement | null>(null);
	let copied = $state(false);

	async function create() {
		busy = true;
		await simulation.createGame(mode, 'a');
		busy = false;
	}
	async function join() {
		busy = true;
		await simulation.joinGame(joinCode);
		busy = false;
	}
	async function leave() {
		await simulation.leaveGame();
		joinCode = '';
	}
	async function copyCode() {
		if (!game) return;
		try {
			await navigator.clipboard.writeText(game.code);
			copied = true;
			setTimeout(() => (copied = false), 1500);
		} catch {
			/* clipboard unavailable */
		}
	}

	onMount(() => {
		simulation.connect();
		// Poll the room so "waiting for opponent" clears when they join, and so a game that
		// ended (match stopped / room reaped) returns us to the lobby.
		const poll = setInterval(() => void simulation.refreshGame(), 2000);
		return () => {
			clearInterval(poll);
			simulation.disconnect();
		};
	});
</script>

<div class="mx-auto flex min-h-screen w-full max-w-2xl flex-col gap-4 p-4">
	<div class="flex items-center justify-between gap-3">
		<h1 class="flex items-center gap-2 font-mono text-base font-bold tracking-tight text-foreground">
			<Gamepad2 class="size-5" />Play a friend
		</h1>
		<Button href="/viz" variant="ghost" size="xs" class="font-mono text-[11px]">Trainer view</Button>
	</div>

	{#if !game}
		<!-- ── Lobby: create or join ─────────────────────────────────────────── -->
		<Card.Root>
			<Card.Header>
				<Card.Title class="font-mono text-sm">Start a new game</Card.Title>
				<Card.Description class="font-mono text-[11px]">
					Create a game, share the code, and your friend joins from their own PC.
				</Card.Description>
			</Card.Header>
			<Card.Content class="flex flex-col gap-3">
				<div class="flex gap-2">
					<Button
						variant={mode === 'casual' ? 'default' : 'outline'}
						size="sm"
						class="flex-1 font-mono text-[11px]"
						onclick={() => (mode = 'casual')}
					>Casual (endless)</Button>
					<Button
						variant={mode === 'match' ? 'default' : 'outline'}
						size="sm"
						class="flex-1 font-mono text-[11px]"
						onclick={() => (mode = 'match')}
					>Full match (2×7 min)</Button>
				</div>
				<Button class="font-mono text-[11px]" disabled={busy} onclick={create}>
					{#if busy}<Loader class="size-3.5 animate-spin" />{/if}Create game
				</Button>
			</Card.Content>
		</Card.Root>

		<Card.Root>
			<Card.Header>
				<Card.Title class="font-mono text-sm">Join with a code</Card.Title>
			</Card.Header>
			<Card.Content class="flex gap-2">
				<Input
					placeholder="ABCD"
					bind:value={joinCode}
					maxlength={6}
					class="font-mono text-sm uppercase tracking-[0.3em]"
					onkeydown={(e: KeyboardEvent) => e.key === 'Enter' && join()}
				/>
				<Button class="font-mono text-[11px]" disabled={busy || !joinCode.trim()} onclick={join}>
					Join
				</Button>
			</Card.Content>
		</Card.Root>

		{#if simulation.gameError}
			<p class="font-mono text-[11px] text-red-400">{simulation.gameError}</p>
		{/if}
	{:else}
		<!-- ── In a game: banner + live field + controls ─────────────────────── -->
		<div class="flex items-center justify-between gap-2">
			{#if playing && youLabel}
				<span
					class="flex items-center gap-2 rounded-full border px-3 py-1 font-mono text-[11px] font-bold uppercase tracking-wider"
					style="border-color: {youColor}; color: {youColor}"
				>
					You are {youLabel} — attack the {attackGoal} goal
				</span>
			{:else}
				<span class="flex items-center gap-1.5 font-mono text-[11px] uppercase tracking-wider text-muted-foreground">
					<Users class="size-3.5" />Spectating
				</span>
			{/if}
			<Button variant="outline" size="sm" class="font-mono text-[11px]" onclick={leave}>
				<LogOut class="size-3.5" />Leave
			</Button>
		</div>

		<div class="flex flex-wrap items-center gap-3">
			<button
				class="flex items-center gap-2 rounded-md border border-border bg-background/60 px-3 py-1.5 font-mono text-sm tracking-[0.3em] text-foreground"
				title="Copy game code"
				onclick={copyCode}
			>
				{game.code}<Copy class="size-3.5 text-muted-foreground" />
			</button>
			{#if copied}<span class="font-mono text-[10px] text-emerald-400">copied</span>{/if}
			<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
				{game.mode === 'match' ? 'full match' : 'casual'}
			</span>
			{#if playing && !game.opponentPresent}
				<span class="flex items-center gap-1.5 font-mono text-[10px] text-amber-400">
					<Loader class="size-3 animate-spin" />waiting for opponent…
				</span>
			{/if}
		</div>

		<Card.Root class="relative mx-auto flex aspect-[303/242] w-full items-center justify-center p-3">
			{#if score}
				<div class="absolute left-1/2 top-3 z-10 -translate-x-1/2">
					<div class="flex items-center gap-3 rounded-full border border-border bg-background/80 px-4 py-1.5 backdrop-blur">
						<span class="font-mono text-[10px] uppercase tracking-wider" style="color:#3c78dc">Blue</span>
						<span class="font-mono text-lg font-bold tabular-nums" style="color:#3c78dc">{score.a}</span>
						<span class="font-mono text-sm text-muted-foreground">:</span>
						<span class="font-mono text-lg font-bold tabular-nums" style="color:#dc3c3c">{score.b}</span>
						<span class="font-mono text-[10px] uppercase tracking-wider" style="color:#dc3c3c">Red</span>
					</div>
				</div>
			{/if}
			<div
				bind:this={fieldEl}
				class="flex h-full w-full items-center justify-center {playing ? 'cursor-crosshair' : ''}"
			>
				<SoccerField
					fit
					allies={fieldData.allies}
					enemies={fieldData.enemies}
					ball={fieldData.ball}
					showBall={!!simulation.gameFrame}
					rotation={90}
					class="border-0"
				/>
			</div>
		</Card.Root>

		{#if playing && game.side}
			<ManualControl {fieldEl} side={game.side} context="game" />
		{:else if !simulation.gameFrame}
			<p class="font-mono text-[11px] text-muted-foreground">Connecting to the match…</p>
		{/if}
	{/if}
</div>
