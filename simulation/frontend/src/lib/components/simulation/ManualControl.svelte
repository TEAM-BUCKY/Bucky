<script lang="ts">
	// Human control overlay for driving one robot. The mouse (or touch) aims its front, W/S
	// drive forward/back, A/D strafe, and Space / left-click fire the kicker. The computed 4-D
	// action [vx, vy, omega, kick] is pushed to the backend at ~30 Hz, which relays it to the
	// match's play.py process. Used in two contexts:
	//   • context="admin" → the legacy single-human "test against the AI" feature (side 'b').
	//   • context="game"  → online play-by-code: this browser drives its claimed side.
	// The body-frame action is side-agnostic (physics applies it in the robot's own heading),
	// so only the pose read for aiming and the push path differ per side/context.
	//
	// Aiming maths: the field is drawn with the viewer's axis remap + a Y-flip + a 90°
	// rotation. Composing those, a physics point (px = goal axis, py = lateral, metres) lands
	// at viewBox (vbW/2 + px·1000, vbH/2 + py·1000). We invert the cursor's client position to
	// a field point and aim the robot at it; the robot's on-screen front angle equals its
	// physics heading, so a simple P-controller on the heading error drives ω.
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import type { SimFrame } from '$lib/state/simulation.svelte.js';
	import { Button } from '$lib/components/ui/button';
	import { Crosshair, Bot, User, Zap } from '@lucide/svelte';

	let {
		fieldEl,
		side = 'b',
		context = 'admin'
	}: { fieldEl: HTMLElement | null; side?: 'a' | 'b'; context?: 'admin' | 'game' } = $props();

	// The frame to read poses from: the game's match in game context, else the followed stream.
	const frame = $derived<SimFrame | null>(
		context === 'game' ? simulation.gameFrame : simulation.frame
	);
	// Pose of the robot we're driving (side 'a' = robot_pos/heading; side 'b' = robot2_*).
	const selfPos = $derived<[number, number] | undefined>(
		side === 'a' ? frame?.robot_pos : frame?.robot2_pos
	);
	const selfHeading = $derived<number | undefined>(
		side === 'a' ? frame?.robot_heading : frame?.robot2_heading
	);

	const SEND_HZ = 30;
	// Physics scales action[2] to ±MAX_OMEGA rad/s (keep in sync with python_backend.MAX_OMEGA).
	// The robot's real top turn rate (~58 rad/s) is far too fast to aim by hand, so the human
	// controller targets a comfortable rate and normalizes it down — physics fidelity for the AI,
	// playable sensitivity for the human.
	const MAX_OMEGA = 58.18; // rad/s, physical max (mirrors backend)
	const MAX_LINEAR = 5.236; // m/s, physical max (mirrors backend)
	// Aim controller. Kept gentle on purpose: heading feedback arrives over the network a few
	// frames stale, so a fast turn overshoots, over-corrects, and can wrap past 180° into a
	// continuous spin. A low gain + low cap + a deadzone keep it stable and let it settle.
	const AIM_KP = 4.0; // rad/s of turn per rad of heading error
	const AIM_TURN_CAP = 2.5; // rad/s — gentle enough to stay stable over the control-loop latency
	const AIM_DEADZONE = 0.05; // rad (~3°): within this of the cursor, stop turning
	// Comfortable human drive speed; at full MAX_LINEAR the robot crosses the field in ~0.35 s.
	const HUMAN_LINEAR_CAP = 1.5; // m/s
	const DRIVE_SCALE = HUMAN_LINEAR_CAP / MAX_LINEAR; // normalized drive command ceiling
	const SCALE = 1000; // mm per metre, matching SoccerField's 1 unit = 1 mm

	let selfMode = $state<'human' | 'ai'>('human');

	// Pressed-key / pointer state (not reactive — read inside the send loop).
	const keys = new Set<string>();
	let pointerActive = false;
	let pointerX = 0;
	let pointerY = 0;
	let kicking = false; // momentary: mouse/touch held or Space pressed

	// admin context relays through the per-run sink (needs a manual match running); game
	// context relays through the game code + side token. `active` gates the send loop.
	const run = $derived(simulation.manualMatchRun);
	const active = $derived(
		context === 'game' ? !!simulation.game && !simulation.game.spectator : !!run
	);
	const kickReady = $derived(
		((side === 'a' ? frame?.kick_ready_a : frame?.kick_ready_b) ?? 1) >= 0.5
	);

	/** Relay a control command through the right channel for this context. */
	function send(payload: { action?: number[]; mode: 'human' | 'ai' }) {
		if (context === 'game') {
			void simulation.pushGameControl({ action: payload.action, mode: payload.mode });
		} else if (run) {
			void simulation.pushControl(run, { action: payload.action, redMode: payload.mode });
		}
	}

	function clamp(v: number, lo: number, hi: number) {
		return Math.max(lo, Math.min(hi, v));
	}
	function wrapPi(a: number) {
		while (a > Math.PI) a -= 2 * Math.PI;
		while (a < -Math.PI) a += 2 * Math.PI;
		return a;
	}

	/** Desired heading (physics radians) so red's (robot B's) front points at the cursor, or
	 *  null if the cursor isn't over the field or the geometry isn't available yet. */
	function aimHeading(): number | null {
		if (!pointerActive || !fieldEl) return null;
		const svg = fieldEl.querySelector('svg');
		if (!svg || !selfPos) return null;
		const vb = (svg as SVGSVGElement).viewBox.baseVal;
		if (!vb || !vb.width || !vb.height) return null;
		const rect = svg.getBoundingClientRect();
		// preserveAspectRatio="xMidYMid meet": uniform scale, letterboxed.
		const scale = Math.min(rect.width / vb.width, rect.height / vb.height);
		if (scale <= 0) return null;
		const offX = (rect.width - vb.width * scale) / 2;
		const offY = (rect.height - vb.height * scale) / 2;
		const vbx = (pointerX - rect.left - offX) / scale;
		const vby = (pointerY - rect.top - offY) / scale;
		// viewBox → physics (inverse of px·1000 + centre).
		const px = (vbx - vb.width / 2) / SCALE;
		const py = (vby - vb.height / 2) / SCALE;
		const rx = selfPos[0];
		const ry = selfPos[1];
		return Math.atan2(py - ry, px - rx);
	}

	function computeAction(): number[] {
		// Scaled to a comfortable speed: physics multiplies these back up by MAX_LINEAR.
		const fwd = ((keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0)) * DRIVE_SCALE;
		// Body-frame +vy is to the robot's left, so A (left) → +vy, D (right) → −vy.
		const strafe = ((keys.has('a') ? 1 : 0) - (keys.has('d') ? 1 : 0)) * DRIVE_SCALE;

		let omega = 0;
		const target = aimHeading();
		if (target != null && selfHeading != null) {
			// P-controller on heading error → a comfortable turn rate, then normalize to the
			// action's [-1, 1] range (physics multiplies it back up by MAX_OMEGA). A deadzone
			// near zero error stops it hunting/spinning once it's pointing at the cursor.
			const err = wrapPi(target - selfHeading);
			const desiredRad =
				Math.abs(err) < AIM_DEADZONE ? 0 : clamp(err * AIM_KP, -AIM_TURN_CAP, AIM_TURN_CAP);
			omega = desiredRad / MAX_OMEGA;
		} else {
			// Fallback when not aiming: Q/E rotate left/right at the same comfortable rate.
			omega = ((keys.has('q') ? 1 : 0) - (keys.has('e') ? 1 : 0)) * (AIM_TURN_CAP / MAX_OMEGA);
		}

		const kick = kicking || keys.has(' ') ? 1 : 0;
		return [fwd, strafe, omega, kick];
	}

	function toggleMode() {
		selfMode = selfMode === 'human' ? 'ai' : 'human';
		if (active) send({ mode: selfMode });
	}

	// ── input wiring ──────────────────────────────────────────────────────────
	function onKeyDown(e: KeyboardEvent) {
		const k = e.key.toLowerCase();
		if (['w', 'a', 's', 'd', 'q', 'e', ' '].includes(k)) {
			keys.add(k);
			e.preventDefault();
		}
	}
	function onKeyUp(e: KeyboardEvent) {
		keys.delete(e.key.toLowerCase());
	}
	function onPointerMove(e: PointerEvent) {
		pointerActive = true;
		pointerX = e.clientX;
		pointerY = e.clientY;
	}
	function onPointerLeave() {
		pointerActive = false;
	}
	function onPointerDown(e: PointerEvent) {
		pointerActive = true;
		pointerX = e.clientX;
		pointerY = e.clientY;
		kicking = true;
	}
	function onPointerUp() {
		kicking = false;
	}

	// Touch d-pad / kick buttons: press-and-hold sets the same key/kick flags.
	function holdKey(k: string, down: boolean) {
		if (down) keys.add(k);
		else keys.delete(k);
	}

	onMount(() => {
		window.addEventListener('keydown', onKeyDown);
		window.addEventListener('keyup', onKeyUp);
		window.addEventListener('pointerup', onPointerUp);
		const target = fieldEl;
		target?.addEventListener('pointermove', onPointerMove);
		target?.addEventListener('pointerdown', onPointerDown);
		target?.addEventListener('pointerleave', onPointerLeave);

		const timer = setInterval(() => {
			if (!active) return;
			send({ action: computeAction(), mode: selfMode });
		}, 1000 / SEND_HZ);

		return () => {
			clearInterval(timer);
			window.removeEventListener('keydown', onKeyDown);
			window.removeEventListener('keyup', onKeyUp);
			window.removeEventListener('pointerup', onPointerUp);
			target?.removeEventListener('pointermove', onPointerMove);
			target?.removeEventListener('pointerdown', onPointerDown);
			target?.removeEventListener('pointerleave', onPointerLeave);
		};
	});
</script>

<div class="flex flex-col gap-2 rounded-lg border border-border bg-background/60 p-3">
	<div class="flex items-center justify-between gap-2">
		<span class="flex items-center gap-1.5 font-mono text-[11px] uppercase tracking-wider text-muted-foreground">
			<Crosshair class="size-3.5" />{context === 'game'
				? `Drive ${side === 'a' ? 'blue' : 'red'}`
				: 'Manual red (test)'}
		</span>
		<span
			class="flex items-center gap-1 rounded-full border px-2 py-0.5 font-mono text-[10px] uppercase tracking-wider"
			style="border-color: {kickReady ? '#34d399' : '#64748b'}; color: {kickReady ? '#34d399' : '#94a3b8'}"
			title="Kicker {kickReady ? 'charged' : 'recharging'}"
		>
			<Zap class="size-3" />{kickReady ? 'Kick ready' : 'Charging'}
		</span>
	</div>

	{#if selfMode === 'human'}
		<p class="font-mono text-[10px] leading-relaxed text-muted-foreground">
			Mouse over the field aims · <span class="text-foreground">W/S</span> drive ·
			<span class="text-foreground">A/D</span> strafe ·
			<span class="text-foreground">Space</span> / click kicks
		</p>
	{:else}
		<p class="font-mono text-[10px] leading-relaxed text-muted-foreground">
			Your robot is running its AI policy. Switch back to drive it yourself.
		</p>
	{/if}

	<!-- Human / AI toggle -->
	<Button variant="outline" size="sm" class="font-mono text-[11px]" onclick={toggleMode}>
		{#if selfMode === 'human'}
			<Bot class="size-3.5" />Hand to AI
		{:else}
			<User class="size-3.5" />Take control
		{/if}
	</Button>

	<!-- On-screen touch controls (also usable with a mouse) -->
	{#if selfMode === 'human'}
		<div class="mt-1 flex items-center justify-between gap-3">
			<div class="grid grid-cols-3 grid-rows-2 gap-1">
				{#each [['w', '↑', 'col-start-2'], ['a', '←', 'col-start-1 row-start-2'], ['s', '↓', 'col-start-2 row-start-2'], ['d', '→', 'col-start-3 row-start-2']] as [k, label, cls]}
					<button
						class="flex size-9 items-center justify-center rounded-md border border-border bg-input/30 font-mono text-sm text-foreground active:bg-primary/40 {cls}"
						onpointerdown={(e) => { e.preventDefault(); holdKey(k, true); }}
						onpointerup={() => holdKey(k, false)}
						onpointerleave={() => holdKey(k, false)}
					>{label}</button>
				{/each}
			</div>
			<button
				class="flex size-16 items-center justify-center rounded-full border-2 font-mono text-[11px] font-bold uppercase tracking-wider text-foreground active:scale-95"
				style="border-color: {kickReady ? '#34d399' : '#64748b'}"
				onpointerdown={(e) => { e.preventDefault(); kicking = true; }}
				onpointerup={() => (kicking = false)}
				onpointerleave={() => (kicking = false)}
			>Kick</button>
		</div>
	{/if}
</div>
