<script lang="ts">
	import { Button } from '$lib/components/ui/button';

	// ── Public props ─────────────────────────────────────────────────────────
	// Coordinates use the same mm-unit field space as the field constants below.
	// theta: heading angle in radians — π/2 = facing toward enemy goal (+Y).
	// If allies/enemies are omitted the mode toggle controls default positions.
	// If allies is supplied the toggle is hidden and mode is derived from its length.
	// All props are optional; defaults reproduce the original rendering.
	type Pose = { x: number; y: number; theta: number };
	let {
		ball: ballProp,
		allies: alliesProp,
		enemies: enemiesProp,
		// Whole-diagram rotation, in degrees (clockwise on screen).
		rotation = 0,
		// ── Visibility toggles ───────────────────────────────────────────────
		showGoalLabels = true,
		showScaleBar = true,
		showCenterCircle = true,
		showHeadingArrows = true,
		showRobotLabels = true,
		showBall = true,
		showPenaltyAreas = true,
		// ── Colors ───────────────────────────────────────────────────────────
		wallColor = '#1a1a1a',
		neutralColor = '#8c8c8c',
		grassColor = '#1e6b1e',
		lineColor = 'white',
		penaltyColor = '#111',
		ballColor = '#ffa000',
		ownGoalColor = '#5a8cff',
		enemyGoalColor = '#ffc800',
		allyColor,
		enemyColor,
		// ── Dimensions (mm) ──────────────────────────────────────────────────
		robotRadius = 90,
		ballRadius = 21,
		fieldHalfWidth = 610,
		fieldHalfHeight = 915,
		neutralZone = 300,
		wallMargin = 300,
		goalHalfWidth = 225,
		penaltyDepth = 450,
		// ── Wrapper passthrough ──────────────────────────────────────────────
		class: className = '',
		style,
	}: {
		ball?: { x: number; y: number };
		allies?: Pose[];
		enemies?: Pose[];
		rotation?: number;
		showGoalLabels?: boolean;
		showScaleBar?: boolean;
		showCenterCircle?: boolean;
		showHeadingArrows?: boolean;
		showRobotLabels?: boolean;
		showBall?: boolean;
		showPenaltyAreas?: boolean;
		wallColor?: string;
		neutralColor?: string;
		grassColor?: string;
		lineColor?: string;
		penaltyColor?: string;
		ballColor?: string;
		ownGoalColor?: string;
		enemyGoalColor?: string;
		allyColor?: string;
		enemyColor?: string;
		robotRadius?: number;
		ballRadius?: number;
		fieldHalfWidth?: number;
		fieldHalfHeight?: number;
		neutralZone?: number;
		wallMargin?: number;
		goalHalfWidth?: number;
		penaltyDepth?: number;
		class?: string;
		style?: string;
	} = $props();

	// ── Internal mode state ──────────────────────────────────────────────────
	let mode = $state<'1v1' | '2v2'>('1v1');
	let effectiveMode = $derived<'1v1' | '2v2'>(
			alliesProp != null ? (alliesProp.length >= 2 ? '2v2' : '1v1') : mode
	);
	let showToggle = $derived(alliesProp == null);

	// ── Field geometry (mm) ──────────────────────────────────────────────────
	// Short internal aliases of the dimension props, so the markup below stays
	// unchanged. Computed values are reactive so prop overrides flow through.
	let R         = $derived(robotRadius);
	let BALL_R    = $derived(ballRadius);
	const GAP_HW  = 25;
	const GAP_D   = 35;
	let ARROW     = $derived(R * 1.5);

	let FW2       = $derived(fieldHalfWidth);
	let FH2       = $derived(fieldHalfHeight);
	let NZ        = $derived(neutralZone);
	let WL        = $derived(wallMargin);
	let GOAL_HW   = $derived(goalHalfWidth);

	let CW2 = $derived(FW2 + NZ + WL);  // canvas half-width
	let CH2 = $derived(FH2 + NZ + WL);  // canvas half-height
	let PENALTY_D = $derived(penaltyDepth);  // penalty area depth into field (mm)

	// ── Rotation-aware viewBox ───────────────────────────────────────────────
	// Rotation is applied inside the SVG (not via CSS transform), and the viewBox
	// grows to the rotated bounding box so the SVG's intrinsic width/height swap
	// with the angle — e.g. a 90° turn makes it land/short instead of overflowing.
	let rad     = $derived((rotation * Math.PI) / 180);
	let absCos  = $derived(Math.abs(Math.cos(rad)));
	let absSin  = $derived(Math.abs(Math.sin(rad)));
	let vbW     = $derived(CW2 * 2 * absCos + CH2 * 2 * absSin);
	let vbH     = $derived(CW2 * 2 * absSin + CH2 * 2 * absCos);

	// Arrow polygon: always drawn pointing local +Y (rotation in transform handles direction)
	let ARROW_POINTS = $derived(`0,${ARROW} -20,${ARROW - 32} 20,${ARROW - 32}`);

	// ── Default positions: kickoff base situation (rule 4.4 "Aftrap") ────────
	// Each half begins with a kickoff: the referee places the ball on the centre
	// spot (ballX/ballY default to 0,0). We (allies) are the kicking-off team, so
	// our robot may stand against the ball, just inside our own half. The opponent
	// is the non-kicking team and must stand inside its own penalty area.
	// Index 0 is used in 1v1; indices 0 and 1 in 2v2.
	const ALLY_DEFAULTS = [
		// Kicker: front (dribbler) edge touching the ball → centre y = -(R + BALL_R) = -111.
		{ x:   0, y: -111, theta:  Math.PI / 2, fill: '#3c78dc' },
		// Support: kept back in our own half.
		{ x:   0, y: -700, theta:  Math.PI / 2, fill: '#2558b0' },
	];
	const ENEMY_DEFAULTS = [
		// Non-kicking team: inside their own penalty area
		// (y ∈ [FH2 − PENALTY_D, FH2] = [465, 915], x ∈ [-225, 225]).
		// Index 0 sits centred in the box (1v1 default); the 2v2 partner tucks
		// to one side since two robots can't both be centred without overlapping.
		{ x:   0, y:  690, theta: -Math.PI / 2, fill: '#dc3c3c' },
		{ x: 135, y:  555, theta: -Math.PI / 2, fill: '#a82222' },
	];

	// ── Internal robot type ──────────────────────────────────────────────────
	type Robot = { x: number; y: number; rotDeg: number; fill: string; label?: string };

	// SVG rotation to apply inside the Y-flipped field group so the robot
	// faces the correct direction on screen.
	// Derivation: rotate(θ − π/2) in degrees, where θ is the math heading.
	function toRotDeg(theta: number): number {
		return (theta - Math.PI / 2) * (180 / Math.PI);
	}

	let robots = $derived.by<Robot[]>(() => {
		const allyCount  = alliesProp?.length  ?? (effectiveMode === '2v2' ? 2 : 1);
		const enemyCount = enemiesProp?.length ?? (effectiveMode === '2v2' ? 2 : 1);

		const allies: Robot[] = Array.from({ length: allyCount }, (_, i) => {
			const pose = alliesProp?.[i];
			const def  = ALLY_DEFAULTS[Math.min(i, ALLY_DEFAULTS.length - 1)];
			return {
				x: pose?.x ?? def.x,
				y: pose?.y ?? def.y,
				rotDeg: toRotDeg(pose?.theta ?? def.theta),
				fill: allyColor ?? def.fill,
				label: allyCount > 1 ? String(i + 1) : undefined,
			};
		});

		const enemies: Robot[] = Array.from({ length: enemyCount }, (_, i) => {
			const pose = enemiesProp?.[i];
			const def  = ENEMY_DEFAULTS[Math.min(i, ENEMY_DEFAULTS.length - 1)];
			return {
				x: pose?.x ?? def.x,
				y: pose?.y ?? def.y,
				rotDeg: toRotDeg(pose?.theta ?? def.theta),
				fill: enemyColor ?? def.fill,
				label: enemyCount > 1 ? String(i + 1) : undefined,
			};
		});

		return [...allies, ...enemies];
	});

	let ballX = $derived(ballProp?.x ?? 0);
	let ballY = $derived(ballProp?.y ?? 0);
</script>



<!--
    Coordinate system: viewBox 0 0 {CW2*2} {CH2*2} (1 unit = 1 mm).
    Main group: translate(CW2, CH2) scale(1, -1)
    → local origin = field center, +Y upward on screen = toward enemy goal.
    Robot groups: translate(x,y) rotate(rotDeg) where rotDeg = (θ − π/2)×(180/π).
      This rotates the robot so its drawn local +Y always faces its heading direction.
      Number labels use rotate(−rotDeg) scale(1,−1) to stay readable on screen.
-->
<div class="rounded-lg overflow-hidden border border-border {className}" {style}>
	<!--
        rotation turns the whole diagram. It is applied inside the SVG (see the
        rotor <g> below) and the viewBox grows to the rotated bounding box, so the
        SVG's intrinsic width/height swap with the angle instead of overflowing.
    -->
	<svg
			viewBox="0 0 {vbW} {vbH}"
			class="w-full h-auto block"
			role="img"
			aria-label="RoboCup Junior 2026 soccer field reference diagram"
	>
		<defs>
			<!--
                Shared clip path for the dribbler gap (circle at local origin r=90).
                clipPathUnits="userSpaceOnUse" applies this in each robot's own
                coordinate space, so every robot gets a circle at its own center.
            -->
			<clipPath id="rcg">
				<circle cx="0" cy="0" r={R} />
			</clipPath>
		</defs>

		<!--
            Rotor: rotate everything about the original canvas center (CW2, CH2),
            then re-center it in the rotated viewBox (vbW, vbH). Content below keeps
            using the original coordinate space.
        -->
		<g transform="translate({vbW / 2}, {vbH / 2}) rotate({rotation}) translate({-CW2}, {-CH2})">

		<!-- Goal labels in root SVG coords (not in the Y-flipped group) -->
		{#if showGoalLabels}
			<text
					x={CW2} y="150"
					text-anchor="middle"
					fill={enemyGoalColor}
					font-size="50"
					font-family="ui-monospace, monospace"
					font-weight="600"
					letter-spacing="3"
			>ENEMY GOAL</text>
			<text
					x={CW2} y={CH2 * 2 - 90}
					text-anchor="middle"
					fill={ownGoalColor}
					font-size="50"
					font-family="ui-monospace, monospace"
					font-weight="600"
					letter-spacing="3"
			>OWN GOAL</text>
		{/if}

		<!-- Scale bar: 500 units = 50 cm -->
		{#if showScaleBar}
		<g transform="translate({CW2 * 2 - 640}, {CH2 * 2 - 90})">
			<line x1="0"   y1="-10" x2="0"   y2="10" stroke="#aaa" stroke-width="4" />
			<line x1="0"   y1="0"   x2="500" y2="0"  stroke="#aaa" stroke-width="3" />
			<line x1="500" y1="-10" x2="500" y2="10" stroke="#aaa" stroke-width="4" />
			<text
					x="250" y="-22"
					text-anchor="middle"
					fill="#aaa"
					font-size="34"
					font-family="ui-monospace, monospace"
			>50 cm</text>
		</g>
		{/if}

		<!-- Main field group: +Y is upward on screen -->
		<g transform="translate({CW2}, {CH2}) scale(1, -1)">

			<!-- Outer wall -->
			<rect x={-CW2} y={-CH2} width={CW2 * 2} height={CH2 * 2} fill={wallColor} />

			<!-- Neutral zone -->
			<rect
					x={-(FW2 + NZ)} y={-(FH2 + NZ)}
					width={(FW2 + NZ) * 2} height={(FH2 + NZ) * 2}
					fill={neutralColor}
			/>

			<!-- Playing field surface -->
			<rect
					x={-FW2} y={-FH2}
					width={FW2 * 2} height={FH2 * 2}
					fill={grassColor}
					stroke={lineColor}
					stroke-width="12"
			/>

			<!--
                Penalty areas (strafschopgebied): 450 mm deep from each goal line,
                same width as the goal opening, colored black (RAL 9005).
                Drawn before other markings so lines/goals render on top.
            -->
			{#if showPenaltyAreas}
			<!-- Own goal penalty area (at bottom, y = -FH2 going into field) -->
			<rect
					x={-GOAL_HW} y={-FH2}
					width={GOAL_HW * 2} height={PENALTY_D}
					fill={penaltyColor}
			/>
			<!-- Enemy goal penalty area (at top, y = FH2 going into field) -->
			<rect
					x={-GOAL_HW} y={FH2 - PENALTY_D}
					width={GOAL_HW * 2} height={PENALTY_D}
					fill={penaltyColor}
			/>
			{/if}

			<!-- Center line -->
			<line x1={-FW2} y1="0" x2={FW2} y2="0" stroke={lineColor} stroke-width="9" />

			<!-- Center circle -->
			{#if showCenterCircle}
			<circle cx="0" cy="0" r="300" fill="none" stroke={lineColor} stroke-width="9" />
			<circle cx="0" cy="0" r="15" fill={lineColor} />
			{/if}

			<!-- Own goal (blue, extends into neutral zone below field) -->
			<rect
					x={-GOAL_HW} y={-(FH2 + NZ)}
					width={GOAL_HW * 2} height={NZ}
					fill={ownGoalColor}
					fill-opacity="0.28"
					stroke={ownGoalColor}
					stroke-width="10"
			/>
			<line
					x1={-GOAL_HW} y1={-(FH2 + NZ)}
					x2={GOAL_HW}  y2={-(FH2 + NZ)}
					stroke={ownGoalColor} stroke-width="6" stroke-dasharray="20 12"
			/>

			<!-- Enemy goal (gold, extends into neutral zone above field) -->
			<rect
					x={-GOAL_HW} y={FH2}
					width={GOAL_HW * 2} height={NZ}
					fill={enemyGoalColor}
					fill-opacity="0.28"
					stroke={enemyGoalColor}
					stroke-width="10"
			/>
			<line
					x1={-GOAL_HW} y1={FH2 + NZ}
					x2={GOAL_HW}  y2={FH2 + NZ}
					stroke={enemyGoalColor} stroke-width="6" stroke-dasharray="20 12"
			/>

			<!-- Ball -->
			{#if showBall}
			<circle
					cx={ballX} cy={ballY} r={BALL_R}
					fill={ballColor}
					stroke="rgba(255,255,255,0.75)"
					stroke-width="6"
			/>
			{/if}

			<!-- Robots -->
			{#each robots as robot}
				<!--
                    rotate(rotDeg) orients the robot so its local +Y faces the heading.
                    Gap and arrow are always drawn pointing local +Y (no dir check needed).
                -->
				<g transform="translate({robot.x}, {robot.y}) rotate({robot.rotDeg})">

					<!-- Drop shadow -->
					<circle r={R + 6} fill="rgba(0,0,0,0.25)" cx="4" cy="-4" />

					<!-- Robot body -->
					<circle r={R} fill={robot.fill} stroke="white" stroke-width="7" />

					<!-- Dribbler gap: dark notch at local +Y (front) edge, clipped to circle -->
					<rect
							x={-GAP_HW} y={R - GAP_D}
							width={GAP_HW * 2} height={GAP_D}
							fill="#111"
							clip-path="url(#rcg)"
					/>

					<!-- Heading arrow -->
					{#if showHeadingArrows}
						<line x1="0" y1={R + 4} x2="0" y2={ARROW} stroke="white" stroke-width="5" stroke-linecap="round" />
						<polygon points={ARROW_POINTS} fill="white" />
					{/if}

					<!--
                        Number label (2v2 only).
                        rotate(-rotDeg) undoes the robot's heading rotation.
                        scale(1,-1) undoes the field group's Y-flip.
                        Net: text renders horizontally right-side-up regardless of heading.
                    -->
					{#if showRobotLabels && robot.label}
						<g transform="rotate({-robot.rotDeg}) scale(1, -1)">
							<text
									dy="0.38em"
									text-anchor="middle"
									font-size="72"
									fill="white"
									font-weight="bold"
									font-family="ui-monospace, monospace"
									paint-order="stroke"
									stroke="rgba(0,0,0,0.35)"
									stroke-width="10"
									stroke-linejoin="round"
							>{robot.label}</text>
						</g>
					{/if}
				</g>
			{/each}

		</g>
		</g>
	</svg>
</div>


