"""Pure NumPy 2D rigid-body physics for Bucky.

Assumptions:
- Robot radius: 0.11 m (≈22 cm diameter)
- Ball radius: 0.021 m (RCJ-05 ball, modelled at 42 mm)
- Robot max speed: 1.0 m/s linear, 6.0 rad/s angular
- Timestep: 0.02 s (50 Hz)
- Field: official RoboCup Junior 1.83 m × 1.22 m playfield, goals on ±x wall, 0.45 m wide
- Robot drives commands applied with first-order lag (τ=0.05 s motor lag)
- Ball: point-mass with damping (μ=0.3), elastic wall bounces, soft collision with robot

Field geometry lives in :mod:`bucky.field` (single source of truth, official NK dimensions).
``FIELD_*`` is the white-line marked field — the out-of-bounds reference. The physical
arena walls sit a uniform 300 mm band further out, so the robot and ball can drive into
the "out" band and the ball can get pinned in the corners.
"""
from __future__ import annotations

import numpy as np

from bucky.game.field import (
    ARENA_HALF_X,
    ARENA_HALF_Y,
    BALL_RADIUS,
    COLLISION_DIST,
    FIELD_H,
    FIELD_W,
    GOAL_HALF_WIDTH,
    GOAL_WIDTH,
    HALF_W,
    PENALTY_DEPTH,
    ROBOT_RADIUS,
    ball_out_of_play,
    is_goal,
)
from bucky.physics.backend import PhysicsBackend, PhysicsState

# Drivetrain limits derived from the real robot (Pololu 4842: 9.7:1, 1000 rpm no-load output;
# wheel radius 0.05 m; wheel mounting radius d = 0.09 m):
#   ω_wheel = 1000/60 · 2π   = 104.72 rad/s
#   v_rim   = ω_wheel · 0.05 = 5.236 m/s   → MAX_LINEAR (straight-line top speed)
#   ω_robot = v_rim / 0.09   ≈ 58.18 rad/s → MAX_OMEGA  (pure in-place rotation)
# Action dims [0:3] are normalized [-1, 1]; the physics scales them to ±MAX_LINEAR / ±MAX_OMEGA
# (see _drive / PyPhysics.step). These are no-load ceilings — see also rewards.MAX_OMEGA_PENALTY
# and obs._MAX_VEL / obs._MAX_OMEGA, which track these so observations don't saturate.
MAX_LINEAR = 5.236  # m/s
MAX_OMEGA = 58.18   # rad/s
DT = 0.02          # s
MOTOR_TAU = 0.05   # first-order lag time constant (s)
BALL_DAMPING = 0.3
BALL_RESTITUTION = 0.7

# ── Kicker (rule 3.8.1) ──────────────────────────────────────────────────────
# A kicker may launch an RCJ-05 ball onto a 20° ramp 220 mm along the hypotenuse, and the
# ball may not pass the top of that ramp. Reading this as a 20° projectile whose peak height
# equals the ramp top fixes the legal max ball speed:
#   h     = 0.220 * sin(20°)            = 0.0752 m   (ramp-top height)
#   v_y   = sqrt(2 * 9.81 * h)          = 1.215 m/s  (vertical launch speed to reach h)
#   v_max = v_y / sin(20°)              ≈ 3.55 m/s   (total launch speed at 20°)
# The 2-D sim has no vertical axis, so v_max is applied as the max horizontal speed the
# kicker can impart to the ball. Ball mass (0.045 kg) / g (9.81) only enter this offline
# derivation, never the sim.
KICK_MAX_SPEED = 3.55          # m/s, legal upper limit (selectable force scales 0..1 of this)
KICK_RANGE = 0.16              # ball must be within this of the robot centre (≈ COLLISION_DIST + margin)
KICK_FRONT_COS = 0.766         # cos(40°): ball must lie within a ±40° forward cone to be kicked
KICK_COOLDOWN_STEPS = 50       # 1.0 s recharge at 50 Hz
KICK_DEADZONE = 0.05           # kick command below this fires nothing


def apply_kick(ball_pos, ball_vel, r_pos, r_heading, kick_cmd, cooldown):
    """Apply a kicker impulse to the ball along the robot heading, subject to gating.

    Fires only when the kicker has recharged (``cooldown == 0``), the ball is within
    ``KICK_RANGE`` and inside a ±40° forward cone, and ``kick_cmd`` clears the deadzone.
    The selectable force ``clip(kick_cmd, 0, 1)`` scales the imparted speed up to
    ``KICK_MAX_SPEED``.

    Returns ``(ball_vel, new_cooldown, kicked, kick_speed)``.
    """
    if cooldown > 0:
        return ball_vel, cooldown - 1, False, 0.0
    force = float(np.clip(kick_cmd, 0.0, 1.0))
    if force < KICK_DEADZONE:
        return ball_vel, 0, False, 0.0
    delta = ball_pos - r_pos
    dist = float(np.linalg.norm(delta))
    if dist > KICK_RANGE or dist < 1e-6:
        return ball_vel, 0, False, 0.0
    heading_vec = np.array([np.cos(r_heading), np.sin(r_heading)])
    if float(np.dot(heading_vec, delta / dist)) < KICK_FRONT_COS:
        return ball_vel, 0, False, 0.0
    kick_speed = force * KICK_MAX_SPEED
    return ball_vel + heading_vec * kick_speed, KICK_COOLDOWN_STEPS, True, kick_speed


def _crosses_goal(prev_x: float, prev_y: float, x: float, y: float, sign: int) -> bool:
    """True if the ball segment (prev → current) crosses the ``sign`` goal line through the mouth.

    The mouth test interpolates y at the exact crossing of ``x = sign·HALF_W`` rather than reading
    the end-of-step y: a fast/diagonal shot that passes through the mouth but ends the step with
    |y| just past the mouth edge still scores (it was inside the opening when it crossed the line).
    Requiring a crossing *from the field side* still rejects a "goal from the side" — a ball that
    slipped laterally into the strip past a side wall never crosses the line from the field.
    """
    line = sign * (FIELD_W / 2)
    if sign > 0:
        if not (prev_x <= line < x):
            return False
    else:
        if not (prev_x >= line > x):
            return False
    t = (line - prev_x) / (x - prev_x)          # x != prev_x is guaranteed by the crossing test
    y_cross = prev_y + t * (y - prev_y)
    return abs(y_cross) < GOAL_WIDTH / 2


def _goal_scored(prev_x: float, prev_y: float, x: float, y: float, sign: int) -> bool:
    """True if the ball scored in the ``sign`` goal (+1 → +x, -1 → -x).

    Combines two robust tests: the ball now *occupies* the goal box (``field.is_goal`` — catches a
    ball that ends a step behind the line in the mouth, including slow multi-step or diagonal
    entries the single-step crossing test alone would miss), OR it *crossed* the goal line through
    the mouth this step (``_crosses_goal`` — the exact-crossing belt-and-braces). Because the goal
    posts, side walls and back wall are all solid for the ball, the box is only reachable through the
    mouth, so occupancy can never be a "goal from the side".
    """
    plus_x, minus_x = is_goal((x, y))
    occupied = plus_x if sign > 0 else minus_x
    return bool(occupied) or _crosses_goal(prev_x, prev_y, x, y, sign)


# ── Goal box (solid for the robot) ───────────────────────────────────────────
# The goal is a box recessed behind each goal line. Its mouth (the GOAL_WIDTH opening at
# x = ±HALF_W between the posts) is open so a robot may poke in, but the two side walls running
# back from the posts are SOLID — the robot cannot cross them or drive through the goal into the
# rear band (the back wall coincides with the arena clamp in x). Modelled as circle-vs-segment
# push-out against each side wall; each segment's field-side endpoint acts as the goalpost.
_GOAL_SIDE_WALLS = tuple(
    (np.array([s * HALF_W, sy * GOAL_HALF_WIDTH]),
     np.array([s * ARENA_HALF_X, sy * GOAL_HALF_WIDTH]))
    for s in (1.0, -1.0)
    for sy in (1.0, -1.0)
)


def _circle_segment_pushout(pos, a, b, radius):
    """Push the circle ``(pos, radius)`` out of the thin solid wall segment ``a→b``.

    The wall has no preferred side: the circle is pushed to whichever side its centre already
    lies on, so a robot can sit on either face but never cross. Endpoints handle the post corners.
    """
    ab = b - a
    denom = float(np.dot(ab, ab))
    t = 0.0 if denom < 1e-12 else float(np.clip(np.dot(pos - a, ab) / denom, 0.0, 1.0))
    closest = a + t * ab
    d = pos - closest
    dist = float(np.linalg.norm(d))
    if 1e-9 < dist < radius:
        return closest + d * (radius / dist)
    return pos


def _resolve_robot_goal(pos):
    """Keep a robot out of both goal boxes (push it off the solid goal side walls/posts)."""
    out = pos
    for a, b in _GOAL_SIDE_WALLS:
        out = _circle_segment_pushout(out, a, b, ROBOT_RADIUS)
    return out


def _bounce_ball_goal_walls(ball_pos, ball_vel, prev_x: float, prev_y: float):
    """Bounce the ball off the solid goal side walls, returning new ``(pos, vel, bounced)``.

    The side walls are the horizontal segments at y = ±GOAL_HALF_WIDTH running from each goalpost
    (x = ±HALF_W) back to the arena wall (x = ±ARENA_HALF_X). The test is swept in y using the
    start-of-step ``prev_y`` so a fast ball crossing the thin wall in one step (its per-step travel
    can exceed the wall thickness) still bounces instead of tunnelling through. The ball is pushed
    back to whichever side it came from and its y-velocity is reflected with restitution.
    """
    bounced = False
    for a, b in _GOAL_SIDE_WALLS:
        wall_y = float(a[1])                       # both endpoints share y (= ±GOAL_HALF_WIDTH)
        x_lo, x_hi = sorted((float(a[0]), float(b[0])))
        if not (x_lo - BALL_RADIUS <= ball_pos[0] <= x_hi + BALL_RADIUS):
            continue
        crossed = (prev_y - wall_y) * (ball_pos[1] - wall_y) < 0.0   # straddled the wall (swept)
        near = abs(ball_pos[1] - wall_y) < BALL_RADIUS               # resting against it
        if not (crossed or near):
            continue
        sgn = 1.0 if prev_y >= wall_y else -1.0                      # side the ball came from
        ball_pos[1] = wall_y + sgn * BALL_RADIUS
        if ball_vel[1] * sgn < 0.0:                                  # moving into the wall
            ball_vel[1] = -ball_vel[1] * BALL_RESTITUTION
        bounced = True
    return ball_pos, ball_vel, bounced


def _resolve_ball_walls_pure(ball_pos, ball_vel, prev_x: float, prev_y: float):
    """Pure ball↔wall resolution (arena walls + goal side walls), returning new (pos, vel, bounced).

    Single source of truth shared by :meth:`PyPhysics._resolve_ball_walls` (in-place wrapper) and
    :func:`predict_goal_by_rollout` (look-ahead), so the prediction matches the real physics.
    ``prev_x``/``prev_y`` are the ball's start-of-step position (makes the goal side-wall test swept).
    """
    ball_pos = np.array(ball_pos, dtype=float)
    ball_vel = np.array(ball_vel, dtype=float)
    bounced = False
    hx, hy = ARENA_HALF_X - BALL_RADIUS, ARENA_HALF_Y - BALL_RADIUS
    # Arena back wall (x). Solid everywhere — including behind the goal mouth, where it is the
    # recessed back of the net. The goal *line* at x = ±HALF_W is not a wall (nothing clamps
    # there), so the ball still passes freely through the mouth; only the back of the goal stops it.
    if ball_pos[0] < -hx:
        ball_pos[0] = -hx
        ball_vel[0] = abs(ball_vel[0]) * BALL_RESTITUTION
        bounced = True
    elif ball_pos[0] > hx:
        ball_pos[0] = hx
        ball_vel[0] = -abs(ball_vel[0]) * BALL_RESTITUTION
        bounced = True
    if abs(ball_pos[1]) > hy:
        ball_vel[1] *= -BALL_RESTITUTION
        ball_pos[1] = np.sign(ball_pos[1]) * hy
        bounced = True
    # Solid goal side walls (the horizontal walls at y = ±GOAL_HALF_WIDTH running from each
    # goalpost back to the arena wall). Bouncing the ball off them means the goal box can only be
    # entered through the mouth opening, so a ball never slips beside a post into the strip and an
    # occupancy-based goal test can't be fooled by a "goal from the side". Swept in y (via prev_y)
    # so a fast ball can't tunnel through the thin wall in a single step.
    ball_pos, ball_vel, hit = _bounce_ball_goal_walls(ball_pos, ball_vel, prev_x, prev_y)
    bounced = bounced or hit
    return ball_pos, ball_vel, bounced


def predict_goal_by_rollout(ball_pos, ball_vel, attack_sign: int = 1, dt: float = DT,
                            horizon_steps: int = 75, opponent_pos=None,
                            speed_floor: float = 0.2):
    """Roll a *ball-only* copy forward to decide whether the current shot will score.

    Mirrors the ball half of :meth:`PyPhysics.step` (damping + wall/goal-wall bounces via
    :func:`_resolve_ball_walls_pure` + the :func:`_crosses_goal` mouth test). Robots are not
    simulated; if ``opponent_pos`` is given, the ball passing within ``COLLISION_DIST`` of it marks
    the prediction *intercepted* (caller treats that as "not confident"). Early-exits the instant
    the outcome is known. ``attack_sign`` is the goal the shooter attacks (+1 → +x, -1 → -x).

    Returns ``(scored, steps_to_goal, intercepted)``.
    """
    pos = np.array(ball_pos, dtype=float)
    vel = np.array(ball_vel, dtype=float)
    opp = None if opponent_pos is None else np.asarray(opponent_pos, dtype=float)
    intercepted = False
    for i in range(1, int(horizon_steps) + 1):
        prev_x, prev_y = float(pos[0]), float(pos[1])
        vel = vel * (1.0 - BALL_DAMPING * dt)
        pos = pos + vel * dt
        pos, vel, _ = _resolve_ball_walls_pure(pos, vel, prev_x, prev_y)
        if opp is not None and float(np.linalg.norm(pos - opp)) < COLLISION_DIST:
            intercepted = True
        if _goal_scored(prev_x, prev_y, float(pos[0]), float(pos[1]), attack_sign):
            return True, i, intercepted
        if ball_out_of_play(pos):
            return False, i, intercepted
        if float(np.linalg.norm(vel)) < speed_floor:
            return False, i, intercepted
    return False, int(horizon_steps), intercepted


def omni_kinematics(vx: float, vy: float, omega: float) -> np.ndarray:
    """Convert body-frame velocity to 3-wheel speeds.

    Wheels at 0°, 120°, 240° from robot +x axis.
    Assumption: wheel radius 0.04 m, robot_radius_to_wheel 0.09 m.
    Returns wheel angular velocities (rad/s).
    """
    r_wheel = 0.04
    d = 0.09
    angles = np.array([0.0, 2 * np.pi / 3, 4 * np.pi / 3])
    wheel_vels = np.array([
        -np.sin(a) * vx + np.cos(a) * vy + d * omega
        for a in angles
    ])
    return wheel_vels / r_wheel


class PyPhysics(PhysicsBackend):
    """Simple 2D omni-drive + ball physics."""

    def __init__(self, dt: float = DT) -> None:
        self._dt = dt
        self._rng = np.random.default_rng()
        self._robot_pos = np.zeros(2)
        self._robot_vel = np.zeros(2)
        self._robot_heading = 0.0
        self._robot_omega = 0.0
        self._robot_vel_cmd = np.zeros(2)
        self._omega_cmd = 0.0
        self._ball_pos = np.zeros(2)
        self._ball_vel = np.zeros(2)
        self._kick_cooldown = 0

    @property
    def dt(self) -> float:
        return self._dt

    def reset(self, seed: int | None = None) -> PhysicsState:
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._robot_pos = np.array([0.3, 0.0]) + self._rng.uniform(-0.1, 0.1, 2)
        self._robot_vel = np.zeros(2)
        self._robot_heading = self._rng.uniform(-np.pi, np.pi)
        self._robot_omega = 0.0
        self._robot_vel_cmd = np.zeros(2)
        self._omega_cmd = 0.0
        self._ball_pos = self._rng.uniform(-0.1, 0.1, 2)
        self._ball_vel = np.zeros(2)
        self._kick_cooldown = 0
        return self._make_state()

    def step(self, vx_body: float, vy_body: float, omega: float,
             kick: float = 0.0) -> tuple[PhysicsState, dict]:
        # Ball position at the start of the step — lets _check_goal require a *crossing* of the
        # goal line through the mouth (interpolating y at the crossing) and so reject a "goal from
        # the side" (a ball that slipped into the strip past a side wall; see _check_goal).
        prev_bx = float(self._ball_pos[0])
        prev_by = float(self._ball_pos[1])
        mag = np.sqrt(vx_body**2 + vy_body**2)
        if mag > 1.0:
            vx_body /= mag
            vy_body /= mag
        # Inputs are normalized [-1, 1]; scale to physical units here (single source of scaling,
        # symmetric with TwoRobotPhysics._drive). The env must pass raw normalized actions.
        omega = float(np.clip(omega, -1.0, 1.0)) * MAX_OMEGA

        c, s = np.cos(self._robot_heading), np.sin(self._robot_heading)
        R = np.array([[c, -s], [s, c]])
        vel_world = R @ np.array([vx_body, vy_body]) * MAX_LINEAR

        alpha = self._dt / (MOTOR_TAU + self._dt)
        self._robot_vel_cmd += alpha * (vel_world - self._robot_vel_cmd)
        self._omega_cmd += alpha * (omega - self._omega_cmd)

        self._robot_pos += self._robot_vel_cmd * self._dt
        self._robot_heading += self._omega_cmd * self._dt
        self._robot_heading = (self._robot_heading + np.pi) % (2 * np.pi) - np.pi
        self._robot_vel = self._robot_vel_cmd.copy()
        self._robot_omega = self._omega_cmd

        self._ball_vel *= (1.0 - BALL_DAMPING * self._dt)
        self._ball_pos += self._ball_vel * self._dt

        self._ball_vel, self._kick_cooldown, kicked, kick_speed = apply_kick(
            self._ball_pos, self._ball_vel, self._robot_pos, self._robot_heading,
            kick, self._kick_cooldown,
        )

        self._resolve_robot_ball_collision()
        bounced = self._resolve_ball_walls(prev_bx, prev_by)

        # Robot is bounded by the arena walls, not the white lines — it may roam the
        # outer band freely.
        self._robot_pos = np.clip(
            self._robot_pos,
            [-ARENA_HALF_X + ROBOT_RADIUS, -ARENA_HALF_Y + ROBOT_RADIUS],
            [ARENA_HALF_X - ROBOT_RADIUS, ARENA_HALF_Y - ROBOT_RADIUS],
        )
        # The goal is a solid-walled recess: the robot may poke into the open mouth (penalised in
        # the reward) but cannot cross the side walls or drive through to the rear band.
        self._robot_pos = _resolve_robot_goal(self._robot_pos)

        goal = self._check_goal(prev_bx, prev_by)
        info = {
            "goal_scored": goal,
            "ball_out": self._check_ball_out(goal),
            "robot_fully_out": self._check_robot_fully_out(),
            "kicked": kicked,
            "kick_speed": kick_speed,
            "ball_wall_bounce": bounced,
        }
        return self._make_state(), info

    def _make_state(self) -> PhysicsState:
        return PhysicsState(
            robot_pos=self._robot_pos.copy(),
            robot_vel=self._robot_vel.copy(),
            robot_heading=self._robot_heading,
            robot_omega=self._robot_omega,
            ball_pos=self._ball_pos.copy(),
            ball_vel=self._ball_vel.copy(),
        )

    def _resolve_robot_ball_collision(self) -> None:
        delta = self._ball_pos - self._robot_pos
        dist = np.linalg.norm(delta)
        if dist < COLLISION_DIST and dist > 1e-6:
            normal = delta / dist
            overlap = COLLISION_DIST - dist
            self._ball_pos += normal * overlap
            rel_vel = self._ball_vel - self._robot_vel
            impulse = np.dot(rel_vel, normal)
            if impulse < 0:
                self._ball_vel -= (1 + BALL_RESTITUTION) * impulse * normal

    def _resolve_ball_walls(self, prev_x: float, prev_y: float) -> bool:
        # In-place wrapper over :func:`_resolve_ball_walls_pure` (the shared single source of truth,
        # also used by the look-ahead rollout). The ball bounces off the arena walls (rolling into
        # the outer band / corners), passes through the goal mouth to score, and bounces off the
        # solid goal side walls (swept test via ``prev_x``/``prev_y`` so a fast ball can't tunnel).
        # Returns True if any wall bounce occurred this step (used for bank-shot detection).
        self._ball_pos, self._ball_vel, bounced = _resolve_ball_walls_pure(
            self._ball_pos, self._ball_vel, prev_x, prev_y
        )
        return bounced

    def _check_goal(self, prev_x: float, prev_y: float) -> bool:
        # Goal = the ball occupying the goal box, or crossing a goal line through the mouth this
        # step (see _goal_scored). Tested on the *post-wall* ball position: solid posts/side/back
        # walls only let the ball into the box through the mouth, so a ball that slipped in from the
        # side has already been bounced back out of the strip and never counts (no goal from side).
        bx, by = float(self._ball_pos[0]), float(self._ball_pos[1])
        return (_goal_scored(prev_x, prev_y, bx, by, 1) or
                _goal_scored(prev_x, prev_y, bx, by, -1))

    def _check_ball_out(self, goal: bool) -> bool:
        """Ball has left the white-line field into the outer band (and isn't a goal)."""
        past_line = (abs(self._ball_pos[0]) > FIELD_W / 2 or
                     abs(self._ball_pos[1]) > FIELD_H / 2)
        return past_line and not goal

    def _check_robot_fully_out(self) -> bool:
        """Entire robot body has crossed the white-line boundary."""
        return (abs(self._robot_pos[0]) > FIELD_W / 2 + ROBOT_RADIUS or
                abs(self._robot_pos[1]) > FIELD_H / 2 + ROBOT_RADIUS)


ROBOT_COLLISION_DIST = 2 * ROBOT_RADIUS
KICKOFF_BEHIND = 0.14            # kickoff robot sits just behind the ball (≈ capture radius)
# Non-kicking robot starts inside its own penalty area (rule 4.4): centred in the
# strafschopgebied, which spans x ∈ [HALF_W - PENALTY_DEPTH, HALF_W].
DEFEND_X = HALF_W - PENALTY_DEPTH / 2   # ≈ 0.69 m, inside the penalty area
# A removed (suspended/defective) robot is parked here, well outside the arena.
_OFF_FIELD = np.array([0.0, -ARENA_HALF_Y * 2.0])


class TwoRobotPhysics:
    """1v1 omni-drive physics: two robots (A, B) + one shared ball.

    Robot A attacks the +x goal, robot B attacks the −x goal. The single-agent
    drive/collision/wall maths from :class:`PyPhysics` are reused per robot; the only
    additions are a second robot and an elastic robot↔robot separation. Goals are
    attributed per side: ``goal_a`` when the ball enters the +x goal, ``goal_b`` for −x.
    """

    def __init__(self, dt: float = DT) -> None:
        self._dt = dt
        self._rng = np.random.default_rng()
        self._a_pos = np.zeros(2)
        self._a_vel = np.zeros(2)
        self._a_heading = 0.0
        self._a_omega = 0.0
        self._b_pos = np.zeros(2)
        self._b_vel = np.zeros(2)
        self._b_heading = float(np.pi)
        self._b_omega = 0.0
        self._ball_pos = np.zeros(2)
        self._ball_vel = np.zeros(2)
        self._a_kick_cooldown = 0
        self._b_kick_cooldown = 0
        # Referee-controlled removal: a removed robot is parked off-field and ignores
        # its action until the referee re-enters it.
        self._a_removed = False
        self._b_removed = False

    @property
    def dt(self) -> float:
        return self._dt

    # ── referee setters ─────────────────────────────────────────────────────
    def place_ball(self, pos, vel=(0.0, 0.0)) -> None:
        self._ball_pos = np.asarray(pos, dtype=float).copy()
        self._ball_vel = np.asarray(vel, dtype=float).copy()

    def place_robot(self, which: str, pos, heading: float | None = None) -> None:
        pos = np.asarray(pos, dtype=float).copy()
        if which == "a":
            self._a_pos = pos
            self._a_vel = np.zeros(2)
            self._a_omega = 0.0
            if heading is not None:
                self._a_heading = float(heading)
        else:
            self._b_pos = pos
            self._b_vel = np.zeros(2)
            self._b_omega = 0.0
            if heading is not None:
                self._b_heading = float(heading)

    def set_removed(self, which: str, removed: bool) -> None:
        """Remove/restore a robot. A removed robot is parked off-field and its action
        is ignored each step until restored (the referee re-enters it via place_robot)."""
        if which == "a":
            self._a_removed = removed
            if removed:
                self.place_robot("a", _OFF_FIELD)
        else:
            self._b_removed = removed
            if removed:
                self.place_robot("b", _OFF_FIELD)

    def is_removed(self, which: str) -> bool:
        return self._a_removed if which == "a" else self._b_removed

    def reset(self, seed: int | None = None, kickoff: str | None = None,
              spawn_jitter: float = 0.05) -> None:
        """Kick-off reset. ``kickoff`` ('a'|'b', else random) gets the ball; the other
        robot starts back in its own goal/black zone. After a goal, the conceding team
        is given the kickoff (RoboCup rule), driven by the caller passing ``kickoff``.

        ``spawn_jitter`` is the lateral (y) spread of the ball and robot start positions; the
        longitudinal (x) spread stays tight at 0.05 m so the kickoff geometry holds. The
        default reproduces the original ±0.05 m jitter; self-play widens it for variety.
        """
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        if kickoff not in ("a", "b"):
            kickoff = "a" if self._rng.random() < 0.5 else "b"

        lon = 0.05  # longitudinal (x) jitter, kept tight to preserve kickoff geometry
        lat = max(0.0, float(spawn_jitter))

        def _jit() -> np.ndarray:
            return np.array([self._rng.uniform(-lon, lon), self._rng.uniform(-lat, lat)])

        ball = _jit()
        a_noise = _jit()
        b_noise = _jit()
        if kickoff == "a":
            self._a_pos = ball + np.array([-KICKOFF_BEHIND, 0.0]) + a_noise  # A on the ball
            self._b_pos = np.array([DEFEND_X, 0.0]) + b_noise                # B in its zone
        else:
            self._b_pos = ball + np.array([KICKOFF_BEHIND, 0.0]) + b_noise   # B on the ball
            self._a_pos = np.array([-DEFEND_X, 0.0]) + a_noise               # A in its zone

        self._a_vel = np.zeros(2)
        self._a_heading = 0.0 + self._rng.uniform(-0.2, 0.2)
        self._a_omega = 0.0
        self._b_vel = np.zeros(2)
        self._b_heading = float(np.pi) + self._rng.uniform(-0.2, 0.2)
        self._b_omega = 0.0
        self._ball_pos = ball
        self._ball_vel = np.zeros(2)
        self._a_kick_cooldown = 0
        self._b_kick_cooldown = 0
        self._a_removed = False
        self._b_removed = False

    def step(self, action_a, action_b) -> dict:
        # Ball x at the start of the step, so the goal check can require the ball to *cross* a
        # goal line through the mouth this step rather than merely sit in the strip — which is
        # what rejects "goals from the side" (see the goal check below).
        prev_bx = float(self._ball_pos[0])
        prev_by = float(self._ball_pos[1])
        if not self._a_removed:
            self._a_pos, self._a_vel, self._a_heading, self._a_omega = self._drive(
                self._a_pos, self._a_vel, self._a_heading, self._a_omega, action_a
            )
        if not self._b_removed:
            self._b_pos, self._b_vel, self._b_heading, self._b_omega = self._drive(
                self._b_pos, self._b_vel, self._b_heading, self._b_omega, action_b
            )

        self._ball_vel = self._ball_vel * (1.0 - BALL_DAMPING * self._dt)
        self._ball_pos = self._ball_pos + self._ball_vel * self._dt

        # Kicks (4th action dim). A removed robot can't kick; its cooldown still ticks.
        kick_a = float(action_a[3]) if len(action_a) > 3 else 0.0
        kick_b = float(action_b[3]) if len(action_b) > 3 else 0.0
        kicked_a = kicked_b = False
        kick_speed_a = kick_speed_b = 0.0
        if not self._a_removed:
            self._ball_vel, self._a_kick_cooldown, kicked_a, kick_speed_a = apply_kick(
                self._ball_pos, self._ball_vel, self._a_pos, self._a_heading,
                kick_a, self._a_kick_cooldown,
            )
        if not self._b_removed:
            self._ball_vel, self._b_kick_cooldown, kicked_b, kick_speed_b = apply_kick(
                self._ball_pos, self._ball_vel, self._b_pos, self._b_heading,
                kick_b, self._b_kick_cooldown,
            )

        if not self._a_removed:
            self._resolve_robot_ball(self._a_pos, self._a_vel)
        if not self._b_removed:
            self._resolve_robot_ball(self._b_pos, self._b_vel)
        if not (self._a_removed or self._b_removed):
            self._resolve_robot_robot()
        bounced = self._resolve_ball_walls(prev_bx, prev_by)

        if not self._a_removed:
            self._a_pos = _resolve_robot_goal(self._clamp_robot(self._a_pos))
        if not self._b_removed:
            self._b_pos = _resolve_robot_goal(self._clamp_robot(self._b_pos))

        # A goal is the ball occupying a goal box, or crossing a goal line through the mouth this
        # step (see _goal_scored — occupancy catches multi-step / diagonal entries, the swept
        # crossing the exact-crossing step). Tested on the post-wall position: solid posts/side/back
        # walls only let the ball into the box through the mouth, so a ball that slipped in from the
        # side has already been bounced back out and never counts as a goal from the side.
        bx, by = float(self._ball_pos[0]), float(self._ball_pos[1])
        goal_a = _goal_scored(prev_bx, prev_by, bx, by, 1)
        goal_b = _goal_scored(prev_bx, prev_by, bx, by, -1)
        past_line = bool(abs(self._ball_pos[0]) > FIELD_W / 2 or abs(self._ball_pos[1]) > FIELD_H / 2)
        return {
            "goal_a": goal_a,
            "goal_b": goal_b,
            "ball_out": past_line and not (goal_a or goal_b),
            "kicked_a": kicked_a,
            "kicked_b": kicked_b,
            "kick_speed_a": kick_speed_a,
            "kick_speed_b": kick_speed_b,
            "ball_wall_bounce": bounced,
        }

    def state_a(self) -> PhysicsState:
        return PhysicsState(
            robot_pos=self._a_pos.copy(), robot_vel=self._a_vel.copy(),
            robot_heading=self._a_heading, robot_omega=self._a_omega,
            ball_pos=self._ball_pos.copy(), ball_vel=self._ball_vel.copy(),
        )

    def state_b(self) -> PhysicsState:
        return PhysicsState(
            robot_pos=self._b_pos.copy(), robot_vel=self._b_vel.copy(),
            robot_heading=self._b_heading, robot_omega=self._b_omega,
            ball_pos=self._ball_pos.copy(), ball_vel=self._ball_vel.copy(),
        )

    # ── helpers ───────────────────────────────────────────────────────────────
    def _drive(self, pos, vel, heading, omega, action):
        vx_body, vy_body, omega_cmd = float(action[0]), float(action[1]), float(action[2])
        mag = np.sqrt(vx_body**2 + vy_body**2)
        if mag > 1.0:
            vx_body /= mag
            vy_body /= mag
        # Action dim [2] is normalized [-1, 1]; scale to the physical max turn rate. (Earlier
        # this only *clipped* to ±MAX_OMEGA, so a normalized action could never exceed 1 rad/s
        # in matches/self-play — robots, including the human's, barely turned.)
        omega_cmd = float(np.clip(omega_cmd, -1.0, 1.0)) * MAX_OMEGA

        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])
        vel_world = R @ np.array([vx_body, vy_body]) * MAX_LINEAR

        alpha = self._dt / (MOTOR_TAU + self._dt)
        vel = vel + alpha * (vel_world - vel)
        omega = omega + alpha * (omega_cmd - omega)

        pos = pos + vel * self._dt
        heading = (heading + omega * self._dt + np.pi) % (2 * np.pi) - np.pi
        return pos, vel, heading, omega

    def _resolve_robot_ball(self, r_pos, r_vel) -> None:
        delta = self._ball_pos - r_pos
        dist = np.linalg.norm(delta)
        if 1e-6 < dist < COLLISION_DIST:
            normal = delta / dist
            self._ball_pos = self._ball_pos + normal * (COLLISION_DIST - dist)
            rel_vel = self._ball_vel - r_vel
            impulse = np.dot(rel_vel, normal)
            if impulse < 0:
                self._ball_vel = self._ball_vel - (1 + BALL_RESTITUTION) * impulse * normal

    def _resolve_robot_robot(self) -> None:
        delta = self._b_pos - self._a_pos
        dist = np.linalg.norm(delta)
        if 1e-6 < dist < ROBOT_COLLISION_DIST:
            normal = delta / dist
            overlap = ROBOT_COLLISION_DIST - dist
            self._a_pos = self._a_pos - normal * (overlap / 2)
            self._b_pos = self._b_pos + normal * (overlap / 2)
        elif dist <= 1e-6:
            # Exactly coincident — shove apart along +x deterministically.
            self._a_pos = self._a_pos - np.array([ROBOT_RADIUS, 0.0])
            self._b_pos = self._b_pos + np.array([ROBOT_RADIUS, 0.0])

    @staticmethod
    def _clamp_robot(pos):
        return np.clip(
            pos,
            [-ARENA_HALF_X + ROBOT_RADIUS, -ARENA_HALF_Y + ROBOT_RADIUS],
            [ARENA_HALF_X - ROBOT_RADIUS, ARENA_HALF_Y - ROBOT_RADIUS],
        )

    # Reused verbatim from PyPhysics (same ball/wall geometry).
    _resolve_ball_walls = PyPhysics._resolve_ball_walls
