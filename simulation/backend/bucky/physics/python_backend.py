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
    GOAL_WIDTH,
    HALF_W,
    PENALTY_DEPTH,
    ROBOT_RADIUS,
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
        # Ball x at the start of the step — lets _check_goal require a *crossing* of the goal
        # line through the mouth and so reject a "goal from the side" (a ball that slipped into
        # the strip past a side wall rather than through the mouth; see _check_goal).
        prev_bx = float(self._ball_pos[0])
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
        bounced = self._resolve_ball_walls()

        # Robot is bounded by the arena walls, not the white lines — it may roam the
        # outer band freely.
        self._robot_pos = np.clip(
            self._robot_pos,
            [-ARENA_HALF_X + ROBOT_RADIUS, -ARENA_HALF_Y + ROBOT_RADIUS],
            [ARENA_HALF_X - ROBOT_RADIUS, ARENA_HALF_Y - ROBOT_RADIUS],
        )

        goal = self._check_goal(prev_bx)
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

    def _resolve_ball_walls(self) -> bool:
        # The ball bounces off the arena walls (so it can roll into the outer band and
        # get pinned in the corners). The goal mouth is an opening in the white-line
        # goal line, so a ball heading into it passes through to score instead.
        # Returns True if any wall bounce occurred this step (used for bank-shot detection).
        bounced = False
        hx, hy = ARENA_HALF_X - BALL_RADIUS, ARENA_HALF_Y - BALL_RADIUS
        ghw = GOAL_WIDTH / 2
        in_goal_y = abs(self._ball_pos[1]) < ghw
        if not in_goal_y:
            if self._ball_pos[0] < -hx:
                self._ball_pos[0] = -hx
                self._ball_vel[0] = abs(self._ball_vel[0]) * BALL_RESTITUTION
                bounced = True
            elif self._ball_pos[0] > hx:
                self._ball_pos[0] = hx
                self._ball_vel[0] = -abs(self._ball_vel[0]) * BALL_RESTITUTION
                bounced = True
        if abs(self._ball_pos[1]) > hy:
            self._ball_vel[1] *= -BALL_RESTITUTION
            self._ball_pos[1] = np.sign(self._ball_pos[1]) * hy
            bounced = True

        # Goal side walls. The goal is a box recessed behind the goal line: its mouth (the
        # GOAL_WIDTH opening at x = ±HALF_W) is open, but the two side walls running back
        # from the goalposts at y = ±GOAL_WIDTH/2 are solid. Without them a ball loose in
        # the neutral band behind a goal line could drift laterally into the goal strip and
        # score "from the side" without ever passing through the mouth. Keep a ball that is
        # behind a goal line and outside the mouth on the outside of these walls; the only
        # way into the strip (and thus a goal) is through the mouth at the goal line itself.
        if abs(self._ball_pos[0]) > HALF_W:
            side = ghw + BALL_RADIUS
            if ghw <= abs(self._ball_pos[1]) < side:
                self._ball_pos[1] = np.sign(self._ball_pos[1]) * side
                self._ball_vel[1] = np.sign(self._ball_pos[1]) * abs(self._ball_vel[1]) * BALL_RESTITUTION
                bounced = True
        return bounced

    def _check_goal(self, prev_x: float) -> bool:
        # Goal = the ball *crossing* a goal line through the mouth this step: it started on the
        # field side of the line and ended past it, within the mouth in y. Testing the crossing
        # (not just "is the ball in the strip") rejects a goal from the side — a ball that
        # slipped laterally past a side wall into the strip (a fast ball can tunnel the thin
        # side-wall band) never crossed the line from the field, so it never scores.
        bx, by = self._ball_pos[0], self._ball_pos[1]
        if abs(by) >= GOAL_WIDTH / 2:
            return False
        return bool(prev_x <= FIELD_W / 2 < bx or prev_x >= -FIELD_W / 2 > bx)

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
        bounced = self._resolve_ball_walls()

        if not self._a_removed:
            self._a_pos = self._clamp_robot(self._a_pos)
        if not self._b_removed:
            self._b_pos = self._clamp_robot(self._b_pos)

        # A goal is the ball *crossing* a goal line through the mouth this step: it started on
        # the field side of the line (|x| <= HALF_W) and ended past it, within the mouth in y.
        # Testing the crossing (not just "is the ball in the strip") is what rejects a goal
        # from the side — a ball that slipped laterally past a side wall into the strip (e.g. a
        # fast ball tunnelling the thin side-wall band) never crossed the line from the field,
        # so it never scores however long it then lingers behind the line.
        ghw = GOAL_WIDTH / 2
        bx, by = self._ball_pos[0], self._ball_pos[1]
        in_mouth = abs(by) < ghw
        goal_a = bool(prev_bx <= FIELD_W / 2 < bx and in_mouth)
        goal_b = bool(prev_bx >= -FIELD_W / 2 > bx and in_mouth)
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
