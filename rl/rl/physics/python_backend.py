"""Pure NumPy 2D rigid-body physics for Bucky.

Assumptions:
- Robot radius: 0.11 m (≈22 cm diameter)
- Ball radius: 0.021 m (standard size-3)
- Robot max speed: 1.0 m/s linear, 6.0 rad/s angular
- Timestep: 0.02 s (50 Hz)
- Field: 2.4 m × 1.8 m, goals on ±x wall, 0.6 m wide, centered
- Robot drives commands applied with first-order lag (τ=0.05 s motor lag)
- Ball: point-mass with damping (μ=0.3), elastic wall bounces, soft collision with robot
"""
from __future__ import annotations
import numpy as np
from rl.physics.backend import PhysicsBackend, PhysicsState

# ── Field geometry (meters) ─────────────────────────────────────────────────
FIELD_W = 2.4
FIELD_H = 1.8
GOAL_WIDTH = 0.6
ROBOT_RADIUS = 0.11
BALL_RADIUS = 0.021
COLLISION_DIST = ROBOT_RADIUS + BALL_RADIUS

MAX_LINEAR = 1.0   # m/s
MAX_OMEGA = 6.0    # rad/s
DT = 0.02          # s
MOTOR_TAU = 0.05   # first-order lag time constant (s)
BALL_DAMPING = 0.3
BALL_RESTITUTION = 0.7


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
        return self._make_state()

    def step(self, vx_body: float, vy_body: float, omega: float) -> tuple[PhysicsState, dict]:
        mag = np.sqrt(vx_body**2 + vy_body**2)
        if mag > 1.0:
            vx_body /= mag
            vy_body /= mag
        omega = np.clip(omega, -MAX_OMEGA, MAX_OMEGA)

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

        self._resolve_robot_ball_collision()
        self._resolve_ball_walls()

        self._robot_pos = np.clip(
            self._robot_pos,
            [-FIELD_W / 2 + ROBOT_RADIUS, -FIELD_H / 2 + ROBOT_RADIUS],
            [FIELD_W / 2 - ROBOT_RADIUS, FIELD_H / 2 - ROBOT_RADIUS],
        )

        info = {
            "goal_scored": self._check_goal(),
            "out_of_bounds": self._check_out_of_bounds(),
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

    def _resolve_ball_walls(self) -> None:
        hx, hy = FIELD_W / 2 - BALL_RADIUS, FIELD_H / 2 - BALL_RADIUS
        in_goal_y = abs(self._ball_pos[1]) < GOAL_WIDTH / 2
        if not in_goal_y:
            if self._ball_pos[0] < -hx:
                self._ball_pos[0] = -hx
                self._ball_vel[0] = abs(self._ball_vel[0]) * BALL_RESTITUTION
            elif self._ball_pos[0] > hx:
                self._ball_pos[0] = hx
                self._ball_vel[0] = -abs(self._ball_vel[0]) * BALL_RESTITUTION
        if abs(self._ball_pos[1]) > hy:
            self._ball_vel[1] *= -BALL_RESTITUTION
            self._ball_pos[1] = np.sign(self._ball_pos[1]) * hy

    def _check_goal(self) -> bool:
        return (abs(self._ball_pos[0]) > FIELD_W / 2 and
                abs(self._ball_pos[1]) < GOAL_WIDTH / 2)

    def _check_out_of_bounds(self) -> bool:
        return (abs(self._robot_pos[0]) > FIELD_W / 2 or
                abs(self._robot_pos[1]) > FIELD_H / 2)
