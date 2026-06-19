"""Self-play / 1v1 helpers: x-axis mirror trick, sonar sensing, opponent-aware obs.

The single-agent policy is trained to attack the **+x** goal. Robot B attacks **−x**, so
to drive B with the same policy we reflect the world across the x-axis (B then "sees" itself
attacking +x), run the policy, and un-mirror the resulting action. Opponent perception is via
a 4-beam sonar model (the real robot only has 4 ultrasonic sensors at 90° spacing), appended
to the 17-dim single-agent observation → a 21-dim opponent-aware observation.
"""
from __future__ import annotations
import numpy as np

from bucky.obs import OBS_DIM, build_observation
from bucky.physics.backend import PhysicsState
from bucky.physics.python_backend import ARENA_HALF_X, ARENA_HALF_Y, ROBOT_RADIUS

# 4 ultrasonic beams, body-fixed: forward, left, right, back.
SONAR_BEAMS = (0.0, np.pi / 2, -np.pi / 2, np.pi)
SONAR_HALF_FOV = np.pi / 4          # ±45° sector per beam (full 360° coverage)
MAX_SONAR_RANGE = 1.5               # metres; beyond this a beam reads "clear" (1.0)
_SONAR_NOISE_STD = 0.02             # metres, when domain randomization is on
_SONAR_DROPOUT_P = 0.05             # chance a beam misses (reads clear)

SELF_PLAY_OBS_DIM = OBS_DIM + 4     # 17 base + 4 sonar = 21


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── x-axis reflection primitives ────────────────────────────────────────────
def reflect_pos(p: np.ndarray) -> np.ndarray:
    return np.array([-p[0], p[1]])


def reflect_vel(v: np.ndarray) -> np.ndarray:
    return np.array([-v[0], v[1]])


def reflect_heading(h: float) -> float:
    """A heading reflected across the y-axis: +x-facing (0) → −x-facing (π)."""
    return _wrap(np.pi - h)


def reflect_omega(w: float) -> float:
    return -w


def mirror_action(a) -> np.ndarray:
    """Body-frame action under x-reflection: forward unchanged, strafe & spin flip."""
    a = np.asarray(a, dtype=np.float32)
    return np.array([a[0], -a[1], -a[2]], dtype=np.float32)


def reflect_state(state: PhysicsState) -> PhysicsState:
    return PhysicsState(
        robot_pos=reflect_pos(state.robot_pos),
        robot_vel=reflect_vel(state.robot_vel),
        robot_heading=reflect_heading(state.robot_heading),
        robot_omega=reflect_omega(state.robot_omega),
        ball_pos=reflect_pos(state.ball_pos),
        ball_vel=reflect_vel(state.ball_vel),
    )


# ── sonar sensor model ───────────────────────────────────────────────────────
def _ray_to_arena(pos: np.ndarray, direction: np.ndarray) -> float:
    """Distance from ``pos`` along unit ``direction`` to the arena wall box."""
    ts = []
    for axis, half in ((0, ARENA_HALF_X), (1, ARENA_HALF_Y)):
        d = direction[axis]
        if abs(d) > 1e-9:
            bound = np.copysign(half, d)
            t = (bound - pos[axis]) / d
            if t > 0:
                ts.append(t)
    return min(ts) if ts else MAX_SONAR_RANGE


def sonar_ranges(
    robot_pos: np.ndarray,
    robot_heading: float,
    opponent_pos: np.ndarray | None,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """4 normalized beam readings (1.0 = clear, 0.0 = touching).

    Each beam returns the nearest of the opponent robot (within its ±45° sector) or the
    arena wall (ray-cast), capped at ``MAX_SONAR_RANGE``.
    """
    if rng is None:
        rng = np.random.default_rng()
    pos = np.asarray(robot_pos, dtype=float)
    readings = np.empty(4, dtype=np.float32)

    opp_bearing = opp_dist = None
    if opponent_pos is not None:
        delta = np.asarray(opponent_pos, dtype=float) - pos
        opp_dist = float(np.linalg.norm(delta))
        opp_bearing = float(np.arctan2(delta[1], delta[0]))

    for i, beam in enumerate(SONAR_BEAMS):
        world_angle = robot_heading + beam
        direction = np.array([np.cos(world_angle), np.sin(world_angle)])
        dist = _ray_to_arena(pos, direction)
        if opp_bearing is not None and abs(_wrap(opp_bearing - world_angle)) <= SONAR_HALF_FOV:
            dist = min(dist, max(0.0, opp_dist - ROBOT_RADIUS))
        if add_noise:
            if rng.random() < _SONAR_DROPOUT_P:
                dist = MAX_SONAR_RANGE
            else:
                dist += rng.normal(0.0, _SONAR_NOISE_STD)
        readings[i] = np.clip(dist / MAX_SONAR_RANGE, 0.0, 1.0)
    return readings


# ── opponent-aware observation (21-dim) ──────────────────────────────────────
def build_robot_obs(
    state: PhysicsState,
    opponent_pos: np.ndarray | None,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
    heading_drift: float = 0.0,
) -> np.ndarray:
    base = build_observation(state, add_noise=add_noise, rng=rng, heading_drift=heading_drift)
    sonar = sonar_ranges(state.robot_pos, state.robot_heading, opponent_pos,
                         add_noise=add_noise, rng=rng)
    return np.concatenate([base, sonar]).astype(np.float32)


def build_opponent_obs(
    b_state: PhysicsState,
    opp_a_pos: np.ndarray,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
    heading_drift: float = 0.0,
) -> np.ndarray:
    """Opponent-aware obs for robot B, built in B's reflected (+x-attacking) frame."""
    mirrored = reflect_state(b_state)
    return build_robot_obs(mirrored, reflect_pos(np.asarray(opp_a_pos, dtype=float)),
                           add_noise=add_noise, rng=rng, heading_drift=heading_drift)


def predict_opponent_action(
    model,
    b_state: PhysicsState,
    opp_a_pos: np.ndarray,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Run ``model`` for robot B via the mirror trick; returns a real-world body action."""
    obs = build_opponent_obs(b_state, opp_a_pos, add_noise=add_noise, rng=rng)
    raw, _ = model.predict(obs, deterministic=True)
    return mirror_action(raw)
