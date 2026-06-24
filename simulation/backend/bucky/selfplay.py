"""Self-play / 1v1 helpers: x-axis mirror trick, sonar sensing, opponent-aware obs.

The single-agent policy is trained to attack the **+x** goal. Robot B attacks **−x**, so
to drive B with the same policy we reflect the world across the x-axis (B then "sees" itself
attacking +x), run the policy, and un-mirror the resulting action. Opponent perception is via
a 4-beam sonar model (the real robot only has 4 ultrasonic sensors at 90° spacing), appended
to the 23-dim single-agent observation → a 27-dim opponent-aware observation.
"""
from __future__ import annotations
import numpy as np

from bucky.obs import (
    LEGACY_OBS_DIM,
    OBS_DIM,
    PRE_GOAL_REL_OBS_DIM,
    PRE_KICK_PRED_OBS_DIM,
    build_observation,
)
from bucky.physics.backend import PhysicsState
from bucky.physics.python_backend import ARENA_HALF_X, ARENA_HALF_Y, ROBOT_RADIUS

# 4 ultrasonic beams, body-fixed: forward, left, right, back.
SONAR_BEAMS = (0.0, np.pi / 2, -np.pi / 2, np.pi)
SONAR_HALF_FOV = np.pi / 4          # ±45° sector per beam (full 360° coverage)
MAX_SONAR_RANGE = 1.5               # metres; beyond this a beam reads "clear" (1.0)
_SONAR_NOISE_STD = 0.02             # metres, when domain randomization is on
_SONAR_DROPOUT_P = 0.05             # chance a beam misses (reads clear)

SELF_PLAY_OBS_DIM = OBS_DIM + 4     # 39 base + 4 sonar = 43

# ── backward compatibility: drive older, narrower policies in a current match ─────────────
# The current opponent-aware obs is laid out [base (OBS_DIM) | sonar (4)], and the base grew by
# appending blocks: [pre-boundary base (LEGACY_OBS_DIM) | ball-boundary block | kick-prediction
# block | ball→goal block]. Each block was inserted *before* the sonar tail, so an older policy's
# obs is reconstructed by keeping the prefix of the base it knew plus the (unchanged) sonar tail and
# dropping the blocks added after it. Map each supported legacy width → the current-obs indices that
# rebuild it.
_SONAR_TAIL = np.arange(OBS_DIM, SELF_PLAY_OBS_DIM)   # the 4 sonar dims, now at [OBS_DIM:OBS_DIM+4]
_LEGACY_OBS_INDICES: dict[int, np.ndarray] = {
    LEGACY_OBS_DIM + 4: np.concatenate([            # 22-dim: [base18 | sonar4]
        np.arange(LEGACY_OBS_DIM), _SONAR_TAIL]),
    PRE_KICK_PRED_OBS_DIM + 4: np.concatenate([     # 27-dim: [base23 (pre-kick-pred) | sonar4]
        np.arange(PRE_KICK_PRED_OBS_DIM), _SONAR_TAIL]),
    PRE_GOAL_REL_OBS_DIM + 4: np.concatenate([      # 39-dim: [base35 (pre-ball→goal) | sonar4]
        np.arange(PRE_GOAL_REL_OBS_DIM), _SONAR_TAIL]),
}
# Observation widths a match can run: the native one plus any we can project down to.
MATCH_COMPATIBLE_OBS_DIMS = frozenset({SELF_PLAY_OBS_DIM, *_LEGACY_OBS_INDICES})


def adapt_obs_to_policy(obs, policy_obs_dim: int) -> np.ndarray:
    """Project the current opponent-aware obs onto the layout a ``policy_obs_dim``-input policy
    was trained on, so older checkpoints can still play. A policy that already matches the
    current width gets the obs unchanged; an unknown width raises (caller should reject it)."""
    obs = np.asarray(obs, dtype=np.float32).reshape(-1)
    if policy_obs_dim == SELF_PLAY_OBS_DIM:
        return obs
    idx = _LEGACY_OBS_INDICES.get(policy_obs_dim)
    if idx is None:
        raise ValueError(
            f"cannot adapt the {SELF_PLAY_OBS_DIM}-dim match observation to a "
            f"{policy_obs_dim}-dim policy (supported: {sorted(MATCH_COMPATIBLE_OBS_DIMS)})")
    return obs[idx]


# Single-agent base widths (no sonar): pure prefixes of the current OBS_DIM base, since each
# obs block was *appended* (18 → 23 → 35 → 39). A single-agent policy of one of these widths is
# driven by slicing that prefix off the current base.
SINGLE_COMPATIBLE_OBS_DIMS = frozenset(
    {LEGACY_OBS_DIM, PRE_KICK_PRED_OBS_DIM, PRE_GOAL_REL_OBS_DIM, OBS_DIM})  # {18, 23, 35, 39}
# Every observation width the eval tool can drive — single-agent (no sonar) or opponent-aware.
EVAL_COMPATIBLE_OBS_DIMS = SINGLE_COMPATIBLE_OBS_DIMS | MATCH_COMPATIBLE_OBS_DIMS


def project_obs(full_obs, policy_obs_dim: int, opponent_aware: bool) -> np.ndarray:
    """Project the full 43-dim opponent-aware obs down to any legacy policy's layout.

    ``opponent_aware`` disambiguates the one width that two eras share: a 39-dim policy is either
    the current single-agent base (39, no sonar) or a legacy opponent-aware policy (35 base + 4
    sonar). For all other widths the dimension alone fixes the layout. The caller builds the full
    obs (in a single drill, the four sonar dims are wall-only — there is no opponent on the field).
    """
    full = np.asarray(full_obs, dtype=np.float32).reshape(-1)
    if opponent_aware:
        return adapt_obs_to_policy(full, policy_obs_dim)
    if policy_obs_dim in SINGLE_COMPATIBLE_OBS_DIMS:
        return full[:policy_obs_dim]
    raise ValueError(
        f"cannot project to a {policy_obs_dim}-dim single-agent policy "
        f"(supported: {sorted(SINGLE_COMPATIBLE_OBS_DIMS)})")


class CompatPolicy:
    """Wraps a policy whose observation is narrower than the current match obs, projecting each
    observation down to that policy's layout before predicting. A drop-in for the wrapped model
    (forwards ``predict`` and exposes ``observation_space``) so the match engine and the mirror
    path use it unchanged."""

    def __init__(self, model, policy_obs_dim: int) -> None:
        self._model = model
        self._dim = policy_obs_dim

    @property
    def observation_space(self):
        return self._model.observation_space

    def predict(self, obs, **kwargs):
        return self._model.predict(adapt_obs_to_policy(obs, self._dim), **kwargs)


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
    """Body-frame action under x-reflection: forward & kick unchanged, strafe & spin flip.

    Accepts a 3-D drive action or a 4-D drive+kick action (kick is frame-invariant)."""
    a = np.asarray(a, dtype=np.float32)
    mirrored = [a[0], -a[1], -a[2]]
    if a.shape[0] > 3:
        mirrored.append(a[3])
    return np.array(mirrored, dtype=np.float32)


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


# ── opponent-aware observation (27-dim) ──────────────────────────────────────
def build_robot_obs(
    state: PhysicsState,
    opponent_pos: np.ndarray | None,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
    heading_drift: float = 0.0,
    kick_ready: float = 1.0,
) -> np.ndarray:
    base = build_observation(state, add_noise=add_noise, rng=rng, heading_drift=heading_drift,
                             kick_ready=kick_ready, opponent_pos=opponent_pos)
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
    kick_ready: float = 1.0,
) -> np.ndarray:
    """Opponent-aware obs for robot B, built in B's reflected (+x-attacking) frame."""
    mirrored = reflect_state(b_state)
    return build_robot_obs(mirrored, reflect_pos(np.asarray(opp_a_pos, dtype=float)),
                           add_noise=add_noise, rng=rng, heading_drift=heading_drift,
                           kick_ready=kick_ready)


def predict_opponent_action(
    model,
    b_state: PhysicsState,
    opp_a_pos: np.ndarray,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
    kick_ready: float = 1.0,
    deterministic: bool = False,
    temperature: float = 1.0,
) -> np.ndarray:
    """Run ``model`` for robot B via the mirror trick; returns a real-world body action.

    During training the opponent is sampled stochastically (``deterministic=False``) so the
    learner faces a non-deterministic copy of itself rather than one fixed pattern it can
    perfectly counter. ``temperature`` scales the opponent's exploration noise. The viz/eval
    path passes ``deterministic=True`` for a crisp opponent.
    """
    obs = build_opponent_obs(b_state, opp_a_pos, add_noise=add_noise, rng=rng, kick_ready=kick_ready)
    if deterministic:
        # Standard SB3-compatible signature (model may be a real PPO at match/eval time).
        raw, _ = model.predict(obs, deterministic=True)
    else:
        # Stochastic sampling is a training-only device — only the NumpyOpponent path, which
        # accepts the rng/temperature kwargs, is ever used here.
        raw, _ = model.predict(obs, deterministic=False, rng=rng, temperature=temperature)
    return mirror_action(raw)


# ── pure-numpy frozen opponent (no torch in the workers) ─────────────────────
# Self-play runs the frozen opponent inside every SubprocVecEnv worker. Loading a full
# PPO/torch model there imports torch (~0.5 GB RSS) into each of the N workers, which on
# a memory-capped host OOM-kills the trainer. The opponent only ever needs a deterministic
# forward pass through a tiny MLP, so we evaluate it in numpy instead and keep torch out
# of the workers entirely. The trainer (which has torch) exports the policy weights to a
# .npz next to each snapshot via :func:`export_policy_npz`; workers load that with
# :func:`load_numpy_opponent`.

class NumpyOpponent:
    """A frozen PPO ``MlpPolicy`` evaluated in pure numpy.

    Replicates SB3's deterministic action for the default (non-squashed) Box policy used
    here: the action is the Gaussian mean ``action_net(policy_net(obs))`` with ``tanh``
    activations between the hidden layers, then clipped to ``[-1, 1]`` exactly as
    ``BasePolicy.predict`` does. Valid only for that config — identity (Flatten) features,
    no ``VecNormalize``, ``tanh`` activation, separate (un-shared) policy layers — which is
    what :mod:`scripts.train` builds.
    """

    def __init__(self, hidden, out_w: np.ndarray, out_b: np.ndarray,
                 log_std: np.ndarray | None = None) -> None:
        self._hidden = hidden          # list of (W, b) tanh layers, applied in order
        self._out_w = out_w
        self._out_b = out_b
        # Per-dim Gaussian log-std (SB3's state-independent ``policy.log_std``). ``None`` for
        # legacy .npz snapshots without it → the opponent stays deterministic (mean action).
        self._log_std = None if log_std is None else np.asarray(log_std, dtype=np.float32).reshape(-1)

    def predict(self, obs, deterministic: bool = True, rng=None, temperature: float = 1.0):
        """Mirror of ``model.predict`` — returns ``(action, None)`` so it is a drop-in for
        :func:`predict_opponent_action`.

        ``deterministic=True`` returns the policy mean (matching ``PPO.predict``). With
        ``deterministic=False`` and a stored ``log_std`` the action is *sampled* from the
        diagonal Gaussian — ``mean + temperature * exp(log_std) * N(0, 1)`` — so the frozen
        opponent explores instead of replaying one memorizable pattern. Without a ``log_std``
        (legacy snapshot) it falls back to the mean.
        """
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        for w, b in self._hidden:
            x = np.tanh(w @ x + b)
        mean = self._out_w @ x + self._out_b
        if not deterministic and self._log_std is not None:
            if rng is None:
                rng = np.random.default_rng()
            noise = rng.standard_normal(mean.shape[0]).astype(np.float32)
            mean = mean + temperature * np.exp(self._log_std) * noise
        a = np.clip(mean, -1.0, 1.0).astype(np.float32)
        return a, None


def load_numpy_opponent(path: str) -> NumpyOpponent:
    """Load a :class:`NumpyOpponent` from a ``.npz`` written by :func:`export_policy_npz`.
    Pure numpy — safe to call inside a SubprocVecEnv worker without importing torch."""
    data = np.load(path)
    n_hidden = int(data["n_hidden"])
    hidden = [(data[f"h{i}_W"].astype(np.float32), data[f"h{i}_b"].astype(np.float32))
              for i in range(n_hidden)]
    log_std = data["log_std"].astype(np.float32) if "log_std" in data.files else None
    return NumpyOpponent(hidden, data["out_W"].astype(np.float32),
                         data["out_b"].astype(np.float32), log_std=log_std)


def export_policy_npz(model, path: str) -> None:
    """Dump a PPO ``MlpPolicy``'s deterministic-action weights to ``path`` (.npz) so the
    workers can drive the frozen opponent in numpy. Runs in the trainer (torch) process
    only; torch is imported lazily so importing this module in a worker never pulls it in.
    """
    import torch.nn as nn  # lazy: keep torch out of worker imports

    pol = model.policy
    layers = [m for m in pol.mlp_extractor.policy_net if isinstance(m, nn.Linear)]
    arrays: dict[str, np.ndarray] = {"n_hidden": np.array(len(layers))}
    for i, m in enumerate(layers):
        arrays[f"h{i}_W"] = m.weight.detach().cpu().numpy()
        arrays[f"h{i}_b"] = m.bias.detach().cpu().numpy()
    arrays["out_W"] = pol.action_net.weight.detach().cpu().numpy()
    arrays["out_b"] = pol.action_net.bias.detach().cpu().numpy()
    # State-independent diagonal Gaussian log-std, so workers can sample a stochastic
    # opponent (see NumpyOpponent.predict). Older snapshots without this stay deterministic.
    if hasattr(pol, "log_std"):
        arrays["log_std"] = pol.log_std.detach().cpu().numpy()
    np.savez(path, **arrays)


def transfer_weights_expand_obs(new_model, old_path: str, device=None) -> tuple[int, int]:
    """Seed ``new_model`` from a checkpoint whose observation space is a *prefix* of the new one.

    Enables curriculum transfer from the single-agent stages (35-dim obs) into SELF_PLAY_1V1
    (39-dim: the same 35 dims + a 4-beam sonar block appended at the end). Copies every policy
    parameter that matches by shape; for the first Linear layer (whose input width grew) copies the
    overlapping input columns and leaves the new sonar columns at their fresh initialisation.
    Returns ``(copied, padded)`` tensor counts. Torch/SB3 imported lazily (trainer process only).
    """
    from stable_baselines3 import PPO

    old = PPO.load(old_path, device=device)
    old_sd = old.policy.state_dict()
    new_sd = new_model.policy.state_dict()
    copied = padded = 0
    for k, nv in new_sd.items():
        ov = old_sd.get(k)
        if ov is None:
            continue
        if ov.shape == nv.shape:
            new_sd[k] = ov.clone()
            copied += 1
        elif (ov.dim() == 2 and nv.dim() == 2 and ov.shape[0] == nv.shape[0]
              and ov.shape[1] < nv.shape[1]):
            tmp = nv.clone()
            tmp[:, :ov.shape[1]] = ov
            new_sd[k] = tmp
            padded += 1
    new_model.policy.load_state_dict(new_sd)
    return copied, padded
