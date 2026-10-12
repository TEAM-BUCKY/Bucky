"""Self-play / 1v1 helpers: x-axis mirror trick, sonar sensing, opponent-aware obs.

The single-agent policy is trained to attack the **+x** goal. Robot B attacks **−x**, so
to drive B with the same policy we reflect the world across the x-axis (B then "sees" itself
attacking +x), run the policy, and un-mirror the resulting action. Opponent perception is via
a 4-beam sonar model (the real robot only has 4 ultrasonic sensors at 90° spacing), appended
to the 23-dim single-agent observation → a 27-dim opponent-aware observation.
"""
from __future__ import annotations
import numpy as np

from bucky.game.geometry import ray_to_arena
from bucky.obs import (
    LEGACY_OBS_DIM,
    OBS_DIM,
    PRE_GOAL_REL_OBS_DIM,
    PRE_KICK_PRED_OBS_DIM,
    build_observation,
)
from bucky.physics.backend import PhysicsState
from bucky.physics.python_backend import ROBOT_RADIUS

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
    return ray_to_arena(pos, direction, MAX_SONAR_RANGE)


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

def _relu(z: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, z)


class NumpyOpponent:
    """A frozen SB3 MLP actor evaluated in pure numpy.

    Replicates SB3's deterministic action for the actor architectures the trainer produces:
      * **On-policy** (PPO/A2C/TRPO): the un-squashed Gaussian mean
        ``action_net(policy_net(obs))`` with ``tanh`` hidden activations, clipped to ``[-1, 1]``.
      * **Off-policy** (SAC/TD3/DDPG): the ``tanh``-squashed deterministic action, with ``relu``
        hidden activations by default (``squash=True``).
    ``activation`` (``"tanh"``/``"relu"``) selects the hidden non-linearity and ``squash`` applies
    a final ``tanh`` to the output. Valid for identity (Flatten) features, no ``VecNormalize``, and
    separate (un-shared) actor layers — which is what :mod:`scripts.train` builds.
    """

    def __init__(self, hidden, out_w: np.ndarray, out_b: np.ndarray,
                 log_std: np.ndarray | None = None, *, activation: str = "tanh",
                 squash: bool = False) -> None:
        self._hidden = hidden          # list of (W, b) layers, applied in order
        self._out_w = out_w
        self._out_b = out_b
        self._activation = activation
        self._squash = bool(squash)
        # The policy's expected input width (first hidden layer's column count). If it's narrower
        # than the current self-play obs, predict() projects the obs down (adapt_obs_to_policy) so a
        # LEGACY-dim model (e.g. 39-dim) can serve as a training-pool opponent against a 43-dim env.
        self._in_dim = int(hidden[0][0].shape[1]) if hidden else None
        # Per-dim Gaussian log-std (SB3's state-independent ``policy.log_std``). ``None`` for
        # legacy .npz snapshots without it → the opponent stays deterministic (mean action).
        self._log_std = None if log_std is None else np.asarray(log_std, dtype=np.float32).reshape(-1)

    def predict(self, obs, deterministic: bool = True, rng=None, temperature: float = 1.0):
        """Mirror of ``model.predict`` — returns ``(action, None)`` so it is a drop-in for
        :func:`predict_opponent_action`.

        ``deterministic=True`` returns the policy's deterministic action (matching ``.predict``).
        With ``deterministic=False`` and a stored ``log_std`` (on-policy only) the action is
        *sampled* from the pre-squash diagonal Gaussian — ``mean + temperature * exp(log_std) *
        N(0, 1)`` — so the frozen opponent explores instead of replaying one memorizable pattern.
        Without a ``log_std`` (off-policy or a legacy snapshot) it stays deterministic.
        """
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if self._in_dim is not None and x.shape[0] != self._in_dim:
            x = adapt_obs_to_policy(x, self._in_dim)   # project 43-dim obs → this policy's layout
        act = np.tanh if self._activation == "tanh" else _relu
        for w, b in self._hidden:
            x = act(w @ x + b)
        pre = self._out_w @ x + self._out_b
        if not deterministic and self._log_std is not None:
            if rng is None:
                rng = np.random.default_rng()
            noise = rng.standard_normal(pre.shape[0]).astype(np.float32)
            pre = pre + temperature * np.exp(self._log_std) * noise
        mean = np.tanh(pre) if self._squash else pre
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
    # activation/squash are absent in legacy (pre-off-policy) snapshots → default to the PPO
    # layout (tanh hidden, no squash) so old opponents load and behave identically.
    activation = str(data["activation"]) if "activation" in data.files else "tanh"
    squash = bool(data["squash"]) if "squash" in data.files else False
    return NumpyOpponent(hidden, data["out_W"].astype(np.float32),
                         data["out_b"].astype(np.float32), log_std=log_std,
                         activation=activation, squash=squash)


def _detect_activation(seq) -> str:
    """Return ``"relu"`` or ``"tanh"`` for the hidden non-linearity of a torch Sequential."""
    import torch.nn as nn  # lazy: keep torch out of worker imports
    for m in seq:
        if isinstance(m, nn.ReLU):
            return "relu"
        if isinstance(m, nn.Tanh):
            return "tanh"
    return "tanh"


def _extract_actor_mlp(model):
    """Extract the deterministic actor-mean MLP from a supported SB3 policy.

    Returns ``(hidden_linears, out_linear, activation, squash, log_std)``. Handles the two
    families the trainer produces:
      * on-policy ``ActorCriticPolicy`` (PPO/A2C/TRPO): ``policy_net`` + ``action_net``, no squash;
      * off-policy actors — SAC (``actor.latent_pi`` + ``actor.mu``, tanh-squashed) and
        TD3/DDPG (``actor.mu`` is a Sequential ending in ``Tanh``; its last Linear is the head).
    Torch imported lazily (trainer process only).
    """
    import torch.nn as nn  # lazy: keep torch out of worker imports

    pol = model.policy
    # On-policy: separate policy MLP + a linear action head; un-squashed Gaussian mean.
    if hasattr(pol, "action_net") and hasattr(pol, "mlp_extractor"):
        hidden = [m for m in pol.mlp_extractor.policy_net if isinstance(m, nn.Linear)]
        activation = _detect_activation(pol.mlp_extractor.policy_net)
        log_std = (pol.log_std.detach().cpu().numpy() if hasattr(pol, "log_std") else None)
        return hidden, pol.action_net, activation, False, log_std

    actor = getattr(pol, "actor", None)
    if actor is not None and hasattr(actor, "latent_pi") and hasattr(actor, "mu"):
        # SAC: deterministic action = tanh(mu(latent_pi(obs))). log_std is state-dependent, so the
        # frozen opponent runs deterministically (no stored constant log_std).
        hidden = [m for m in actor.latent_pi if isinstance(m, nn.Linear)]
        return hidden, actor.mu, _detect_activation(actor.latent_pi), True, None
    if actor is not None and hasattr(actor, "mu"):
        # TD3/DDPG: actor.mu is Sequential[Linear, ReLU, ..., Linear, Tanh]. The trailing Tanh is
        # the squash; the last Linear is the output head, the rest are hidden layers.
        linears = [m for m in actor.mu if isinstance(m, nn.Linear)]
        if not linears:
            raise TypeError("TD3/DDPG actor.mu has no Linear layers")
        return linears[:-1], linears[-1], _detect_activation(actor.mu), True, None

    raise TypeError(f"cannot extract a numpy actor MLP from policy {type(pol).__name__}")


def export_policy_npz(model, path: str) -> None:
    """Dump an SB3 actor's deterministic-action weights to ``path`` (.npz) so the workers can
    drive the frozen opponent in numpy. Supports on-policy (PPO/A2C/TRPO) and off-policy
    (SAC/TD3/DDPG) actors — see :func:`_extract_actor_mlp`. Runs in the trainer (torch) process
    only; torch is imported lazily so importing this module in a worker never pulls it in.
    """
    hidden, out, activation, squash, log_std = _extract_actor_mlp(model)
    arrays: dict[str, np.ndarray] = {
        "n_hidden": np.array(len(hidden)),
        "activation": np.array(activation),
        "squash": np.array(bool(squash)),
    }
    for i, m in enumerate(hidden):
        arrays[f"h{i}_W"] = m.weight.detach().cpu().numpy()
        arrays[f"h{i}_b"] = m.bias.detach().cpu().numpy()
    arrays["out_W"] = out.weight.detach().cpu().numpy()
    arrays["out_b"] = out.bias.detach().cpu().numpy()
    # State-independent diagonal Gaussian log-std (on-policy only), so workers can sample a
    # stochastic opponent (see NumpyOpponent.predict). Off-policy/legacy snapshots omit it.
    if log_std is not None:
        arrays["log_std"] = log_std
    np.savez(path, **arrays)


def transfer_weights_expand_obs(new_model, old_path: str, device=None) -> tuple[int, int]:
    """Seed ``new_model`` from a checkpoint whose observation space is a *prefix* of the new one.

    Enables curriculum transfer from the single-agent stages (35-dim obs) into SELF_PLAY_1V1
    (39-dim: the same 35 dims + a 4-beam sonar block appended at the end). Copies every policy
    parameter that matches by shape; for the first Linear layer (whose input width grew) copies the
    overlapping input columns and leaves the new sonar columns at their fresh initialisation.
    Returns ``(copied, padded)`` tensor counts. Torch/SB3 imported lazily (trainer process only).
    The old checkpoint is loaded with the *same* algorithm class as ``new_model`` (curriculum
    transfer keeps the algo fixed across phases), so this works for any registered algo.
    """
    old = type(new_model).load(old_path, device=device)
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
