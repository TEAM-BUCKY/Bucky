"""SB3 callbacks that stream live state and training scalars to the viz hub.

Both run on the same thread as ``model.learn()``; they only ever push JSON onto the
``StreamClient``'s thread-safe queue (never block, never touch the network directly).
"""
from __future__ import annotations
from collections import deque

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.utils import safe_mean

from bucky.stream_client import StreamClient


class LiveVizCallback(BaseCallback):
    """Continuously stream a live-policy rollout, independent of the eval schedule.

    Holds its own in-process ``BuckySingleEnv`` (a "shadow" env). Every ``every``
    steps it predicts an action with the current policy, advances the shadow env one
    step, and streams a ``step`` frame with the real 18-dim observation and reward
    terms. This keeps the field animating during the whole run rather than only
    during periodic evaluation.
    """

    def __init__(
        self,
        stream: StreamClient,
        stage: str,
        every: int = 2,
        domain_rand: bool = False,
        verbose: int = 0,
    ) -> None:
        super().__init__(verbose)
        self._stream = stream
        self._stage = stage
        self._every = max(1, every)
        self._domain_rand = domain_rand
        self._env = None
        self._obs = None
        self._captured = None  # (PhysicsState, RewardTerms) from the shadow env's step
        self._episode = 0
        self._step = 0
        self._total_return = 0.0

    def _init_callback(self) -> None:
        from bucky.envs.bucky_single import BuckySingleEnv

        def capture(state, terms):
            self._captured = (state, terms)

        self._env = BuckySingleEnv(
            stage=self._stage, domain_rand=self._domain_rand, viz_callback=capture
        )
        self._obs, _ = self._env.reset()

    def _on_step(self) -> bool:
        if self.n_calls % self._every != 0:
            return True

        action, _ = self.model.predict(self._obs, deterministic=False)
        obs, reward, terminated, truncated, _info = self._env.step(action)
        state, terms = self._captured

        self._step += 1
        self._total_return += float(reward)

        self._stream.send({
            "type": "step",
            "robot_pos": state.robot_pos.tolist(),
            "robot_heading": float(state.robot_heading),
            "ball_pos": state.ball_pos.tolist(),
            "reward_terms": terms.as_dict(),
            "reward_total": float(terms.total),
            "obs": [float(x) for x in obs],
            "episode": self._episode,
            "step": self._step,
            "total_return": self._total_return,
            "num_timesteps": int(self.num_timesteps),
        })

        self._obs = obs
        if terminated or truncated:
            self._episode += 1
            self._step = 0
            self._total_return = 0.0
            self._obs, _ = self._env.reset()
        return True


class SelfPlaySnapshotCallback(BaseCallback):
    """Periodically freeze the current policy into a rolling *pool* of self-play opponents.

    Every ``every`` callbacks it dumps the live policy to a versioned numpy snapshot and
    hands every worker the current pool (via ``env_method("set_opponent_pool", ...)``), so
    each episode the learner faces a *random* recent copy of itself rather than only the
    latest one. Facing a mix of past snapshots breaks the symmetric "perfectly counter each
    other" collapse and the cyclic counter-the-counter dynamic of single-snapshot self-play.
    Snapshot files round-robin across ``pool_size`` slots so disk stays bounded.
    """

    def __init__(self, snapshot_base: str, every: int = 200, pool_size: int = 5,
                 verbose: int = 0) -> None:
        super().__init__(verbose)
        self._base = snapshot_base
        self._npz = snapshot_base + ".npz"        # canonical single-file snapshot (viz/back-compat)
        self._every = max(1, every)
        self._pool_size = max(1, pool_size)
        self._count = 0
        self._paths: deque[str] = deque(maxlen=self._pool_size)

    def _on_step(self) -> bool:
        if self.n_calls % self._every == 0:
            from bucky.selfplay import export_policy_npz
            self.model.save(self._base)              # resumable .zip checkpoint
            export_policy_npz(self.model, self._npz)  # canonical snapshot (used by the viz env)
            # Versioned snapshot in a round-robin slot → numpy weights the workers reload.
            path = f"{self._base}_{self._count % self._pool_size}.npz"
            export_policy_npz(self.model, path)
            if path not in self._paths:
                self._paths.append(path)
            self._count += 1
            try:
                self.training_env.env_method("set_opponent_pool", list(self._paths))
            except Exception:  # noqa: BLE001 — best effort; workers keep the old pool
                pass
        return True


class SelfPlayVizCallback(BaseCallback):
    """Stream a live 1v1 rollout (both robots) during self-play training.

    Mirror of :class:`LiveVizCallback` but holds a shadow :class:`BuckySelfPlayEnv` and
    emits ``robot2_*`` so the field shows the opponent too. Reloads the opponent snapshot
    at each episode boundary to track the latest frozen policy.
    """

    def __init__(self, stream: StreamClient, opponent_path: str | None = None,
                 every: int = 2, verbose: int = 0) -> None:
        super().__init__(verbose)
        self._stream = stream
        self._opponent_path = opponent_path
        self._every = max(1, every)
        self._env = None
        self._obs = None
        self._episode = 0
        self._step = 0
        self._total_return = 0.0

    def _init_callback(self) -> None:
        from bucky.envs.bucky_selfplay import BuckySelfPlayEnv

        self._env = BuckySelfPlayEnv(domain_rand=False)
        if self._opponent_path:
            self._env.set_opponent(self._opponent_path)
        self._obs, _ = self._env.reset()

    def _on_step(self) -> bool:
        if self.n_calls % self._every != 0:
            return True

        action, _ = self.model.predict(self._obs, deterministic=False)
        obs, reward, terminated, truncated, _info = self._env.step(action)
        sa = self._env.physics.state_a()
        sb = self._env.physics.state_b()
        terms = self._env.last_terms

        self._step += 1
        self._total_return += float(reward)

        self._stream.send({
            "type": "step",
            "mode": "play",
            "robot_pos": sa.robot_pos.tolist(),
            "robot_heading": float(sa.robot_heading),
            "robot2_pos": sb.robot_pos.tolist(),
            "robot2_heading": float(sb.robot_heading),
            "ball_pos": sa.ball_pos.tolist(),
            "reward_terms": terms.as_dict(),
            "reward_total": float(terms.total),
            "obs": [float(x) for x in obs],
            "episode": self._episode,
            "step": self._step,
            "total_return": self._total_return,
            "num_timesteps": int(self.num_timesteps),
        })

        self._obs = obs
        if terminated or truncated:
            self._episode += 1
            self._step = 0
            self._total_return = 0.0
            if self._opponent_path:
                self._env.set_opponent(self._opponent_path)   # track latest snapshot
            self._obs, _ = self._env.reset()
        return True


class MetricsCallback(BaseCallback):
    """Stream PPO training scalars (loss, KL, entropy, …) once per update.

    Read at ``_on_rollout_start`` — the window after ``train()`` has populated the
    ``train/*`` logger values and before the next ``logger.dump()`` clears them.
    ``ep_rew_mean`` is derived from ``ep_info_buffer`` so it doesn't depend on the
    SB3 log interval.
    """

    _TRAIN_KEYS = (
        "train/loss",
        "train/approx_kl",
        "train/entropy_loss",
        "train/value_loss",
        "train/policy_gradient_loss",
        "train/clip_fraction",
    )

    def __init__(self, stream: StreamClient, verbose: int = 0) -> None:
        super().__init__(verbose)
        self._stream = stream

    def _on_rollout_start(self) -> None:
        nv = self.model.logger.name_to_value
        values = {k: float(nv[k]) for k in self._TRAIN_KEYS if k in nv}

        buf = self.model.ep_info_buffer
        if buf and len(buf) > 0:
            values["rollout/ep_rew_mean"] = float(safe_mean([e["r"] for e in buf]))
            values["rollout/ep_len_mean"] = float(safe_mean([e["l"] for e in buf]))

        if not values:
            return  # nothing logged yet (before the first train() call)

        self._stream.send({
            "type": "metrics",
            "num_timesteps": int(self.num_timesteps),
            "iteration": int(getattr(self.model, "_n_updates", 0)),
            "values": values,
        })

    def _on_step(self) -> bool:
        return True
