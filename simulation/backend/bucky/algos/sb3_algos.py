"""Stable-Baselines3 algorithm builders, registered into the algo registry.

Each builder shares the uniform signature
``(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef)`` and pulls the
hyperparameters its algorithm understands out of ``params`` (with the same defaults the
trainer used before the registry existed). ``ent_coef`` is the trainer's stage-resolved
entropy coefficient; only the on-policy algos that accept it use it.

Torch/SB3 classes are imported lazily (inside the ``_*_cls`` thunks) so importing this
module never pulls torch into a SubprocVecEnv worker.
"""
from __future__ import annotations

from bucky.algos.registry import AlgoSpec, register


def _hp(params, key, default):
    return params.get(key, default)


# ── PPO (default) — reproduces the historical build_fresh_model() exactly ──────────────
def _ppo_cls():
    from stable_baselines3 import PPO
    return PPO


def _build_ppo(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 3e-4), n_steps=_hp(params, "n_steps", 1024),
        batch_size=_hp(params, "batch_size", 512), n_epochs=_hp(params, "n_epochs", 10),
        gamma=_hp(params, "gamma", 0.99), gae_lambda=_hp(params, "gae_lambda", 0.95),
        clip_range=_hp(params, "clip_range", 0.2), ent_coef=ent_coef,
        vf_coef=_hp(params, "vf_coef", 0.5), max_grad_norm=_hp(params, "max_grad_norm", 0.5),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("ppo", "continuous", True, _ppo_cls, _build_ppo))


# ── A2C — on-policy actor-critic (same policy layout as PPO → self-play capable) ────────
def _a2c_cls():
    from stable_baselines3 import A2C
    return A2C


def _build_a2c(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 7e-4), n_steps=int(_hp(params, "n_steps", 5)),
        gamma=_hp(params, "gamma", 0.99), gae_lambda=_hp(params, "gae_lambda", 1.0),
        ent_coef=ent_coef, vf_coef=_hp(params, "vf_coef", 0.5),
        max_grad_norm=_hp(params, "max_grad_norm", 0.5),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("a2c", "continuous", True, _a2c_cls, _build_a2c))


# ── Off-policy continuous control (replay-buffer actor-critics) ─────────────────────────
# All three export a plain-MLP (ReLU, tanh-squashed) actor → self-play / on-device capable.
# The trainer's stage ``ent_coef`` is on-policy-specific, so these builders ignore it (SAC does
# its own entropy auto-tuning via ``ent_coef="auto"`` unless overridden in algo_params).
def _sac_cls():
    from stable_baselines3 import SAC
    return SAC


def _build_sac(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 3e-4),
        buffer_size=int(_hp(params, "buffer_size", 1_000_000)),
        batch_size=int(_hp(params, "batch_size", 256)),
        tau=_hp(params, "tau", 0.005), gamma=_hp(params, "gamma", 0.99),
        train_freq=int(_hp(params, "train_freq", 1)),
        gradient_steps=int(_hp(params, "gradient_steps", 1)),
        learning_starts=int(_hp(params, "learning_starts", 5_000)),
        ent_coef=_hp(params, "ent_coef", "auto"),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("sac", "continuous", True, _sac_cls, _build_sac))


def _td3_cls():
    from stable_baselines3 import TD3
    return TD3


def _ddpg_cls():
    from stable_baselines3 import DDPG
    return DDPG


def _build_deterministic_offpolicy(cls, *, env, params, net_arch, seed, device,
                                   tensorboard_log, ent_coef):
    """Shared builder for TD3 and DDPG (deterministic-actor off-policy control, no ent_coef)."""
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 1e-3),
        buffer_size=int(_hp(params, "buffer_size", 1_000_000)),
        batch_size=int(_hp(params, "batch_size", 256)),
        tau=_hp(params, "tau", 0.005), gamma=_hp(params, "gamma", 0.99),
        train_freq=int(_hp(params, "train_freq", 1)),
        gradient_steps=int(_hp(params, "gradient_steps", 1)),
        learning_starts=int(_hp(params, "learning_starts", 5_000)),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("td3", "continuous", True, _td3_cls, _build_deterministic_offpolicy))
register(AlgoSpec("ddpg", "continuous", True, _ddpg_cls, _build_deterministic_offpolicy))


# ── DQN — discrete value-based (needs the discrete-action wrapper env) ──────────────────
# Q-network over discrete actions → no continuous actor to export, so not self-play capable; it
# plays in matches/tournaments via bucky.policies.discrete.DiscretePolicyAdapter. SB3's DQN is
# standard DQN with a target network; the Double/Dueling/distributional variants (QR-DQN) arrive
# with sb3-contrib in a later stage.
def _dqn_cls():
    from stable_baselines3 import DQN
    return DQN


def _build_dqn(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 1e-4),
        buffer_size=int(_hp(params, "buffer_size", 100_000)),
        batch_size=int(_hp(params, "batch_size", 128)),
        gamma=_hp(params, "gamma", 0.99),
        train_freq=int(_hp(params, "train_freq", 4)),
        gradient_steps=int(_hp(params, "gradient_steps", 1)),
        learning_starts=int(_hp(params, "learning_starts", 5_000)),
        target_update_interval=int(_hp(params, "target_update_interval", 1_000)),
        exploration_fraction=_hp(params, "exploration_fraction", 0.2),
        exploration_final_eps=_hp(params, "exploration_final_eps", 0.05),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("dqn", "discrete", False, _dqn_cls, _build_dqn, env_wrapper="discrete"))


# ── Fuzzy + RL residual hybrid — RL learns a correction on top of the classical controller ──
# The env is wrapped so the agent's action is a *residual* added to the reactive controller's
# action (see bucky.envs.residual_wrapper). The learner is a standard continuous algo; at match
# time bucky.policies.residual.ResidualPolicyAdapter recombines controller + learned residual.
# Not a plain-MLP opponent (controller + net), so not self-play-pool capable.
register(AlgoSpec("residual_sac", "continuous", False, _sac_cls, _build_sac,
                  env_wrapper="residual"))
register(AlgoSpec("residual_ppo", "continuous", False, _ppo_cls, _build_ppo,
                  env_wrapper="residual"))
