"""sb3-contrib algorithm builders (TQC, TRPO, ARS, QR-DQN), registered into the registry.

  * **TQC** — Truncated Quantile Critics: off-policy continuous control with a SAC-style
    tanh-squashed actor → exports a numpy opponent (self-play capable).
  * **TRPO** — Trust Region Policy Optimization: on-policy ActorCriticPolicy like PPO → self-play.
  * **ARS** — Augmented Random Search: an evolutionary-strategy baseline over an MLP policy.
  * **QR-DQN** — distributional (quantile) DQN: the practical Rainbow-flavoured stand-in for the
    discrete value-based family (SB3 does not expose separate Double/Dueling toggles).
"""
from __future__ import annotations

from bucky.algos.registry import AlgoSpec, register
from bucky.algos.sb3_algos import _hp


def _tqc_cls():
    from sb3_contrib import TQC
    return TQC


def _build_tqc(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
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


register(AlgoSpec("tqc", "continuous", True, _tqc_cls, _build_tqc))


def _trpo_cls():
    from sb3_contrib import TRPO
    return TRPO


def _build_trpo(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        learning_rate=_hp(params, "learning_rate", 1e-3),
        n_steps=int(_hp(params, "n_steps", 1024)),
        batch_size=int(_hp(params, "batch_size", 128)),
        gamma=_hp(params, "gamma", 0.99), gae_lambda=_hp(params, "gae_lambda", 0.95),
        target_kl=_hp(params, "target_kl", 0.01),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


register(AlgoSpec("trpo", "continuous", True, _trpo_cls, _build_trpo))


def _ars_cls():
    from sb3_contrib import ARS
    return ARS


def _build_ars(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
    n_delta = int(_hp(params, "n_delta", 8))
    return cls(
        policy="MlpPolicy", env=env, verbose=1, tensorboard_log=tensorboard_log, seed=seed,
        n_delta=n_delta, n_top=min(int(_hp(params, "n_top", n_delta)), n_delta),
        learning_rate=_hp(params, "learning_rate", 0.02),
        delta_std=_hp(params, "delta_std", 0.05),
        policy_kwargs={"net_arch": list(net_arch)}, device=device,
    )


# ARS's policy layout differs from the SB3 actor-critic, so it doesn't export a numpy opponent;
# it still competes in matches/tournaments via its SB3 predict.
register(AlgoSpec("ars", "continuous", False, _ars_cls, _build_ars))


def _qrdqn_cls():
    from sb3_contrib import QRDQN
    return QRDQN


def _build_qrdqn(cls, *, env, params, net_arch, seed, device, tensorboard_log, ent_coef):
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


register(AlgoSpec("qrdqn", "discrete", False, _qrdqn_cls, _build_qrdqn, env_wrapper="discrete"))
