"""Algorithm registry: decouples the trainer from any single RL library or algorithm.

The trainer historically hardcoded Stable-Baselines3 ``PPO``. This registry makes the
algorithm a config field (``ModelConfig.algo``) so many learning approaches can share the
same env, curriculum, self-play, eval and tournament infrastructure — they only differ in
how the policy is built/trained and how its actor weights are extracted.

Each algorithm is described by an :class:`AlgoSpec`:
  * ``action_kind`` — ``"continuous"`` (native Box(4) env) or ``"discrete"`` (needs the
    discrete-action wrapper env).
  * ``supports_selfplay`` — whether it exports a plain-MLP numpy opponent and can play in
    the self-play pool / on-device.
  * ``build`` — construct a fresh model with that algo's hyperparameters.
  * ``load`` — load a saved checkpoint of that algo.

PPO is the default and its builder reproduces the exact model the trainer built before this
registry existed, so existing PPO runs are byte-for-byte unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

# A builder receives the algo class + resolved knobs and returns a fresh SB3(-contrib) model.
Builder = Callable[..., Any]


@dataclass(frozen=True)
class AlgoSpec:
    name: str
    action_kind: str            # "continuous" | "discrete"
    supports_selfplay: bool     # can export a numpy MLP opponent + play in the self-play pool
    _cls: Callable[[], Any]     # lazy import → the SB3(-contrib) algorithm class
    _build: Builder
    # Optional env transform the trainer applies: "discrete" (Box(4)→Discrete for DQN) or
    # "residual" (RL learns a residual on top of the classical controller — the fuzzy+RL hybrid).
    env_wrapper: str | None = None

    @property
    def cls(self) -> Any:
        """The underlying SB3(-contrib) algorithm class (imported lazily so importing this
        module never pulls torch into a SubprocVecEnv worker)."""
        return self._cls()

    def build(self, **kwargs) -> Any:
        """Construct a fresh model. Every builder accepts the uniform kwarg set
        ``env, params, net_arch, seed, device, tensorboard_log, ent_coef`` and uses what
        its algorithm needs (ignoring the rest)."""
        return self._build(self.cls, **kwargs)

    def load(self, path: str, *, env=None, device=None, tensorboard_log=None) -> Any:
        kwargs: dict[str, Any] = {}
        if env is not None:
            kwargs["env"] = env
        if device is not None:
            kwargs["device"] = device
        if tensorboard_log is not None:
            kwargs["tensorboard_log"] = tensorboard_log
        return self.cls.load(path, **kwargs)


_REGISTRY: dict[str, AlgoSpec] = {}


def register(spec: AlgoSpec) -> AlgoSpec:
    _REGISTRY[spec.name] = spec
    return spec


def get_spec(algo: str | None) -> AlgoSpec:
    key = (algo or "ppo").lower()
    if key not in _REGISTRY:
        raise KeyError(f"unknown algo {algo!r}; registered: {sorted(_REGISTRY)}")
    return _REGISTRY[key]


def available() -> list[str]:
    return sorted(_REGISTRY)
