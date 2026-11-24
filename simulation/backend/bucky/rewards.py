"""Reward computation for Bucky — public entry point.

The default reward is the PWS reward (§9.3): five integer-weighted terms (goal, ball_distance,
ball_position, kick_goal, smoothness) plus the RoboCup rule penalties (goal_against,
out_of_bounds, defective, lack_of_progress).
- The terms live in ``bucky/reward_terms/`` and are registered in an extensible registry, so
  adding a weight means writing one decorated function. See ``bucky/reward_terms/registry.py``.
- The old ~37-weight hand-tuned reward is not deleted. It is the ``legacy`` profile
  (``bucky/reward_terms/legacy.py``), switched on with ``reward_profile: "legacy"``.

All terms are individually logged: ``info["reward_terms"]`` carries ``terms.as_dict()``.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from bucky.physics.backend import PhysicsState
from bucky.reward_terms import PROFILES, RewardContext, check_weight, profile_terms
from bucky.reward_terms.legacy import (  # noqa: F401  (constants re-exported for env/test imports)
    ALIGN_RADIUS, CAPTURE_RADIUS, OPP_GOAL, OWN_GOAL, SHOT_SPEED_THRESHOLD,
    LegacyRewardConfig, LegacyRewardTerms, _shot_enters_goal_mouth, compute_legacy_rewards,
)

DEFAULT_PROFILE = "pws"
LEGACY_PROFILE = "legacy"
DEFAULT_MAX_STEPS = 1500


def available_profiles() -> list[str]:
    return sorted(PROFILES) + [LEGACY_PROFILE]


class RewardConfig:
    """Reward weights for one profile.

    ``pws`` profile: one integer weight per registered term, exposed as ``w_<term>``. Weights
    that are not whole numbers are rejected (PWS §9.3.1).
    ``legacy`` profile: wraps the old float-weighted :class:`LegacyRewardConfig` unchanged.
    """

    def __init__(self, profile: str = DEFAULT_PROFILE, weights: dict[str, Any] | None = None,
                 predicted_goal_horizon_steps: int = 75,
                 legacy: LegacyRewardConfig | None = None) -> None:
        self.profile = profile
        if profile == LEGACY_PROFILE:
            self.legacy = legacy or LegacyRewardConfig()
            self._weights: dict[str, int] = {}
            return
        self.legacy = None
        self._weights = {spec.name: spec.weight for spec in profile_terms(profile)}
        for key, value in (weights or {}).items():
            name = key[2:] if key.startswith("w_") else key
            if name in self._weights:
                self._weights[name] = check_weight(key, value)
        self._horizon = int(predicted_goal_horizon_steps)

    @classmethod
    def from_dict(cls, data: dict | None, profile: str | None = None) -> "RewardConfig":
        """Build from a (partial) ``{"w_<term>": weight}`` mapping, ignoring unknown keys.

        Unknown keys are ignored, so model configs saved under another profile (for example
        the old legacy float weights) still load.
        """
        profile = profile or DEFAULT_PROFILE
        if profile == LEGACY_PROFILE:
            return cls(profile, legacy=LegacyRewardConfig.from_dict(data))
        return cls(profile, weights=data)

    @property
    def uses_predicted_goal(self) -> bool:
        """Only the legacy reward pays the look-ahead "predicted goal" (and then suppresses the
        real goal so it isn't paid twice). The PWS reward pays real goals only."""
        return self.profile == LEGACY_PROFILE

    @property
    def predicted_goal_horizon_steps(self) -> int:
        if self.legacy is not None:
            return int(self.legacy.predicted_goal_horizon_steps)
        return self._horizon

    def weight(self, term: str) -> int:
        if term not in self._weights:      # term registered after this config was built
            from bucky.reward_terms import TERMS
            return TERMS[term].weight
        return self._weights[term]

    def weight_dict(self) -> dict[str, float]:
        """``{"w_<term>": weight}``: the shape the web UI edits and model configs store."""
        if self.legacy is not None:
            from dataclasses import fields
            return {f.name: getattr(self.legacy, f.name) for f in fields(self.legacy)}
        return {f"w_{name}": w for name, w in self._weights.items()}

    def __getattr__(self, name: str):
        # Only reached for attributes not set in __init__, e.g. ``cfg.w_goal``.
        if name.startswith("w_"):
            weights = self.__dict__.get("_weights", {})
            if name[2:] in weights:
                return weights[name[2:]]
            legacy = self.__dict__.get("legacy")
            if legacy is not None and hasattr(legacy, name):
                return getattr(legacy, name)
        raise AttributeError(name)

    def __repr__(self) -> str:
        return f"RewardConfig(profile={self.profile!r}, {self.weight_dict()})"


class RewardTerms:
    """Weighted value of every term for one step (dict-backed, so any number of terms)."""

    def __init__(self, values: dict[str, float] | None = None) -> None:
        self._values = dict(values or {})

    @property
    def total(self) -> float:
        return float(sum(self._values.values()))

    def as_dict(self) -> dict[str, float]:
        return dict(self._values)

    def __getattr__(self, name: str) -> float:
        values = self.__dict__.get("_values", {})
        if name in values:
            return values[name]
        raise AttributeError(name)


def compute_rewards(
    s0: PhysicsState,
    s1: PhysicsState,
    config: RewardConfig,
    info: dict,
    action: np.ndarray | None = None,
    prev_action: np.ndarray | None = None,
):
    """Per-term rewards for one step. ``config.profile`` selects PWS (default) or legacy."""
    if isinstance(config, LegacyRewardConfig):
        return compute_legacy_rewards(s0, s1, config, info, action, prev_action)
    if config.profile == LEGACY_PROFILE:
        return compute_legacy_rewards(s0, s1, config.legacy, info, action, prev_action)
    ctx = RewardContext(s0, s1, info, action, prev_action,
                        max_steps=int(info.get("max_steps", DEFAULT_MAX_STEPS)))
    return RewardTerms({spec.name: config.weight(spec.name) * float(spec.fn(ctx))
                        for spec in profile_terms(config.profile)})
