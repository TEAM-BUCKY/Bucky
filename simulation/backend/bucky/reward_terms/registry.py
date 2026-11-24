"""Extensible reward-term registry (PWS §9.3).

A reward *term* is one named, individually-weighted piece of the reward. Every term is a plain
function ``fn(ctx) -> float`` registered with :func:`reward_term`. Its weight is an integer
(PWS convention §9.3.1: weights are whole multiples of 1, never e.g. 1.5) and the function
carries the sign, so the weight always reads as "how important is this".

Normalisation convention: a term's function is scaled so that

* an **event** term (goal, out of bounds, ...) is ±1 per event, and
* a **shaping** term (distance to ball, smoothness, ...) sums to a value in [-1, 1] over a whole
  episode.

So the weight equals "the maximum number of points this term can give" — the "Max waarde"
column of Bijlage A.

Adding a new weight = write one decorated function (and add its name to a profile). The
config, per-term logging, ``/api/reward-defaults`` and the web UI editor all pick it up.

    @reward_term("my_term", weight=3, group="pws", description="...")
    def my_term(ctx: RewardContext) -> float:
        return ...
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from bucky.physics.backend import PhysicsState


@dataclass
class RewardContext:
    """Everything a term may look at for one environment step."""

    s0: PhysicsState                    # state before the step
    s1: PhysicsState                    # state after the step
    info: dict                          # env event flags (goal_scored, kicked, ...)
    action: np.ndarray | None = None
    prev_action: np.ndarray | None = None
    max_steps: int = 1500               # episode length, for normalising per-step shaping


TermFn = Callable[[RewardContext], float]


@dataclass(frozen=True)
class TermSpec:
    name: str
    weight: int
    group: str            # "pws" (the five PWS weights) | "rule" (RoboCup rule penalties) | ...
    description: str
    fn: TermFn


TERMS: dict[str, TermSpec] = {}

# Which terms make up each reward profile. "legacy" is special-cased in bucky.rewards (the old
# hand-tuned reward lives on unchanged in bucky.reward_terms.legacy).
PROFILES: dict[str, list[str]] = {"pws": []}


def check_weight(name: str, value) -> int:
    """Validate a weight against the PWS convention: a whole number (``10`` or ``10.0``)."""
    as_float = float(value)
    if not as_float.is_integer():
        raise ValueError(
            f"reward weight {name!r}={value!r} is not a whole number; PWS §9.3.1 requires "
            f"integer weights (put fractional scaling inside the term's formula instead)")
    return int(as_float)


def reward_term(name: str, *, weight: int, group: str, description: str = "",
                profiles: tuple[str, ...] = ("pws",)) -> Callable[[TermFn], TermFn]:
    """Register ``fn`` as reward term ``name`` with default integer ``weight``."""
    check_weight(name, weight)

    def deco(fn: TermFn) -> TermFn:
        if name in TERMS:
            raise ValueError(f"reward term {name!r} is already registered")
        TERMS[name] = TermSpec(name, int(weight), group, description or (fn.__doc__ or "").strip(), fn)
        for p in profiles:
            PROFILES.setdefault(p, []).append(name)
        return fn

    return deco


def unregister(name: str) -> None:
    """Remove a term (mainly for tests that register throw-away terms)."""
    TERMS.pop(name, None)
    for names in PROFILES.values():
        if name in names:
            names.remove(name)


def profile_terms(profile: str) -> list[TermSpec]:
    if profile not in PROFILES:
        raise ValueError(f"unknown reward profile {profile!r} (known: {sorted(PROFILES) + ['legacy']})")
    return [TERMS[n] for n in PROFILES[profile]]
