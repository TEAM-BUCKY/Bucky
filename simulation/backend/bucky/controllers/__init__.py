"""Hand-coded (non-learned) controllers.

These satisfy the same ``predict(obs, deterministic=True) -> (action, _)`` contract as the
learned policies, so they compete in the tournament, act as the imitation-learning expert, and
serve as the parameter vector evolutionary search (CMA-ES/GA) optimizes.
"""
from __future__ import annotations

from bucky.controllers.reactive import ReactiveController, ReactiveParams

__all__ = ["ReactiveController", "ReactiveParams"]
