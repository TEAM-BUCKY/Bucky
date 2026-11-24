"""Hand-coded (non-learned) controllers.

These satisfy the same ``predict(obs, deterministic=True) -> (action, _)`` contract as the
learned policies, so they compete in the tournament, act as the imitation-learning expert, and
serve as the parameter vector evolutionary search (CMA-ES/GA) optimizes.
"""
from __future__ import annotations

from bucky.controllers.apf import APFController, APFParams
from bucky.controllers.behaviour_tree import BehaviourTreeController
from bucky.controllers.fsm import FSMController
from bucky.controllers.reactive import ReactiveController, ReactiveParams

# Scripted PWS §9 algorithms, loadable by name (``load_policy("fsm")`` etc.).
SCRIPTED_CONTROLLERS = {
    "fsm": FSMController,
    "apf": APFController,
    "bt": BehaviourTreeController,
}

__all__ = ["APFController", "APFParams", "BehaviourTreeController", "FSMController",
           "ReactiveController", "ReactiveParams", "SCRIPTED_CONTROLLERS"]
