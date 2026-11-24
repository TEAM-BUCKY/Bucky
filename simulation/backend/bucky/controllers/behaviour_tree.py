"""Behaviour Tree controller (PWS §9.2.3).

A behaviour tree is a tree of nodes that is "ticked" from the root every step. Each node
returns SUCCESS, FAILURE or RUNNING.

* **Selector** (fallback, "?"): ticks its children left to right and stops at the first child
  that does not FAIL. It means "try this, otherwise that".
* **Sequence** ("→"): ticks its children left to right and stops at the first child that does
  not SUCCEED. It means "only if all of these".
* **Condition**: a yes/no check on the world (SUCCESS or FAILURE).
* **Action**: writes an action to the blackboard and returns RUNNING.

Unlike the FSM there are no stored states or transitions. Every tick the tree re-weighs every
consideration in priority order, so adding a behaviour means adding a branch rather than
transitions to every existing state. The tree used here:

    ?  Selector
    ├── →  ball out of play?            → retreat
    ├── →  have the ball? → behind it?  → shoot / dribble to goal
    ├── →  ball in our half? → opponent closer? → defend
    └── approach the ball (bisector point, §5)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

import numpy as np

from bucky.controllers import skills
from bucky.controllers.perception import WorldView, decode_obs, in_own_half


class Status(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


@dataclass
class Blackboard:
    view: WorldView
    action: np.ndarray | None = None
    trace: list[str] = field(default_factory=list)   # names of the nodes that fired (debugging)


class Node:
    name = "node"

    def tick(self, bb: Blackboard) -> Status:
        raise NotImplementedError


class Selector(Node):
    def __init__(self, name: str, *children: Node) -> None:
        self.name, self.children = name, children

    def tick(self, bb: Blackboard) -> Status:
        for child in self.children:
            status = child.tick(bb)
            if status != Status.FAILURE:
                return status
        return Status.FAILURE


class Sequence(Node):
    def __init__(self, name: str, *children: Node) -> None:
        self.name, self.children = name, children

    def tick(self, bb: Blackboard) -> Status:
        for child in self.children:
            status = child.tick(bb)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Condition(Node):
    def __init__(self, name: str, check: Callable[[WorldView], bool]) -> None:
        self.name, self.check = name, check

    def tick(self, bb: Blackboard) -> Status:
        return Status.SUCCESS if self.check(bb.view) else Status.FAILURE


class Action(Node):
    def __init__(self, name: str, behaviour: Callable[[WorldView], np.ndarray]) -> None:
        self.name, self.behaviour = name, behaviour

    def tick(self, bb: Blackboard) -> Status:
        bb.action = self.behaviour(bb.view)
        bb.trace.append(self.name)
        return Status.RUNNING


def build_striker_tree() -> Node:
    return Selector(
        "root",
        Sequence("ball-out",
                 Condition("ball out of play?", lambda v: v.ball_out),
                 Action("retreat", skills.act_recover)),
        Sequence("attack",
                 Condition("close to ball?", lambda v: v.ball_dist < 0.25),
                 Condition("behind the ball?", lambda v: skills.behind_ball(v, 40.0)),
                 Action("shoot/dribble", skills.act_attack)),
        Sequence("defend",
                 Condition("ball in own half?", lambda v: in_own_half(v.ball_pos)),
                 Condition("opponent closer?", skills.opponent_closer_to_ball),
                 Action("defend", skills.act_defend)),
        Action("approach", skills.act_approach),
    )


class BehaviourTreeController:
    name = "bt"

    def __init__(self, tree: Node | None = None, obs_dim: int = 43) -> None:
        from gymnasium import spaces
        self.tree = tree or build_striker_tree()
        self.observation_space = spaces.Box(-3.0, 3.0, (obs_dim,), np.float32)
        self.last_trace: list[str] = []

    def predict(self, obs, deterministic: bool = True, **_):
        bb = Blackboard(view=decode_obs(obs))
        self.tree.tick(bb)
        self.last_trace = bb.trace
        act = bb.action if bb.action is not None else skills.action(np.zeros(2), 0.0, False)
        return act, None
