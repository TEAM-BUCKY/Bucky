"""Finite State Machine striker (PWS §9.2.1), a Mealy machine.

The robot is in exactly one *state* at a time. Each step:

1. The *inputs* are read from the observation (:class:`Inputs`).
2. The *transition table* is checked top to bottom, and the first row whose state matches and
   whose condition holds picks the next state.
3. The *output* (the action) is produced by the state's behaviour from the state **and** the
   current input. This makes it a Mealy machine: in ATTACK the output changes (kick / no kick)
   without the state changing.

The whole machine is the table below, so it can be drawn on paper:

    state      condition                                   → next state
    ─────────  ──────────────────────────────────────────  ───────────
    any        ball out of play                            → RECOVER
    RECOVER    ball back in play                           → APPROACH
    APPROACH   ball in own half and opponent closer to it  → DEFEND
    ATTACK     ball in own half and opponent closer to it  → DEFEND
    DEFEND     not (ball in own half and opponent closer)  → APPROACH
    APPROACH   behind the ball and close to it             → ATTACK
    ATTACK     lost the ball (far, or not behind it)       → APPROACH
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable

import numpy as np

from bucky.controllers import skills
from bucky.controllers.perception import WorldView, decode_obs, in_own_half


class State(str, Enum):
    APPROACH = "APPROACH"
    ATTACK = "ATTACK"
    DEFEND = "DEFEND"
    RECOVER = "RECOVER"


STATES: tuple[State, ...] = tuple(State)
ATTACK_ENTER_DIST = 0.20      # m: close enough (and behind the ball) to start attacking
ATTACK_EXIT_DIST = 0.32       # m: farther than this → we lost it (hysteresis vs ENTER)


@dataclass
class Inputs:
    """The FSM's inputs: yes/no facts about the world, read once per step."""

    view: WorldView
    ball_out: bool
    ball_dist: float
    behind_ball: bool          # tight cone, used for entering ATTACK
    roughly_behind: bool       # wide cone, used for staying in ATTACK (hysteresis)
    threat: bool               # ball in our half and the opponent is closer to it

    @classmethod
    def read(cls, view: WorldView) -> "Inputs":
        return cls(
            view=view,
            ball_out=view.ball_out,
            ball_dist=view.ball_dist,
            behind_ball=skills.behind_ball(view, 30.0),
            roughly_behind=skills.behind_ball(view, 65.0),
            threat=in_own_half(view.ball_pos) and skills.opponent_closer_to_ball(view),
        )


ANY = None   # wildcard "from" state
Condition = Callable[[Inputs], bool]

TRANSITIONS: list[tuple[State | None, Condition, State]] = [
    (ANY,            lambda i: i.ball_out,                                         State.RECOVER),
    (State.RECOVER,  lambda i: not i.ball_out,                                     State.APPROACH),
    (State.APPROACH, lambda i: i.threat,                                           State.DEFEND),
    (State.ATTACK,   lambda i: i.threat,                                           State.DEFEND),
    (State.DEFEND,   lambda i: not i.threat,                                       State.APPROACH),
    (State.APPROACH, lambda i: i.behind_ball and i.ball_dist < ATTACK_ENTER_DIST,  State.ATTACK),
    (State.ATTACK,   lambda i: not i.roughly_behind or i.ball_dist > ATTACK_EXIT_DIST, State.APPROACH),
]

# Output per state. Each behaviour also reads the input (view), which is what makes it Mealy.
OUTPUTS: dict[State, Callable[[WorldView], np.ndarray]] = {
    State.APPROACH: skills.act_approach,
    State.ATTACK: skills.act_attack,
    State.DEFEND: skills.act_defend,
    State.RECOVER: skills.act_recover,
}


def next_state(state: State, inputs: Inputs) -> State:
    for frm, cond, to in TRANSITIONS:
        if (frm is ANY or frm == state) and to != state and cond(inputs):
            return to
    return state


class FSMController:
    """Mealy-FSM striker with the ``predict`` contract of a learned policy."""

    name = "fsm"

    def __init__(self, obs_dim: int = 43) -> None:
        from gymnasium import spaces
        self.observation_space = spaces.Box(-3.0, 3.0, (obs_dim,), np.float32)
        self.state = State.APPROACH

    def reset(self) -> None:
        self.state = State.APPROACH

    def step_state(self, obs) -> tuple[State, Inputs]:
        """Advance the machine on ``obs`` (no action). Used by the FSM+RL hybrid."""
        inputs = Inputs.read(decode_obs(obs))
        self.state = next_state(self.state, inputs)
        return self.state, inputs

    def predict(self, obs, deterministic: bool = True, **_):
        state, inputs = self.step_state(obs)
        return OUTPUTS[state](inputs.view), None
