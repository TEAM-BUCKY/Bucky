"""Artificial Potential Field controller (PWS §9.2.2).

Imaginary forces act on the robot, and the robot drives in the direction of their sum:

* attractive force: pulls toward the goal of the moment. That is the bisector point behind the
  ball (§5), or the shooting point once we are behind the ball. It is proportional to distance
  and saturates at 1.
* repulsive forces push away from obstacles that are closer than an influence radius ρ₀:
  - the opponent (sonar estimate),
  - the white lines,
  - the own goal area.
  Each uses the classic Khatib form  F = k·(1/ρ − 1/ρ₀)·(1/ρ²)·n̂.
* local-minimum escape: the PWS names the drawback that attraction and repulsion can cancel
  out, so the robot stops making progress. When the summed force stays below ε for
  ``stall_steps`` steps, a tangential force is added that swings the robot around the obstacle.

The omni drive (§4) means the force can be used directly as the move direction, without
turning first. The heading is controlled separately: face the ball, or the aim point when on
the ball.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from bucky.controllers import skills
from bucky.controllers.perception import OWN_GOAL, WorldView, decode_obs
from bucky.game.field import HALF_H, HALF_W, PENALTY_DEPTH, ROBOT_RADIUS


@dataclass
class APFParams:
    k_attract: float = 3.0          # attractive gain (per metre)
    k_opponent: float = 0.04        # opponent repulsion gain
    rho_opponent: float = 0.40      # m: opponent influence radius (centre-to-centre)
    k_wall: float = 0.004           # white-line repulsion gain
    rho_wall: float = 0.15          # m: line influence radius
    k_own_goal: float = 0.02        # own-goal-area repulsion gain
    rho_own_goal: float = 0.55      # m: own-goal influence radius
    stall_force: float = 0.15       # |F| below this counts as "stuck in a minimum"
    stall_steps: int = 10           # …for this many steps → add the escape force
    k_escape: float = 0.8           # tangential escape force


def repulse(point: np.ndarray, obstacle: np.ndarray, k: float, rho0: float,
            min_rho: float = 0.02) -> np.ndarray:
    """Khatib repulsion from ``obstacle`` acting on ``point`` (0 outside ``rho0``)."""
    d = np.asarray(point, dtype=float) - np.asarray(obstacle, dtype=float)
    rho = max(float(np.linalg.norm(d)), min_rho)
    if rho >= rho0:
        return np.zeros(2)
    return k * (1.0 / rho - 1.0 / rho0) / rho ** 2 * (d / rho)


class APFController:
    name = "apf"

    def __init__(self, params: APFParams | None = None, obs_dim: int = 43) -> None:
        from gymnasium import spaces
        self.params = params or APFParams()
        self.observation_space = spaces.Box(-3.0, 3.0, (obs_dim,), np.float32)
        self._stalled = 0

    def reset(self) -> None:
        self._stalled = 0

    # ── the forces ──────────────────────────────────────────────────────────
    def goal_point(self, view: WorldView) -> np.ndarray:
        aim = skills.aim_point(view)
        if skills.behind_ball(view, 30.0) and view.ball_dist < 0.25:
            return view.ball_pos + 0.3 * skills.unit(aim - view.ball_pos)   # push through the ball
        return skills.approach_point(view, aim=aim)

    def attractive(self, view: WorldView, goal: np.ndarray) -> np.ndarray:
        return skills.clip_norm(self.params.k_attract * (goal - view.robot_pos), 1.0)

    def repulsive(self, view: WorldView) -> np.ndarray:
        p, pos = self.params, view.robot_pos
        f = np.zeros(2)
        if view.opponent_pos is not None:
            f += repulse(pos, view.opponent_pos, p.k_opponent, p.rho_opponent + ROBOT_RADIUS)
        x, y = pos
        for wall in (np.array([HALF_W, y]), np.array([-HALF_W, y]),
                     np.array([x, HALF_H]), np.array([x, -HALF_H])):
            f += repulse(pos, wall, p.k_wall, p.rho_wall)
        # Our own goal area: stay out unless the ball is in it (the robot must not camp there).
        if np.linalg.norm(view.ball_pos - OWN_GOAL) > PENALTY_DEPTH + 0.1:
            f += repulse(pos, OWN_GOAL, p.k_own_goal, p.rho_own_goal)
        return f

    def total_force(self, view: WorldView) -> np.ndarray:
        goal = self.goal_point(view)
        f = self.attractive(view, goal) + self.repulsive(view)
        far_from_goal = np.linalg.norm(goal - view.robot_pos) > 0.05
        self._stalled = self._stalled + 1 if (np.linalg.norm(f) < self.params.stall_force
                                              and far_from_goal) else 0
        if self._stalled >= self.params.stall_steps:
            to_goal = skills.unit(goal - view.robot_pos)
            f = f + self.params.k_escape * np.array([-to_goal[1], to_goal[0]])
        return skills.clip_norm(f, 1.0)

    def predict(self, obs, deterministic: bool = True, **_):
        view = decode_obs(obs)
        if view.ball_out:
            return skills.act_recover(view), None
        f = self.total_force(view)
        on_ball = view.ball_dist < 0.3
        face = skills.aim_point(view) if on_ball and skills.behind_ball(view, 45.0) else view.ball_pos
        return skills.action(view.to_body(f), skills.turn_toward(view, face),
                             skills.should_kick(view)), None
