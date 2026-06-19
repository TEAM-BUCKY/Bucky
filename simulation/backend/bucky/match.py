"""1v1 match engine: steps two policies against each other under the full rule set.

Pure game logic (no I/O) so it can be unit-tested. ``scripts/play.py`` wraps this in a
real-time loop and streams each :meth:`MatchEngine.tick` frame to the viz hub.

Robot A runs its policy on the normal observation; robot B runs its policy via the
x-axis mirror trick (:mod:`bucky.selfplay`). A :class:`~bucky.referee.Referee` in ``match_mode``
governs the match: two 7-minute halves, scoring + conceding-team kickoff, ball
relocation (out-of-reach / lack-of-progress), and robot suspensions/defective removals.
"""
from __future__ import annotations

import numpy as np

from bucky.physics.python_backend import TwoRobotPhysics
from bucky.referee import Referee
from bucky.selfplay import build_robot_obs, predict_opponent_action


class MatchEngine:
    def __init__(self, model_a, model_b, seed: int = 0, domain_rand: bool = False,
                 match_mode: bool = True, first_kickoff: str = "a") -> None:
        self._model_a = model_a
        self._model_b = model_b
        self._dr = domain_rand
        self._rng = np.random.default_rng(seed)
        self._phys = TwoRobotPhysics()
        self._phys.reset(seed=seed)
        self._ref = Referee(match_mode=match_mode, first_kickoff=first_kickoff)
        self._ref.reset(self._phys)
        self._episode = 0
        self._step = 0

    def tick(self) -> dict:
        sa, sb = self._phys.state_a(), self._phys.state_b()
        obs_a = build_robot_obs(sa, opponent_pos=sb.robot_pos,
                                add_noise=self._dr, rng=self._rng)
        act_a, _ = self._model_a.predict(obs_a, deterministic=True)
        act_b = predict_opponent_action(self._model_b, sb, sa.robot_pos,
                                        add_noise=self._dr, rng=self._rng)

        info = self._phys.step(act_a, act_b)
        decision = self._ref.update(self._phys, info)
        self._step += 1

        # A goal or halftime resets positions → start a fresh "episode" counter.
        if decision.goal_a or decision.goal_b or "halftime" in decision.events:
            self._episode += 1
            self._step = 0

        sa, sb = self._phys.state_a(), self._phys.state_b()
        return {
            "type": "step",
            "mode": "play",
            "robot_pos": sa.robot_pos.tolist(),
            "robot_heading": float(sa.robot_heading),
            "robot2_pos": sb.robot_pos.tolist(),
            "robot2_heading": float(sb.robot_heading),
            "ball_pos": sa.ball_pos.tolist(),
            "score": dict(decision.score),
            "clock": round(decision.clock, 2),
            "half": decision.half,
            "match_over": decision.match_over,
            "status": decision.status,
            "events": decision.events,
            "obs": [float(x) for x in obs_a],
            "episode": self._episode,
            "step": self._step,
        }
