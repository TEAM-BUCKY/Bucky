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
                 match_mode: bool = True, first_kickoff: str = "a",
                 control_source=None) -> None:
        # ``control_source`` (optional) is a zero-arg callable returning the latest manual
        # control for the RED robot (robot B), e.g.
        # ``{"action": [vx, vy, omega, kick], "red_mode": "human"}``. When it reports
        # ``red_mode == "human"`` the human action drives red instead of B's policy, so a person
        # can play (and test the kicker) against the AI on blue (robot A). ``None`` → AI only.
        self._model_a = model_a
        self._model_b = model_b
        self._control_source = control_source
        self._dr = domain_rand
        self._rng = np.random.default_rng(seed)
        self._phys = TwoRobotPhysics()
        self._phys.reset(seed=seed)
        self._ref = Referee(match_mode=match_mode, first_kickoff=first_kickoff)
        self._ref.reset(self._phys)
        self._episode = 0
        self._step = 0

    def _manual_red_action(self):
        """The human's action for red (robot B) when under human control, else ``None`` so the
        caller falls back to B's policy. Applied directly to robot B (no mirror — the mirror
        trick is only for *predicting* an opponent policy's action)."""
        if self._control_source is None:
            return None
        control = self._control_source() or {}
        if control.get("red_mode") != "human":
            return None
        action = control.get("action") or (0.0, 0.0, 0.0, 0.0)
        return np.asarray(action, dtype=np.float32)

    def tick(self) -> dict:
        sa, sb = self._phys.state_a(), self._phys.state_b()
        a_ready = 1.0 if self._phys._a_kick_cooldown == 0 else 0.0
        b_ready = 1.0 if self._phys._b_kick_cooldown == 0 else 0.0
        obs_a = build_robot_obs(sa, opponent_pos=sb.robot_pos,
                                add_noise=self._dr, rng=self._rng, kick_ready=a_ready)
        act_a, _ = self._model_a.predict(obs_a, deterministic=True)
        # Red (robot B) is the human's to drive when under human control; otherwise its policy.
        act_b = self._manual_red_action()
        if act_b is None:
            # Match play is deterministic — opponent stochasticity is a training-only device.
            act_b = predict_opponent_action(self._model_b, sb, sa.robot_pos,
                                            add_noise=self._dr, rng=self._rng, kick_ready=b_ready,
                                            deterministic=True)

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
            # Kicker charge state (1 = ready, 0 = recharging) so the UI can show an indicator.
            "kick_ready_a": a_ready,
            "kick_ready_b": b_ready,
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
