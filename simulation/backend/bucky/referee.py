"""Autonomous referee — RoboCup Junior Soccer 1:1 rules engine.

Pure game logic (no I/O), driven once per physics step. It reads the
:class:`~bucky.physics.python_backend.TwoRobotPhysics` state, enforces the official NK
rules, mutates the physics (relocating the ball, removing/re-entering robots,
kicking off) and returns a :class:`RefereeDecision` describing what happened.

The same engine serves both surfaces (decision: shared referee):
  * ``match_mode=True``  → a full 2×7 min match: clock, halves, side-switch, score.
  * ``match_mode=False`` → training: every per-event rule applies, but the episode
    length is governed by the env (short episodes), not the match clock.

Coordinate convention (see :mod:`bucky.field`): robot **A** attacks +x (defends −x); robot
**B** attacks −x (defends +x).

Rules covered: ball out / out-of-reach §4.8/§4.9.5, lack of progress §4.6, robot
out-of-bounds + 30 s suspension + goal nullification + push waiver §4.8.2/§4.9,
defective robot §4.7, kickoff §4.4, scoring + conceding-team kickoff + own goals §4.5,
match duration/halves §4.2/§4.3.
"""
from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field as dc_field

import numpy as np

from bucky import field
from bucky.physics.python_backend import DT, ROBOT_COLLISION_DIST, TwoRobotPhysics

# ── Rule timings (seconds) ───────────────────────────────────────────────────
SUSPENSION_SECONDS = 30.0       # §4.9.2 out-of-bounds suspension
DEFECTIVE_SECONDS = 30.0        # §4.7.4 minimum time a defective robot stays out
OUT_OF_REACH_SECONDS = 3.0      # §4.9.5 "count to 3" before the ball is replaced
LACK_OF_PROGRESS_SECONDS = 5.0  # §4.6 ball stuck between robots
LACK_REPEAT_WINDOW = 20.0       # §4.6 if it recurs within this window → centre spot
STALL_SECONDS = 8.0             # §4.7.1 robot not moving → defective
GOAL_AREA_SECONDS = 20.0        # §4.7.2 too long in own goal area → defective
HALF_SECONDS = 7 * 60.0         # §4.2 two halves of 7 minutes

# ── Detection thresholds ─────────────────────────────────────────────────────
STALL_SPEED = 0.01              # m/s; below this the robot counts as "not moving"
BALL_STILL_SPEED = 0.03         # m/s; below this the ball counts as stationary
CONTEST_DIST = field.COLLISION_DIST + 0.06   # both robots this close to a still ball
CONTACT_DIST = ROBOT_COLLISION_DIST + 0.02   # robots counted as "in contact"
CONTACT_WAIVER_WINDOW = 0.3     # s; OOB within this long after contact → waived
NEUTRAL_CLEARANCE = field.ROBOT_RADIUS + field.BALL_RADIUS  # keep relocations clear


@dataclass
class _RobotState:
    removed: bool = False
    defective: bool = False        # removed specifically as defective (vs suspended)
    penalty_remaining: float = 0.0
    out_since_goal: bool = False   # defective may return when a goal is scored
    stall_timer: float = 0.0
    goal_area_timer: float = 0.0


@dataclass
class RefereeDecision:
    events: list[str] = dc_field(default_factory=list)
    goal_a: bool = False           # awarded this step (after nullification)
    goal_b: bool = False
    score: dict[str, int] = dc_field(default_factory=lambda: {"a": 0, "b": 0})
    clock: float = 0.0             # seconds elapsed in the current half (match mode)
    half: int = 1
    match_over: bool = False
    ball_relocated: bool = False
    kickoff: str | None = None     # team given the kickoff this step, if any
    status: dict[str, dict] = dc_field(default_factory=dict)


class Referee:
    def __init__(self, dt: float = DT, *, match_mode: bool = False,
                 first_kickoff: str = "a") -> None:
        self._dt = dt
        self._match_mode = match_mode
        self._first_kickoff = first_kickoff
        self._score = {"a": 0, "b": 0}
        self._clock = 0.0
        self._half = 1
        self._match_over = False
        self._rs = {"a": _RobotState(), "b": _RobotState()}
        self._ball_out_timer = 0.0
        self._lop_timer = 0.0
        self._lop_recent = 0.0           # time since last lack-of-progress call
        self._contact_timer = 1e9        # time since robots last touched
        self._prev_a = None
        self._prev_b = None
        self._sides_switched = False

    # ── lifecycle ────────────────────────────────────────────────────────────
    def reset(self, phys: TwoRobotPhysics) -> None:
        """Reset the match/episode and perform the opening kickoff."""
        self._score = {"a": 0, "b": 0}
        self._clock = 0.0
        self._half = 1
        self._match_over = False
        self._sides_switched = False
        self._reset_event_state()
        self._kickoff(phys, self._first_kickoff)

    def _reset_event_state(self) -> None:
        self._rs = {"a": _RobotState(), "b": _RobotState()}
        self._ball_out_timer = 0.0
        self._lop_timer = 0.0
        self._lop_recent = LACK_REPEAT_WINDOW + 1.0
        self._contact_timer = 1e9
        self._prev_a = None
        self._prev_b = None

    def _kickoff(self, phys: TwoRobotPhysics, team: str) -> None:
        phys.reset(kickoff=team)
        self._rs = {"a": _RobotState(), "b": _RobotState()}
        self._ball_out_timer = 0.0
        self._lop_timer = 0.0
        self._contact_timer = 1e9
        self._prev_a = None
        self._prev_b = None

    @property
    def score(self) -> dict[str, int]:
        return dict(self._score)

    # ── per-step update ──────────────────────────────────────────────────────
    def update(self, phys: TwoRobotPhysics, step_info: dict) -> RefereeDecision:
        dec = RefereeDecision(score=dict(self._score), half=self._half)
        if self._match_over:
            dec.match_over = True
            dec.clock = self._clock
            dec.status = self._status()
            return dec

        # 1) suspension / defective countdowns and re-entry
        self._tick_penalties(phys, dec)

        # 2) goals (validated against suspension) → score + kickoff
        scored = self._handle_goals(phys, step_info, dec)

        if not scored:
            # 3) new robot violations (out-of-bounds, defective)
            self._check_robot_violations(phys, dec)
            # 4) ball relocation (out-of-reach, lack of progress)
            self._check_ball(phys, step_info, dec)

        # 5) advance the match clock / halves (match mode only)
        if self._match_mode:
            self._advance_clock(phys, dec)

        self._update_trackers(phys)
        dec.clock = self._clock
        dec.half = self._half
        dec.score = dict(self._score)
        dec.status = self._status()
        return dec

    # ── (1) penalties ────────────────────────────────────────────────────────
    def _tick_penalties(self, phys: TwoRobotPhysics, dec: RefereeDecision) -> None:
        for which in ("a", "b"):
            rs = self._rs[which]
            if not rs.removed:
                continue
            rs.penalty_remaining = max(0.0, rs.penalty_remaining - self._dt)
            if rs.penalty_remaining <= 0.0:
                self._reenter(phys, which, dec)

    def _reenter(self, phys: TwoRobotPhysics, which: str, dec: RefereeDecision) -> None:
        defended = -1 if which == "a" else 1
        ball = phys.state_a().ball_pos
        other = phys.state_b().robot_pos if which == "a" else phys.state_a().robot_pos
        spot = field.nearest_neutral_to_goal(defended, ball_pos=ball,
                                             occupied=[other], clearance=NEUTRAL_CLEARANCE)
        heading = 0.0 if which == "a" else float(np.pi)
        phys.set_removed(which, False)
        phys.place_robot(which, spot, heading)
        self._rs[which] = _RobotState()
        dec.events.append(f"reenter_{which}")

    # ── (2) goals ────────────────────────────────────────────────────────────
    def _handle_goals(self, phys: TwoRobotPhysics, step_info: dict,
                      dec: RefereeDecision) -> bool:
        goal_a = bool(step_info.get("goal_a"))
        goal_b = bool(step_info.get("goal_b"))
        if not (goal_a or goal_b):
            return False

        # §4.9.2 — a goal by a team whose robot is currently suspended is disallowed.
        if goal_a and self._rs["a"].removed:
            dec.events.append("goal_a_disallowed")
            goal_a = False
        if goal_b and self._rs["b"].removed:
            dec.events.append("goal_b_disallowed")
            goal_b = False

        if not (goal_a or goal_b):
            # Disallowed: replace the ball at the nearest neutral spot, play on.
            self._relocate_ball(phys, dec, to_center=False, label="goal_void")
            return False

        if goal_a:
            self._score["a"] += 1
            dec.goal_a = True
            conceding = "b"
        else:
            self._score["b"] += 1
            dec.goal_b = True
            conceding = "a"
        dec.events.append("goal_a" if goal_a else "goal_b")
        dec.kickoff = conceding
        self._kickoff(phys, conceding)
        return True

    # ── (3) robot violations ─────────────────────────────────────────────────
    def _check_robot_violations(self, phys: TwoRobotPhysics, dec: RefereeDecision) -> None:
        for which in ("a", "b"):
            rs = self._rs[which]
            if rs.removed:
                continue
            pos = (phys.state_a().robot_pos if which == "a"
                   else phys.state_b().robot_pos)

            # Out of bounds — whole robot past the white line (§4.8.2/§4.9).
            if field.robot_fully_out(pos):
                if self._contact_timer <= CONTACT_WAIVER_WINDOW:
                    self._waive_back(phys, which, pos, dec)   # §4.9.4 pushed out
                else:
                    self._suspend(phys, which, dec)
                continue

            # Defective — not moving (§4.7.1) or too long in own goal area (§4.7.2).
            defended = -1 if which == "a" else 1
            if rs.stall_timer >= STALL_SECONDS:
                self._mark_defective(phys, which, dec, "stall")
            elif rs.goal_area_timer >= GOAL_AREA_SECONDS and field.in_penalty_area(pos, defended):
                self._mark_defective(phys, which, dec, "goal_area")

    def _suspend(self, phys: TwoRobotPhysics, which: str, dec: RefereeDecision) -> None:
        rs = self._rs[which]
        rs.removed = True
        rs.defective = False
        rs.penalty_remaining = SUSPENSION_SECONDS
        phys.set_removed(which, True)
        dec.events.append(f"out_of_bounds_{which}")

    def _mark_defective(self, phys: TwoRobotPhysics, which: str, dec: RefereeDecision,
                        reason: str) -> None:
        rs = self._rs[which]
        rs.removed = True
        rs.defective = True
        rs.penalty_remaining = DEFECTIVE_SECONDS
        phys.set_removed(which, True)
        dec.events.append(f"defective_{which}_{reason}")

    def _waive_back(self, phys: TwoRobotPhysics, which: str, pos, dec: RefereeDecision) -> None:
        nudged = np.array([
            float(np.clip(pos[0], -(field.HALF_W - field.ROBOT_RADIUS),
                          field.HALF_W - field.ROBOT_RADIUS)),
            float(np.clip(pos[1], -(field.HALF_H - field.ROBOT_RADIUS),
                          field.HALF_H - field.ROBOT_RADIUS)),
        ])
        phys.place_robot(which, nudged)
        dec.events.append(f"oob_waived_{which}")

    # ── (4) ball relocation ──────────────────────────────────────────────────
    def _check_ball(self, phys: TwoRobotPhysics, step_info: dict,
                    dec: RefereeDecision) -> None:
        ball = phys.state_a().ball_pos
        ball_vel = phys.state_a().ball_vel

        # Out of reach (§4.9.5): ball past the white line and not retrieved in 3 s.
        if field.ball_out_of_play(ball):
            self._ball_out_timer += self._dt
            if self._ball_out_timer >= OUT_OF_REACH_SECONDS:
                self._relocate_ball(phys, dec, to_center=False, label="out_of_reach")
                self._ball_out_timer = 0.0
        else:
            self._ball_out_timer = 0.0

        # Lack of progress (§4.6): still ball wedged between both robots.
        self._lop_recent += self._dt
        a_pos = phys.state_a().robot_pos
        b_pos = phys.state_b().robot_pos
        contested = (
            float(np.linalg.norm(ball_vel)) < BALL_STILL_SPEED
            and not self._rs["a"].removed and not self._rs["b"].removed
            and float(np.linalg.norm(a_pos - ball)) < CONTEST_DIST
            and float(np.linalg.norm(b_pos - ball)) < CONTEST_DIST
        )
        if contested:
            self._lop_timer += self._dt
            if self._lop_timer >= LACK_OF_PROGRESS_SECONDS:
                to_center = self._lop_recent < LACK_REPEAT_WINDOW
                self._relocate_ball(phys, dec, to_center=to_center, label="lack_of_progress")
                self._lop_timer = 0.0
                self._lop_recent = 0.0
        else:
            self._lop_timer = 0.0

    def _relocate_ball(self, phys: TwoRobotPhysics, dec: RefereeDecision, *,
                       to_center: bool, label: str) -> None:
        ball = phys.state_a().ball_pos
        if to_center:
            spot = field.CENTER_SPOT
        else:
            occupied = [phys.state_a().robot_pos, phys.state_b().robot_pos]
            spot = field.nearest_neutral_spot(ball, occupied=occupied,
                                              clearance=NEUTRAL_CLEARANCE)
        phys.place_ball(spot, vel=(0.0, 0.0))
        dec.ball_relocated = True
        dec.events.append(label)

    # ── (5) match clock ──────────────────────────────────────────────────────
    def _advance_clock(self, phys: TwoRobotPhysics, dec: RefereeDecision) -> None:
        self._clock += self._dt
        if self._clock < HALF_SECONDS:
            return
        if self._half == 1:
            self._half = 2
            self._clock = 0.0
            self._sides_switched = True
            second_kickoff = "b" if self._first_kickoff == "a" else "a"
            self._kickoff(phys, second_kickoff)
            dec.events.append("halftime")
        else:
            self._match_over = True
            dec.match_over = True
            dec.events.append("full_time")

    # ── trackers ─────────────────────────────────────────────────────────────
    def _update_trackers(self, phys: TwoRobotPhysics) -> None:
        a_pos = phys.state_a().robot_pos
        b_pos = phys.state_b().robot_pos

        # robot↔robot contact memory (for the push waiver)
        if (not self._rs["a"].removed and not self._rs["b"].removed
                and float(np.linalg.norm(a_pos - b_pos)) < CONTACT_DIST):
            self._contact_timer = 0.0
        else:
            self._contact_timer += self._dt

        for which, pos, prev in (("a", a_pos, self._prev_a), ("b", b_pos, self._prev_b)):
            rs = self._rs[which]
            if rs.removed:
                rs.stall_timer = 0.0
                rs.goal_area_timer = 0.0
                continue
            if prev is not None:
                speed = float(np.linalg.norm(pos - prev)) / self._dt
                rs.stall_timer = rs.stall_timer + self._dt if speed < STALL_SPEED else 0.0
            defended = -1 if which == "a" else 1
            if field.in_penalty_area(pos, defended):
                rs.goal_area_timer += self._dt
            else:
                rs.goal_area_timer = 0.0

        self._prev_a = a_pos.copy()
        self._prev_b = b_pos.copy()

    def _status(self) -> dict[str, dict]:
        out = {}
        for which in ("a", "b"):
            rs = self._rs[which]
            out[which] = {
                "removed": rs.removed,
                "suspended": rs.removed and not rs.defective,
                "defective": rs.removed and rs.defective,
                "penalty_remaining": round(rs.penalty_remaining, 2),
            }
        return out
