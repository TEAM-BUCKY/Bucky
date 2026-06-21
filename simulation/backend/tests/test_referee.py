"""Tests for the autonomous referee (RoboCup Junior Soccer 1:1 rules engine)."""
import numpy as np
import pytest

from bucky import field
from bucky.physics.python_backend import DT, TwoRobotPhysics
from bucky.referee import (
    GOAL_AREA_SECONDS,
    LACK_OF_PROGRESS_SECONDS,
    OUT_OF_REACH_SECONDS,
    STALL_SECONDS,
    SUSPENSION_SECONDS,
    Referee,
)


def _is_neutral_spot(pos, tol=1e-6) -> bool:
    return any(np.linalg.norm(np.asarray(pos) - s) < tol for s in field.NEUTRAL_SPOTS)


@pytest.fixture
def setup():
    phys = TwoRobotPhysics()
    ref = Referee(match_mode=False)
    ref.reset(phys)
    return ref, phys


def run(ref, phys, seconds, info=None, before=None):
    """Step the referee for ``seconds``; returns the last decision with all events seen
    during the run accumulated into ``.events`` (rule events fire on a single step)."""
    info = info or {}
    last = None
    all_events: list[str] = []
    for _ in range(int(round(seconds / DT))):
        if before is not None:
            before(phys)
        last = ref.update(phys, info)
        all_events.extend(last.events)
    last.events = all_events
    return last


def step_until(ref, phys, label, max_seconds, before=None, info=None):
    """Step until ``label`` appears in a decision; return (decision, ball_pos at firing)."""
    info = info or {}
    for _ in range(int(round(max_seconds / DT))):
        if before is not None:
            before(phys)
        dec = ref.update(phys, info)
        if label in dec.events:
            return dec, phys.state_a().ball_pos.copy()
    return None, None


# ── ball out / out of reach (§4.8.1, §4.9.5) ─────────────────────────────────
def test_ball_out_relocated_to_neutral_spot_after_count_to_three(setup):
    ref, phys = setup
    phys.place_ball([0.0, field.HALF_H + 0.15])      # past the lateral white line
    dec = run(ref, phys, OUT_OF_REACH_SECONDS + 0.1)
    ball = phys.state_a().ball_pos
    assert _is_neutral_spot(ball)
    assert not field.ball_out_of_play(ball)
    assert "out_of_reach" in dec.events or any("out_of_reach" in e for e in dec.events)


def test_ball_back_in_time_is_not_relocated(setup):
    ref, phys = setup
    phys.place_ball([0.0, field.HALF_H + 0.15])
    run(ref, phys, OUT_OF_REACH_SECONDS - 0.5)
    phys.place_ball([0.0, 0.0])                       # retrieved before the count of 3
    dec = run(ref, phys, 0.2)
    assert np.linalg.norm(phys.state_a().ball_pos) < 0.05
    assert not dec.ball_relocated


# ── lack of progress (§4.6) ──────────────────────────────────────────────────
def test_lack_of_progress_relocates_then_centers_on_repeat(setup):
    ref, phys = setup
    tog = {"d": 0.0}
    def wedge(p):
        # Ball wedged off-centre between both robots, held stationary. The robots are
        # nudged slightly each step so they don't read as "not moving" (defective).
        tog["d"] = 0.006 if tog["d"] == 0.0 else 0.0
        p.place_ball([0.4, 0.0], vel=(0.0, 0.0))
        p.place_robot("a", [0.32 + tog["d"], 0.0])
        p.place_robot("b", [0.48 - tog["d"], 0.0])
    # First occurrence → nearest neutral spot (not the centre).
    dec1, ball1 = step_until(ref, phys, "lack_of_progress", LACK_OF_PROGRESS_SECONDS + 1, wedge)
    assert dec1 is not None and _is_neutral_spot(ball1)
    assert np.linalg.norm(ball1 - field.CENTER_SPOT) > 0.1
    # Recurrence within the window → ball goes to the centre spot.
    dec2, ball2 = step_until(ref, phys, "lack_of_progress", LACK_OF_PROGRESS_SECONDS + 1, wedge)
    assert dec2 is not None
    assert np.linalg.norm(ball2 - field.CENTER_SPOT) < 1e-6


# ── casual mode (online human-vs-human) ─────────────────────────────────────
def test_casual_mode_does_not_suspend_out_of_bounds_robot():
    phys = TwoRobotPhysics()
    ref = Referee(match_mode=False, casual=True)
    ref.reset(phys)
    phys.place_robot("a", [field.HALF_W + field.ROBOT_RADIUS + 0.05, 0.0])  # fully out
    dec = ref.update(phys, {})
    assert dec.status["a"]["suspended"] is False
    assert dec.status["a"]["removed"] is False
    assert not phys.is_removed("a")


def test_casual_mode_still_relocates_ball_out_of_play():
    # Ball handling is kept in casual so a ball shoved out still comes back.
    phys = TwoRobotPhysics()
    ref = Referee(match_mode=False, casual=True)
    ref.reset(phys)
    phys.place_ball([0.0, field.HALF_H + 0.15])
    dec = run(ref, phys, OUT_OF_REACH_SECONDS + 0.1)
    assert _is_neutral_spot(phys.state_a().ball_pos)


# ── robot out of bounds + suspension (§4.9) ──────────────────────────────────
def test_robot_out_is_suspended_then_reenters_after_30s(setup):
    ref, phys = setup
    phys.place_robot("a", [field.HALF_W + field.ROBOT_RADIUS + 0.05, 0.0])
    dec = ref.update(phys, {})
    assert dec.status["a"]["suspended"] is True
    assert phys.is_removed("a")
    # Still suspended just before 30 s.
    run(ref, phys, SUSPENSION_SECONDS - 1.0)
    assert phys.is_removed("a")
    # Re-enters after the suspension elapses, at a neutral spot on its own half.
    run(ref, phys, 1.2)
    assert not phys.is_removed("a")
    a = phys.state_a().robot_pos
    assert a[0] < 0.0 and _is_neutral_spot(a)


def test_oob_waived_when_pushed_out_by_opponent(setup):
    ref, phys = setup
    # Establish recent robot↔robot contact near the +x boundary.
    phys.place_robot("a", [field.HALF_W - 0.05, 0.0])
    phys.place_robot("b", [field.HALF_W - 0.05 - 0.15, 0.0])
    ref.update(phys, {})
    # Next step A is shoved fully out while still in contact range.
    phys.place_robot("a", [field.HALF_W + field.ROBOT_RADIUS + 0.05, 0.0])
    dec = ref.update(phys, {})
    assert not phys.is_removed("a")                   # penalty waived
    assert any("waived" in e for e in dec.events)
    assert not field.robot_fully_out(phys.state_a().robot_pos)


# ── goal nullification while suspended (§4.9.2) ──────────────────────────────
def test_goal_disallowed_while_team_suspended(setup):
    ref, phys = setup
    phys.place_robot("a", [field.HALF_W + field.ROBOT_RADIUS + 0.05, 0.0])
    ref.update(phys, {})                              # A suspended
    assert phys.is_removed("a")
    before = ref.score["a"]
    dec = ref.update(phys, {"goal_a": True})
    assert dec.goal_a is False
    assert ref.score["a"] == before
    assert any("disallow" in e for e in dec.events)


# ── defective robot (§4.7) ───────────────────────────────────────────────────
def test_defective_when_not_moving(setup):
    ref, phys = setup
    phys.place_robot("a", [0.0, 0.3])                 # stationary, mid-field
    phys.place_robot("b", [0.3, 0.4])
    dec = run(ref, phys, STALL_SECONDS + 0.1)
    assert dec.status["a"]["defective"] is True
    assert phys.is_removed("a")


def test_defective_when_too_long_in_goal_area(setup):
    ref, phys = setup
    phys.place_robot("b", [0.6, 0.4])                 # B out of the way, kept moving below
    toggle = {"v": 0.0}
    def jitter(p):
        toggle["v"] = 0.04 if toggle["v"] == 0.0 else 0.0
        p.place_robot("a", [-0.6, toggle["v"]])       # inside A's own penalty area, moving
        p.place_robot("b", [0.6, 0.4 + toggle["v"]])
    dec = run(ref, phys, GOAL_AREA_SECONDS + 0.2, before=jitter)
    assert phys.is_removed("a")
    assert any("defective_a" in e for e in dec.events)


# ── scoring & kickoff (§4.5) ─────────────────────────────────────────────────
def test_goal_scored_awards_and_conceding_team_kicks_off(setup):
    ref, phys = setup
    dec = ref.update(phys, {"goal_a": True})
    assert dec.goal_a and ref.score["a"] == 1
    assert dec.kickoff == "b"
    a, b, ball = phys.state_a().robot_pos, phys.state_b().robot_pos, phys.state_a().ball_pos
    assert np.linalg.norm(b - ball) < np.linalg.norm(a - ball)   # B (conceding) on the ball


def test_own_goal_counts_for_opponent(setup):
    ref, phys = setup
    # Ball driven into A's OWN (−x) goal → credited to B (§4.5.3); physics reports goal_b.
    phys.place_ball([-(field.HALF_W + 0.05), 0.0])
    info = phys.step((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert info["goal_b"] is True
    dec = ref.update(phys, info)
    assert dec.goal_b and ref.score["b"] == 1


# ── match clock / halves (§4.2, §4.3) ────────────────────────────────────────
def test_match_clock_halves_and_full_time(monkeypatch):
    import bucky.referee as R
    monkeypatch.setattr(R, "HALF_SECONDS", 1.0)
    phys = TwoRobotPhysics()
    ref = Referee(match_mode=True, first_kickoff="a")
    ref.reset(phys)
    # Drive forward so the robots are never flagged defective during the run.
    half1 = run(ref, phys, 0.5, before=lambda p: p.place_ball(
        [0.0, 0.0], vel=(0.0, 0.0)))
    assert half1.half == 1 and not half1.match_over
    run(ref, phys, 0.6)                               # crosses 1.0 s → halftime
    assert ref._half == 2
    ft = run(ref, phys, 1.1)                          # crosses 2nd half end → full time
    assert ft.match_over is True
    # Clock is frozen and no further scoring once the match is over.
    after = ref.update(phys, {"goal_a": True})
    assert after.match_over and after.goal_a is False
