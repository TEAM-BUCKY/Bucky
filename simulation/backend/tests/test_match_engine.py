"""Tests for the 1v1 match engine (scoreboard + auto-reset + frame schema)."""
import numpy as np

from bucky.match import MatchEngine
from bucky.physics.python_backend import FIELD_W
from bucky.selfplay import SELF_PLAY_OBS_DIM


class FixedModel:
    """Stand-in policy returning a constant action (no NN needed for logic tests)."""

    def __init__(self, action=(0.0, 0.0, 0.0)):
        self._a = np.array(action, dtype=np.float32)

    def predict(self, obs, deterministic=True):
        return self._a.copy(), None


def test_tick_frame_has_required_keys():
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    f = eng.tick()
    for k in ("robot_pos", "robot_heading", "robot2_pos", "robot2_heading",
              "ball_pos", "score", "mode", "obs", "episode", "step"):
        assert k in f, f"missing {k}"
    assert f["mode"] == "play"
    assert f["score"] == {"a": 0, "b": 0}
    assert len(f["robot_pos"]) == 2 and len(f["robot2_pos"]) == 2
    assert len(f["obs"]) == SELF_PLAY_OBS_DIM


def test_goal_for_a_increments_score_and_resets():
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    eng._phys._ball_pos = np.array([FIELD_W / 2 + 0.05, 0.0])
    eng._phys._ball_vel = np.zeros(2)
    f = eng.tick()
    assert f["score"] == {"a": 1, "b": 0}
    assert f["episode"] == 1                          # episode advanced on reset
    assert np.linalg.norm(np.array(f["ball_pos"])) < 0.5   # ball re-centered


def test_goal_for_b_increments_score():
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    eng._phys._ball_pos = np.array([-(FIELD_W / 2 + 0.05), 0.0])
    eng._phys._ball_vel = np.zeros(2)
    f = eng.tick()
    assert f["score"] == {"a": 0, "b": 1}


def test_conceding_team_gets_kickoff_after_goal():
    # A scores → B was scored against → B should restart on the ball.
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    eng._phys._ball_pos = np.array([FIELD_W / 2 + 0.05, 0.0])
    eng._phys._ball_vel = np.zeros(2)
    f = eng.tick()
    a = np.array(f["robot_pos"])
    b = np.array(f["robot2_pos"])
    ball = np.array(f["ball_pos"])
    assert np.linalg.norm(b - ball) < np.linalg.norm(a - ball)


def test_step_counter_advances_without_goal():
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    f1 = eng.tick()
    f2 = eng.tick()
    assert f1["step"] == 1 and f2["step"] == 2
    assert f1["episode"] == 0 and f2["episode"] == 0


def test_frame_reports_kicker_ready():
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0)
    f = eng.tick()
    assert f["kick_ready_a"] == 1.0 and f["kick_ready_b"] == 1.0


def test_human_control_overrides_policy_for_red():
    # Red is robot B. Its policy would sit still; the human drives it forward (vx=1). B faces
    # its own goal (heading≈π), so body-frame forward moves it in −x — it must displace.
    control = {"red_mode": "human", "action": [1.0, 0.0, 0.0, 0.0]}
    eng = MatchEngine(FixedModel(), FixedModel((0.0, 0.0, 0.0)), seed=0,
                      control_source=lambda: control)
    start = eng._phys.state_b().robot_pos.copy()
    for _ in range(10):
        eng.tick()
    assert np.linalg.norm(eng._phys.state_b().robot_pos - start) > 0.01


def test_ai_mode_uses_policy_not_human_action():
    # Same control payload but red_mode='ai' → the human action is ignored, B's policy drives.
    control = {"red_mode": "ai", "action": [1.0, 0.0, 0.0, 0.0]}
    eng = MatchEngine(FixedModel(), FixedModel((0.0, 0.0, 0.0)), seed=0,
                      control_source=lambda: control)
    start = eng._phys.state_b().robot_pos.copy()
    for _ in range(10):
        eng.tick()
    assert np.linalg.norm(eng._phys.state_b().robot_pos - start) < 0.01


def test_human_control_drives_blue_side_a():
    # New per-side shape: a human on side "a" drives blue while its policy would sit still.
    control = {"a": {"mode": "human", "action": [1.0, 0.0, 0.0, 0.0]}}
    eng = MatchEngine(FixedModel((0.0, 0.0, 0.0)), FixedModel(), seed=0,
                      control_source=lambda: control)
    start = eng._phys.state_a().robot_pos.copy()
    for _ in range(10):
        eng.tick()
    assert np.linalg.norm(eng._phys.state_a().robot_pos - start) > 0.01


def test_both_sides_human_simultaneously():
    # Two players, one per side — both robots move under human control.
    control = {"a": {"mode": "human", "action": [1.0, 0.0, 0.0, 0.0]},
               "b": {"mode": "human", "action": [1.0, 0.0, 0.0, 0.0]}}
    eng = MatchEngine(FixedModel((0.0, 0.0, 0.0)), FixedModel((0.0, 0.0, 0.0)), seed=0,
                      control_source=lambda: control)
    a0 = eng._phys.state_a().robot_pos.copy()
    b0 = eng._phys.state_b().robot_pos.copy()
    for _ in range(10):
        eng.tick()
    assert np.linalg.norm(eng._phys.state_a().robot_pos - a0) > 0.01
    assert np.linalg.norm(eng._phys.state_b().robot_pos - b0) > 0.01


def test_casual_mode_never_ends_and_scores():
    # Casual mode has no clock; a goal still increments the score and re-centres the ball.
    eng = MatchEngine(FixedModel(), FixedModel(), seed=0, mode="casual")
    eng._phys._ball_pos = np.array([FIELD_W / 2 + 0.05, 0.0])
    eng._phys._ball_vel = np.zeros(2)
    f = eng.tick()
    assert f["score"] == {"a": 1, "b": 0}
    assert f["match_over"] is False
    assert eng._ref._casual is True and eng._ref._match_mode is False
