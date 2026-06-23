"""Tests for the solid goal box, the in-goal penalty, the possession-decay reshape, and the
look-ahead 'inevitable goal' rollout that closes the kick→goal loop."""
import numpy as np
import pytest

from bucky.game.field import (
    ARENA_HALF_X, GOAL_HALF_WIDTH, HALF_W, ROBOT_RADIUS, robot_in_goal,
)
from bucky.physics.backend import PhysicsState
from bucky.physics.python_backend import PyPhysics, predict_goal_by_rollout
from bucky.rewards import CAPTURE_RADIUS, RewardConfig, compute_rewards


@pytest.fixture
def physics():
    p = PyPhysics()
    p.reset(seed=0)
    return p


def _drive_many(physics, vx, vy, n=60):
    for _ in range(n):
        state, _ = physics.step(vx, vy, 0.0)
    return state


# ── Solid goal box (physics) ──────────────────────────────────────────────────

def test_robot_cannot_cross_goal_side_wall_from_inside(physics):
    """A robot inside the goal box driving laterally cannot push through the side wall."""
    physics._robot_pos = np.array([1.0, 0.0])   # inside the +x goal box, centred
    physics._robot_heading = np.pi / 2
    state = _drive_many(physics, 0.0, 1.0)       # drive hard toward +y side wall
    assert abs(state.robot_pos[1]) < GOAL_HALF_WIDTH    # never crossed into the side band


def test_robot_cannot_enter_goal_box_from_side_band(physics):
    """A robot in the band beside the goal cannot push through the side wall into the box."""
    physics._robot_pos = np.array([1.0, 0.45])  # behind the goal line, beside the goal
    physics._robot_heading = -np.pi / 2
    state = _drive_many(physics, 0.0, 1.0)       # drive hard toward -y (into the box)
    assert state.robot_pos[1] > GOAL_HALF_WIDTH         # stayed out of the box


def test_robot_cannot_pass_through_to_rear_band(physics):
    """Driving straight at the goal, the robot is stopped well short of the arena back wall."""
    physics._robot_pos = np.array([0.85, 0.0])
    physics._robot_heading = 0.0
    state = _drive_many(physics, 1.0, 0.0)
    assert state.robot_pos[0] <= ARENA_HALF_X - ROBOT_RADIUS + 1e-6


def test_robot_may_enter_open_mouth(physics):
    """The mouth is open: a robot can poke its centre past the goal line (then gets penalised)."""
    physics._robot_pos = np.array([0.85, 0.0])
    physics._robot_heading = 0.0
    state = _drive_many(physics, 1.0, 0.0)
    assert state.robot_pos[0] > HALF_W
    assert robot_in_goal(state.robot_pos)


def test_ball_still_scores_through_mouth(physics):
    """Regression: the robot↔goal change must not block a ball scoring through the mouth."""
    physics._robot_pos = np.array([-0.5, 0.0])
    physics._ball_pos = np.array([0.88, 0.0])
    physics._ball_vel = np.array([4.0, 0.0])
    _, info = physics.step(0.0, 0.0, 0.0)
    assert info["goal_scored"]


# ── In-goal penalty (reward) ─────────────────────────────────────────────────

def _state(robot_pos, ball_pos, heading=0.0, ball_vel=(0.0, 0.0), robot_vel=(0.0, 0.0)):
    return PhysicsState(
        robot_pos=np.array(robot_pos, dtype=float),
        robot_vel=np.array(robot_vel, dtype=float),
        robot_heading=heading,
        robot_omega=0.0,
        ball_pos=np.array(ball_pos, dtype=float),
        ball_vel=np.array(ball_vel, dtype=float),
    )


def test_in_goal_penalty_applies():
    cfg = RewardConfig()
    s = _state(robot_pos=[1.0, 0.0], ball_pos=[-0.5, 0.0])
    terms = compute_rewards(s, s, cfg, info={})
    assert terms.in_goal == cfg.w_in_goal
    assert terms.in_goal < 0


def test_in_goal_penalty_absent_in_field():
    cfg = RewardConfig()
    s = _state(robot_pos=[0.0, 0.0], ball_pos=[0.5, 0.0])
    terms = compute_rewards(s, s, cfg, info={})
    assert terms.in_goal == 0.0


# ── Possession decay reshape ─────────────────────────────────────────────────

def test_possession_full_at_zero_hold():
    cfg = RewardConfig()
    s = _state(robot_pos=[0.0, 0.0], ball_pos=[0.1, 0.0])   # in capture radius, facing +x
    assert 0.1 < CAPTURE_RADIUS
    terms = compute_rewards(s, s, cfg, info={"possession_steps": 0})
    assert terms.possession == pytest.approx(cfg.w_possession)


def test_possession_decays_to_zero_when_camping():
    cfg = RewardConfig()
    s = _state(robot_pos=[0.0, 0.0], ball_pos=[0.1, 0.0])
    terms = compute_rewards(s, s, cfg, info={"possession_steps": int(cfg.possession_decay_steps)})
    assert terms.possession == pytest.approx(0.0, abs=1e-9)


def test_possession_facing_away_still_penalised():
    cfg = RewardConfig()
    # Ball behind the robot (robot faces +x, ball at -x) → facing-away penalty, no decay.
    s = _state(robot_pos=[0.0, 0.0], ball_pos=[-0.1, 0.0])
    terms = compute_rewards(s, s, cfg, info={"possession_steps": 999})
    assert terms.possession == pytest.approx(-cfg.w_possession)


# ── Look-ahead rollout ───────────────────────────────────────────────────────

def test_rollout_predicts_goal_on_target():
    scored, steps, intercepted = predict_goal_by_rollout(
        np.array([0.5, 0.0]), np.array([3.0, 0.0]), attack_sign=1)
    assert scored
    assert steps > 0
    assert not intercepted


def test_rollout_rejects_wide_shot():
    scored, _, _ = predict_goal_by_rollout(
        np.array([0.5, 0.0]), np.array([0.0, 3.0]), attack_sign=1)
    assert not scored


def test_rollout_flags_interception():
    scored, _, intercepted = predict_goal_by_rollout(
        np.array([0.5, 0.0]), np.array([3.0, 0.0]), attack_sign=1,
        opponent_pos=np.array([0.8, 0.0]))
    assert intercepted
