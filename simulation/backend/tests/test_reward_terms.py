import numpy as np
import pytest
from bucky.physics.python_backend import PyPhysics, FIELD_W
from bucky.rewards import RewardConfig, RewardTerms, compute_rewards, ALIGN_RADIUS, OPP_GOAL  # noqa: F401

@pytest.fixture
def config():
    return RewardConfig()

@pytest.fixture
def physics():
    p = PyPhysics()
    p.reset(seed=0)
    return p

def _state_pair(physics, vx=0.0):
    s0 = physics._make_state()
    physics.step(vx, 0.0, 0.0)
    s1 = physics._make_state()
    return s0, s1

def test_reward_terms_all_finite(physics, config):
    s0, s1 = _state_pair(physics)
    terms = compute_rewards(s0, s1, config, info={})
    for name, val in terms.as_dict().items():
        assert np.isfinite(val), f"{name} is not finite: {val}"

def test_approach_potential_based(physics, config):
    s0, s1 = _state_pair(physics, vx=1.0)
    terms = compute_rewards(s0, s1, config, info={})
    d0 = np.linalg.norm(s0.ball_pos - s0.robot_pos)
    d1 = np.linalg.norm(s1.ball_pos - s1.robot_pos)
    expected = config.w_approach * (d0 - d1)
    assert abs(terms.approach - expected) < 1e-6

def test_ball_to_goal_potential_based(physics, config):
    opp_goal = np.array([FIELD_W / 2, 0.0])
    s0, s1 = _state_pair(physics, vx=1.0)
    terms = compute_rewards(s0, s1, config, info={})
    d0 = np.linalg.norm(s0.ball_pos - opp_goal)
    d1 = np.linalg.norm(s1.ball_pos - opp_goal)
    expected = config.w_ball_to_goal * (d0 - d1)
    assert abs(terms.ball_to_goal - expected) < 1e-6

def test_goal_bonus_on_goal_scored(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"goal_scored": True})
    assert terms.goal > 0.0

def test_out_of_bounds_penalty(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"out_of_bounds": True})
    assert terms.out_of_bounds < 0.0

def _place(physics, robot_pos, heading, ball_pos):
    physics._robot_pos = np.asarray(robot_pos, dtype=float)
    physics._robot_heading = float(heading)
    physics._ball_pos = np.asarray(ball_pos, dtype=float)
    return physics._make_state()

def test_front_alignment_rewards_correct_setup(physics, config):
    # Robot just behind the ball (-x side), front facing the ball (+x) — which
    # is also toward the goal, so it can catch and drive to the goal.
    ball = np.array([0.0, 0.0])
    robot = ball - np.array([0.1, 0.0])  # within ALIGN_RADIUS, behind the ball
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment > 0.0

def test_front_alignment_penalizes_wrong_side_and_rotation(physics, config):
    # Robot on the goal (+x) side of the ball, front pointing away from the ball
    # (toward the goal): can't catch it AND wrong side to drive it.
    ball = np.array([0.0, 0.0])
    robot = ball + np.array([0.1, 0.0])
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment < 0.0

def test_front_alignment_gated_by_proximity(physics, config):
    # Robot beyond ALIGN_RADIUS from the ball -> term fades to zero.
    ball = np.array([0.0, 0.0])
    robot = ball - np.array([ALIGN_RADIUS + 0.1, 0.0])
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment == 0.0

def test_front_alignment_finite_when_ball_on_goal(physics, config):
    # Degenerate: ball exactly at the goal -> term stays finite (0).
    s = _place(physics, np.array([1.1, 0.0]), 0.0, OPP_GOAL.copy())
    terms = compute_rewards(s, s, config, info={})
    assert np.isfinite(terms.front_alignment)

def test_total_is_weighted_sum(physics, config):
    s0, s1 = _state_pair(physics)
    terms = compute_rewards(s0, s1, config, info={})
    d = terms.as_dict()
    expected = sum(d.values())
    assert abs(terms.total - expected) < 1e-6


# ── skilled-play terms (driven by env-supplied info flags) ───────────────────
def test_steal_scales_with_gradient(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"stole_ball": True, "steal_gradient": 0.8})
    assert abs(terms.steal - config.w_steal * 0.8) < 1e-6


def test_steal_zero_without_flag(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={}).steal == 0.0


def test_blocked_shot_positive(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={"blocked_shot": True}).blocked_shot > 0.0


def test_kick_goal_bonus_only_on_goal(physics, config):
    s = physics._make_state()
    scored = compute_rewards(s, s, config, info={"goal_scored": True, "kicked_goal": True})
    assert abs(scored.kick_goal - config.w_kick_goal) < 1e-6
    # kicked flag without an actual goal → no bonus
    assert compute_rewards(s, s, config, info={"kicked_goal": True}).kick_goal == 0.0


def test_bank_shot_bonus_only_on_goal(physics, config):
    s = physics._make_state()
    scored = compute_rewards(s, s, config, info={"goal_scored": True, "bank_shot": True})
    assert abs(scored.bank_shot - config.w_bank_shot) < 1e-6
    assert compute_rewards(s, s, config, info={"bank_shot": True}).bank_shot == 0.0


def test_risky_shot_scales_with_factor(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"risky_shot": True, "risky_factor": 0.5})
    assert abs(terms.risky_shot - config.w_risky_shot * 0.5) < 1e-6


def test_kick_lost_is_extra_punishment(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={"kick_lost": True}).kick_lost < 0.0


def test_action_magnitude_ignores_kick_dim(physics, config):
    s = physics._make_state()
    action = np.array([0.5, 0.5, 0.0, 1.0])   # big kick dim must not be penalized
    terms = compute_rewards(s, s, config, info={}, action=action)
    expected = config.w_action_mag * (0.5**2 + 0.5**2)
    assert abs(terms.action_magnitude - expected) < 1e-6
