import numpy as np
import pytest
from rl.physics.python_backend import PyPhysics, FIELD_W
from rl.rewards import RewardConfig, RewardTerms, compute_rewards

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

def test_total_is_weighted_sum(physics, config):
    s0, s1 = _state_pair(physics)
    terms = compute_rewards(s0, s1, config, info={})
    d = terms.as_dict()
    expected = sum(d.values())
    assert abs(terms.total - expected) < 1e-6
