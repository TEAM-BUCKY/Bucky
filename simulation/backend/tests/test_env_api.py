import numpy as np
import pytest
import gymnasium as gym
from gymnasium.utils.env_checker import check_env
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.curriculum import Stage

@pytest.fixture
def env():
    e = BuckySingleEnv(stage=Stage.SELF_PLAY_1V1, domain_rand=False)
    yield e
    e.close()

def test_gymnasium_check_env():
    env = BuckySingleEnv(stage=Stage.SELF_PLAY_1V1, domain_rand=False)
    check_env(env, warn=True)
    env.close()

def test_obs_in_bounds(env):
    obs, _ = env.reset(seed=0)
    assert env.observation_space.contains(obs.astype(np.float32))

def test_action_space_shape(env):
    assert env.action_space.shape == (4,)

def test_step_returns_correct_types(env):
    env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert isinstance(obs, np.ndarray)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)

def test_episode_terminates(env):
    env.reset(seed=0)
    for _ in range(1000):
        obs, reward, term, trunc, info = env.step(np.array([1.0, 0.0, 0.0, 0.0]))
        if term or trunc:
            break
    else:
        pytest.fail("Episode did not terminate within 1000 steps")

def test_reward_info_keys(env):
    env.reset(seed=0)
    _, _, _, _, info = env.step(env.action_space.sample())
    assert "reward_terms" in info

def test_push_to_goal_stage():
    env = BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (18,)
    env.close()
