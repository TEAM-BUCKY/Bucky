"""Tests for the 1v1 self-play training environment."""
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from bucky.envs.bucky_selfplay import BuckySelfPlayEnv
from bucky.selfplay import SELF_PLAY_OBS_DIM


@pytest.fixture
def env():
    e = BuckySelfPlayEnv(domain_rand=False)
    yield e
    e.close()


def test_obs_shape_and_bounds(env):
    obs, _ = env.reset(seed=0)
    assert obs.shape == (SELF_PLAY_OBS_DIM,)
    assert env.observation_space.contains(obs.astype(np.float32))


def test_action_space(env):
    assert env.action_space.shape == (4,)


def test_step_returns_types(env):
    env.reset(seed=0)
    obs, r, term, trunc, info = env.step(env.action_space.sample())
    assert obs.shape == (SELF_PLAY_OBS_DIM,)
    assert isinstance(r, float)
    assert isinstance(term, bool) and isinstance(trunc, bool)
    assert "reward_terms" in info


def test_gymnasium_check_env():
    e = BuckySelfPlayEnv(domain_rand=False)
    check_env(e, warn=True)
    e.close()


def test_episode_terminates(env):
    env.reset(seed=0)
    for _ in range(2000):
        _, _, term, trunc, _ = env.step(np.array([1.0, 0.0, 0.0, 0.0]))
        if term or trunc:
            break
    else:
        pytest.fail("Episode did not terminate within 2000 steps")


def test_runs_with_standstill_opponent(env):
    env.reset(seed=0)
    env.set_opponent(None)                       # no frozen model → opponent stays put
    obs, r, term, trunc, info = env.step(np.array([0.0, 0.0, 0.0, 0.0]))
    assert obs.shape == (SELF_PLAY_OBS_DIM,)
