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


def _fake_opponent_npz(path, seed):
    """A minimal valid opponent .npz (torch-free) — random weights of the right shape."""
    rng = np.random.default_rng(seed)
    np.savez(
        path,
        n_hidden=np.array(1),
        h0_W=rng.standard_normal((8, SELF_PLAY_OBS_DIM)).astype(np.float32),
        h0_b=rng.standard_normal(8).astype(np.float32),
        out_W=rng.standard_normal((4, 8)).astype(np.float32),
        out_b=rng.standard_normal(4).astype(np.float32),
        log_std=np.zeros(4, np.float32),
    )


def test_opponent_pool_samples_across_episodes(env, tmp_path):
    paths = []
    for i in range(4):
        p = str(tmp_path / f"opp_{i}.npz")
        _fake_opponent_npz(p, seed=i)
        paths.append(p)
    env.set_opponent_pool(paths)
    assert len(env._opponent_pool) == 4

    env.reset(seed=0)
    chosen = {id(env._opponent)}
    for _ in range(50):
        env.reset()                               # no seed → rng advances, re-samples opponent
        chosen.add(id(env._opponent))
    pool_ids = {id(o) for o in env._opponent_pool}
    assert chosen <= pool_ids                     # only ever picks from the pool
    assert len(chosen) >= 2                        # and it actually varies across episodes


def test_set_opponent_pool_skips_bad_paths(env, tmp_path):
    good = str(tmp_path / "good.npz")
    _fake_opponent_npz(good, seed=0)
    env.set_opponent_pool([good, str(tmp_path / "missing.npz"), None])
    assert len(env._opponent_pool) == 1           # bad/missing entries dropped, good kept
    env.reset(seed=0)
    assert env._opponent is not None
