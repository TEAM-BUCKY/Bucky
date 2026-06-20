"""The pure-numpy frozen opponent must match SB3's deterministic PPO action exactly.

Self-play evaluates the opponent in numpy (NumpyOpponent) inside every SubprocVecEnv
worker so the workers never import torch — that torch-per-worker footprint is what
OOM-killed self-play training. These tests pin the numpy forward pass to PPO.predict so
the substitution can never silently drift, and guard that the worker path stays torch-free.
"""
import sys

import numpy as np
import pytest

from bucky.selfplay import (
    SELF_PLAY_OBS_DIM, export_policy_npz, load_numpy_opponent,
)


def _make_model():
    from stable_baselines3 import PPO
    import gymnasium as gym
    from gymnasium import spaces

    class Dummy(gym.Env):
        observation_space = spaces.Box(-3.0, 3.0, (SELF_PLAY_OBS_DIM,), np.float32)
        action_space = spaces.Box(-1.0, 1.0, (3,), np.float32)

        def reset(self, *a, **k):
            return self.observation_space.sample(), {}

        def step(self, a):
            return self.observation_space.sample(), 0.0, True, False, {}

    return PPO("MlpPolicy", Dummy(), policy_kwargs={"net_arch": [64, 64]},
               seed=0, device="cpu")


def test_numpy_opponent_matches_ppo_predict(tmp_path):
    model = _make_model()
    npz = str(tmp_path / "opponent_snapshot.npz")
    export_policy_npz(model, npz)
    opp = load_numpy_opponent(npz)

    rng = np.random.default_rng(0)
    for _ in range(200):
        obs = rng.uniform(-3.0, 3.0, SELF_PLAY_OBS_DIM).astype(np.float32)
        ref, _ = model.predict(obs, deterministic=True)
        got, _ = opp.predict(obs, deterministic=True)
        assert np.allclose(ref, got, atol=1e-5), f"\nref={ref}\ngot={got}"


def test_numpy_opponent_output_is_clipped(tmp_path):
    model = _make_model()
    npz = str(tmp_path / "opponent_snapshot.npz")
    export_policy_npz(model, npz)
    opp = load_numpy_opponent(npz)
    action, state = opp.predict(np.full(SELF_PLAY_OBS_DIM, 3.0, np.float32))
    assert state is None
    assert action.shape == (3,) and action.dtype == np.float32
    assert np.all(action >= -1.0) and np.all(action <= 1.0)


def test_worker_path_never_imports_torch(tmp_path):
    """The decisive guard: a worker that builds the self-play env and runs the opponent
    must never import torch — that is the entire point of the numpy opponent. Run it in a
    clean subprocess so torch imported by other tests in this process can't mask a regression.
    """
    import subprocess

    model = _make_model()
    npz = str(tmp_path / "opponent_snapshot.npz")
    export_policy_npz(model, npz)

    child = (
        "import sys\n"
        "from bucky.envs.bucky_selfplay import BuckySelfPlayEnv\n"
        "env = BuckySelfPlayEnv(domain_rand=False)\n"
        f"env.set_opponent({npz!r})\n"
        "obs, _ = env.reset()\n"
        "env.step(env.action_space.sample())\n"
        "assert env._opponent is not None, 'opponent failed to load'\n"
        "assert 'torch' not in sys.modules, 'torch was imported in the worker path!'\n"
        "print('OK')\n"
    )
    proc = subprocess.run([sys.executable, "-c", child], capture_output=True, text=True)
    assert proc.returncode == 0, f"stdout={proc.stdout}\nstderr={proc.stderr}"
    assert "OK" in proc.stdout
