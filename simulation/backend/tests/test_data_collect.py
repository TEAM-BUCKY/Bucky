"""Trajectory collection: shapes line up, episodes reset, and the .npz round-trips."""
import numpy as np

from bucky.controllers import ReactiveController
from bucky.data import Transitions, collect
from bucky.envs.bucky_single import BuckySingleEnv


def test_collect_shapes_and_roundtrip(tmp_path):
    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    ctrl = ReactiveController(obs_dim=39)
    data = collect(ctrl, env, n_steps=500, seed=0)

    assert len(data) == 500
    assert data.obs.shape == (500, 39) and data.actions.shape == (500, 4)
    assert data.rewards.shape == (500,) and data.terminals.shape == (500,)
    assert data.obs.dtype == np.float32 and data.terminals.dtype == bool
    # timeouts (truncation) is tracked separately from terminals — never both at once.
    assert data.timeouts.shape == (500,) and data.timeouts.dtype == bool
    assert not np.any(data.terminals & data.timeouts)
    # The drill terminates fast, so at least one episode boundary should appear in 500 steps.
    assert data.terminals.any() or data.timeouts.any()

    path = str(tmp_path / "d.npz")
    data.save_npz(path)
    back = Transitions.load_npz(path)
    assert np.array_equal(back.obs, data.obs)
    assert np.array_equal(back.actions, data.actions)
    assert np.array_equal(back.terminals, data.terminals)
    assert np.array_equal(back.timeouts, data.timeouts)
