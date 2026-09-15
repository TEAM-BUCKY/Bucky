"""Offline RL (d3rlpy CQL/IQL) trains from a dataset and round-trips through .d3.

Skipped unless the optional `learners-extra` group (d3rlpy) is installed."""
import numpy as np
import pytest

pytest.importorskip("d3rlpy")

from bucky.algos.offline import save_offline, train_offline  # noqa: E402
from bucky.controllers import ReactiveController  # noqa: E402
from bucky.data import collect  # noqa: E402
from bucky.envs.bucky_single import BuckySingleEnv  # noqa: E402
from bucky.policies import load_policy  # noqa: E402


@pytest.fixture(scope="module")
def dataset():
    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    return collect(ReactiveController(obs_dim=39), env, 1000, seed=0)


@pytest.mark.parametrize("algo", ["cql", "iql"])
def test_offline_train_and_loader_roundtrip(dataset, tmp_path, algo):
    model = train_offline(dataset, algo=algo, n_steps=150, n_steps_per_epoch=150, seed=0)
    assert tuple(model.observation_shape) == (39,)
    path = str(tmp_path / f"{algo}.d3")
    save_offline(model, path)

    policy = load_policy(path)                     # loader dispatches .d3 → D3rlpyPolicy
    assert type(policy).__name__ == "D3rlpyPolicy"
    action, state = policy.predict(np.zeros(43, np.float32))   # wider obs sliced to 39
    assert state is None and action.shape == (4,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)
