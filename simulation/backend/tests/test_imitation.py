"""Behavioral cloning + supervised policies clone the expert and round-trip through .joblib."""
import numpy as np
import pytest

from bucky.algos.imitation import behavioral_cloning
from bucky.algos.supervised import fit_supervised, load_model, save_model
from bucky.controllers import ReactiveController
from bucky.data import collect
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.policies.sklearn_policy import SklearnPolicy


@pytest.fixture(scope="module")
def expert_data():
    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    return collect(ReactiveController(obs_dim=39), env, 2000, seed=0)


@pytest.mark.parametrize("kind", ["tree", "forest"])
def test_bc_clones_expert(expert_data, kind):
    model = behavioral_cloning(expert_data, kind=kind, seed=0)
    # A tree/forest fits the demonstrations closely → low training error.
    mse = float(np.mean((model.predict(expert_data.obs) - expert_data.actions) ** 2))
    assert mse < 0.05, f"{kind} BC did not fit (MSE {mse})"


def test_sklearn_policy_predict_and_roundtrip(expert_data, tmp_path):
    model = fit_supervised(expert_data.obs, expert_data.actions, kind="tree", seed=0)
    path = str(tmp_path / "p.joblib")
    save_model(model, path)
    policy = SklearnPolicy(load_model(path))

    # Feeds a wider (43-dim) obs; the adapter slices to the model's trained width.
    action, state = policy.predict(np.zeros(43, np.float32))
    assert state is None and action.shape == (4,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)


def test_loader_dispatches_joblib(expert_data, tmp_path):
    from bucky.policies import load_policy

    model = fit_supervised(expert_data.obs, expert_data.actions, kind="tree", seed=0)
    path = str(tmp_path / "clone.joblib")
    save_model(model, path)
    policy = load_policy(path)
    assert type(policy).__name__ == "SklearnPolicy"
