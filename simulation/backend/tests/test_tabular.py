"""Tabular Q-learning / SARSA(λ): training runs, the table learns, and it round-trips."""
import numpy as np
import pytest

from bucky.algos.tabular import (
    N_DISCRETE_ACTIONS,
    Discretizer,
    load_qtable,
    save_qtable,
    train_tabular,
)
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.policies.tabular import TabularPolicy


def test_discretizer_indices_in_range():
    disc = Discretizer()
    rng = np.random.default_rng(0)
    for _ in range(500):
        obs = rng.uniform(-3, 3, 39).astype(np.float32)
        s = disc.index(obs)
        assert 0 <= s < disc.n_states


@pytest.mark.parametrize("algo", ["q", "sarsa"])
def test_train_and_roundtrip(tmp_path, algo):
    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    q, disc = train_tabular(env, algo=algo, episodes=60, max_steps=200, seed=0)

    assert q.shape == (disc.n_states, N_DISCRETE_ACTIONS)
    assert np.count_nonzero(q) > 0, "no state-action values were updated"

    path = str(tmp_path / "p.qtable.npz")
    save_qtable(path, q, disc)
    q2, disc2 = load_qtable(path)
    assert np.array_equal(q2, q)
    assert disc2.to_dict() == disc.to_dict()

    policy = TabularPolicy.load(path)
    action, state = policy.predict(np.zeros(43, np.float32))   # wider obs is fine (indices < 18)
    assert state is None and action.shape == (4,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)


def test_loader_routes_qtable_npz_by_content(tmp_path):
    """A Q-table saved as a plain .npz (no .qtable suffix) must still route to TabularPolicy —
    the loader dispatches by file contents, not just the name."""
    from bucky.policies import load_policy

    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    q, disc = train_tabular(env, algo="q", episodes=20, max_steps=100, seed=0)
    path = str(tmp_path / "q.npz")          # deliberately NOT *.qtable.npz
    save_qtable(path, q, disc)
    assert type(load_policy(path)).__name__ == "TabularPolicy"
