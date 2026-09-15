"""Discrete-action wrapper + inference adapter for value-based methods (DQN, tabular)."""
import numpy as np
from gymnasium import spaces

from bucky.envs.discrete_wrapper import (
    DISCRETE_ACTIONS,
    N_DISCRETE_ACTIONS,
    DiscreteActionWrapper,
)
from bucky.policies.discrete import DiscretePolicyAdapter


def test_action_table_valid():
    assert DISCRETE_ACTIONS.shape == (N_DISCRETE_ACTIONS, 4)
    assert DISCRETE_ACTIONS.dtype == np.float32
    assert np.all(DISCRETE_ACTIONS >= -1.0) and np.all(DISCRETE_ACTIONS <= 1.0)


def test_wrapper_exposes_discrete_and_maps_actions():
    from bucky.envs.bucky_single import BuckySingleEnv

    env = DiscreteActionWrapper(BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False))
    assert isinstance(env.action_space, spaces.Discrete)
    assert env.action_space.n == N_DISCRETE_ACTIONS
    # The wrapper translates an index to its continuous primitive before stepping.
    assert np.array_equal(env.action(3), DISCRETE_ACTIONS[3])
    obs, _ = env.reset(seed=0)
    obs2, reward, term, trunc, info = env.step(5)   # step with a discrete index
    assert obs2.shape == obs.shape


class _FixedQ:
    """A stand-in value policy that always picks action index 10 (drive forward + kick)."""
    observation_space = spaces.Box(-3.0, 3.0, (39,), np.float32)

    def predict(self, obs, deterministic=True):
        return np.int64(10), None


def test_adapter_maps_index_to_primitive_and_slices_obs():
    adapter = DiscretePolicyAdapter(_FixedQ(), obs_dim=39)
    # Feed a wider (self-play, 43-dim) obs; the adapter slices to the policy's 39-dim prefix.
    action, state = adapter.predict(np.zeros(43, np.float32))
    assert state is None
    assert action.shape == (4,)
    assert np.array_equal(action, DISCRETE_ACTIONS[10])
