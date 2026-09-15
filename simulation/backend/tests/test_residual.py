"""Fuzzy + RL residual hybrid: the wrapper adds a residual to the controller, and the match-time
adapter recombines the same way (zero residual → exactly the controller's action)."""
import numpy as np
from gymnasium import spaces

from bucky.envs.residual_wrapper import RESIDUAL_SCALE, ResidualActionWrapper
from bucky.policies.residual import ResidualPolicyAdapter


class _ZeroResidual:
    """A stand-in RL policy that always outputs a zero residual."""
    observation_space = spaces.Box(-3.0, 3.0, (39,), np.float32)

    def predict(self, obs, deterministic=True):
        return np.zeros(4, np.float32), None


def test_wrapper_keeps_box_action_and_steps():
    from bucky.envs.bucky_single import BuckySingleEnv

    env = ResidualActionWrapper(BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False))
    assert isinstance(env.action_space, spaces.Box) and env.action_space.shape == (4,)
    obs, _ = env.reset(seed=0)
    # Step with a zero residual — the controller drives; the episode advances without error.
    obs2, reward, term, trunc, info = env.step(np.zeros(4, np.float32))
    assert obs2.shape == obs.shape


def test_adapter_zero_residual_equals_controller():
    from bucky.controllers import ReactiveController
    from bucky.obs import build_observation
    from bucky.physics.backend import PhysicsState

    ctrl = ReactiveController(obs_dim=39)
    adapter = ResidualPolicyAdapter(_ZeroResidual(), base=ctrl)
    state = PhysicsState(
        robot_pos=np.array([-0.6, 0.0]), robot_vel=np.zeros(2), robot_heading=0.0,
        robot_omega=0.0, ball_pos=np.array([-0.2, 0.0]), ball_vel=np.zeros(2))
    obs = build_observation(state)

    base_action, _ = ctrl.predict(obs)
    combined, _ = adapter.predict(obs)
    # residual = 0 → combined == clip(base + scale*0) == base.
    assert np.allclose(combined, base_action, atol=1e-6)
    assert RESIDUAL_SCALE > 0
