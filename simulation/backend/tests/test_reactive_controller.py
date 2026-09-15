"""The classical reactive controller must satisfy the shared policy contract and actually play:
drive toward the ball, and roll the ball goalward in the empty-goal drill."""
import numpy as np

from bucky.controllers import ReactiveController, ReactiveParams
from bucky.obs import build_observation
from bucky.physics.backend import PhysicsState


def _state(robot_pos, ball_pos, heading=0.0):
    return PhysicsState(
        robot_pos=np.array(robot_pos, float), robot_vel=np.zeros(2),
        robot_heading=heading, robot_omega=0.0,
        ball_pos=np.array(ball_pos, float), ball_vel=np.zeros(2),
    )


def test_action_shape_and_range():
    ctrl = ReactiveController(obs_dim=39)
    obs = build_observation(_state([-0.5, 0.0], [0.0, 0.0]))
    action, state = ctrl.predict(obs)
    assert state is None
    assert action.shape == (4,) and action.dtype == np.float32
    assert np.all(action[:3] >= -1.0) and np.all(action[:3] <= 1.0)
    assert np.hypot(action[0], action[1]) <= 1.0 + 1e-6   # velocity magnitude capped


def test_drives_toward_ball_ahead():
    """Robot behind the ball, both between it and the +x goal → should drive forward (+vx)."""
    ctrl = ReactiveController(obs_dim=39)
    obs = build_observation(_state([-0.6, 0.0], [-0.2, 0.0], heading=0.0))
    action, _ = ctrl.predict(obs)
    assert action[0] > 0.2, f"expected forward drive, got vx={action[0]}"


def test_params_vector_roundtrip():
    p = ReactiveParams(approach_gain=2.5, kick_distance=0.3)
    v = p.to_vector()
    back = ReactiveParams.from_vector(v)
    assert np.allclose(back.to_vector(), v)
    assert back.approach_gain == 2.5 and back.kick_distance == 0.3


def test_scores_in_empty_goal_drill():
    """Sanity that the controller is a functioning striker, not just a valid signature."""
    from bucky.envs.bucky_single import BuckySingleEnv

    ctrl = ReactiveController(obs_dim=39)
    env = BuckySingleEnv(stage="PUSH_TO_EMPTY_GOAL", domain_rand=False)
    goals = 0
    for ep in range(6):
        obs, _ = env.reset(seed=ep)
        for _ in range(2000):
            action, _ = ctrl.predict(obs)
            obs, _, term, trunc, info = env.step(action)
            if info.get("goal_scored"):
                goals += 1
            if term or trunc:
                break
    assert goals >= 2, f"controller scored only {goals}/6 in the empty-goal drill"
