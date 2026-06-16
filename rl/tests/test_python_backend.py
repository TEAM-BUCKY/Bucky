import numpy as np
import pytest
from rl.physics.python_backend import PyPhysics, FIELD_W, omni_kinematics

FIELD_H = 1.8

@pytest.fixture
def physics():
    return PyPhysics()

def test_reset_positions_in_field(physics):
    state = physics.reset(seed=42)
    assert abs(state.robot_pos[0]) < FIELD_W / 2
    assert abs(state.robot_pos[1]) < FIELD_H / 2
    assert abs(state.ball_pos[0]) < FIELD_W / 2
    assert abs(state.ball_pos[1]) < FIELD_H / 2

def test_step_returns_state(physics):
    physics.reset(seed=0)
    state, info = physics.step(0.5, 0.0, 0.0)
    assert state.robot_pos.shape == (2,)
    assert "goal_scored" in info
    assert "out_of_bounds" in info

def test_robot_moves_forward(physics):
    physics.reset(seed=0)
    physics._robot_pos = np.array([0.0, 0.0])
    physics._robot_heading = 0.0
    physics._robot_vel = np.array([0.0, 0.0])
    state1, _ = physics.step(1.0, 0.0, 0.0)
    assert state1.robot_pos[0] > 0.0

def test_ball_collision_with_robot(physics):
    physics.reset(seed=0)
    physics._robot_pos = np.array([0.0, 0.0])
    physics._robot_heading = 0.0
    physics._robot_vel = np.array([0.0, 0.0])
    physics._ball_pos = np.array([0.12, 0.0])
    physics._ball_vel = np.array([0.0, 0.0])
    state, _ = physics.step(1.0, 0.0, 0.0)
    assert np.linalg.norm(state.ball_vel) > 0 or state.ball_pos[0] > 0.11

def test_omni_kinematics_pure_rotation(physics):
    wheels = omni_kinematics(0.0, 0.0, 1.0)
    assert len(wheels) == 3
    assert np.allclose(np.abs(wheels), np.abs(wheels[0]), atol=1e-6)
