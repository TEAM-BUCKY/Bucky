import numpy as np
import pytest
from bucky.physics.python_backend import (
    PyPhysics, FIELD_W, FIELD_H, ARENA_HALF_X, ARENA_HALF_Y, ROBOT_RADIUS, omni_kinematics,
)

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
    assert "ball_out" in info

def test_robot_can_enter_outer_band(physics):
    """Robot is bounded by the arena wall, not the white line — it may roam the band."""
    physics.reset(seed=0)
    physics._robot_pos = np.array([10.0, 10.0])  # well past everything
    state, _ = physics.step(0.0, 0.0, 0.0)
    assert state.robot_pos[0] > FIELD_W / 2                         # crossed the white line
    assert state.robot_pos[1] > FIELD_H / 2
    assert state.robot_pos[0] <= ARENA_HALF_X - ROBOT_RADIUS + 1e-6  # stopped at the wall
    assert state.robot_pos[1] <= ARENA_HALF_Y - ROBOT_RADIUS + 1e-6

def test_ball_out_flag_in_outer_band(physics):
    physics.reset(seed=0)
    physics._robot_pos = np.array([0.0, 0.0])
    physics._ball_pos = np.array([0.0, FIELD_H / 2 + 0.1])  # past lateral white line
    physics._ball_vel = np.zeros(2)
    _, info = physics.step(0.0, 0.0, 0.0)
    assert bool(info["ball_out"]) is True
    assert bool(info["goal_scored"]) is False

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
