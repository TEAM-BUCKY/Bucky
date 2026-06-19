import numpy as np
import pytest
from bucky.physics.backend import PhysicsBackend, PhysicsState

def test_physics_state_fields():
    s = PhysicsState(
        robot_pos=np.array([0.0, 0.0]),
        robot_vel=np.array([0.0, 0.0]),
        robot_heading=0.0,
        robot_omega=0.0,
        ball_pos=np.array([0.1, 0.0]),
        ball_vel=np.array([0.0, 0.0]),
    )
    assert s.robot_pos.shape == (2,)
