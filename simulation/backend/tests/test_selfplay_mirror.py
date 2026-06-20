"""Tests for the x-axis mirror trick used to run robot B's policy."""
import numpy as np

from bucky.physics.python_backend import TwoRobotPhysics
from bucky.physics.backend import PhysicsState
from bucky.selfplay import (
    reflect_pos, reflect_vel, reflect_heading, reflect_omega, mirror_action,
    reflect_state, build_robot_obs,
)
from bucky.obs import build_observation, OBS_DIM


def test_reflect_pos_negates_x():
    assert np.allclose(reflect_pos(np.array([1.0, 2.0])), [-1.0, 2.0])


def test_reflect_vel_negates_x():
    assert np.allclose(reflect_vel(np.array([3.0, -4.0])), [-3.0, -4.0])


def test_reflect_heading_flips_across_y_axis():
    # Heading is a direction vector; negating x maps (cos h, sin h) → (−cos h, sin h).
    h = reflect_heading(0.0)                                    # +x facing → −x facing
    assert np.isclose(np.cos(h), -1.0) and np.isclose(np.sin(h), 0.0, atol=1e-9)
    h2 = reflect_heading(np.pi)                                 # −x facing → +x facing
    assert np.isclose(np.cos(h2), 1.0) and np.isclose(np.sin(h2), 0.0, atol=1e-9)


def test_reflect_omega_negates():
    assert reflect_omega(1.5) == -1.5


def test_mirror_action_keeps_forward_flips_strafe_and_spin():
    assert np.allclose(mirror_action(np.array([1.0, 0.0, 0.0])), [1.0, 0.0, 0.0])
    assert np.allclose(mirror_action(np.array([0.0, 1.0, 0.0])), [0.0, -1.0, 0.0])
    assert np.allclose(mirror_action(np.array([0.0, 0.0, 1.0])), [0.0, 0.0, -1.0])


def test_mirror_action_preserves_kick_dim():
    # 4-D action: forward & kick unchanged, strafe & spin flip.
    assert np.allclose(mirror_action(np.array([0.5, 0.3, 0.2, 0.8])), [0.5, -0.3, -0.2, 0.8])


def test_reflect_state_is_involution():
    st = PhysicsState(
        robot_pos=np.array([0.3, -0.2]), robot_vel=np.array([0.1, 0.4]),
        robot_heading=0.7, robot_omega=0.5,
        ball_pos=np.array([-0.1, 0.2]), ball_vel=np.array([0.2, -0.3]),
    )
    back = reflect_state(reflect_state(st))
    assert np.allclose(back.robot_pos, st.robot_pos)
    assert np.allclose(back.robot_vel, st.robot_vel)
    assert np.isclose((back.robot_heading - st.robot_heading + np.pi) % (2 * np.pi) - np.pi, 0.0)
    assert np.allclose(back.ball_pos, st.ball_pos)


def test_build_robot_obs_is_22_dims():
    st = PhysicsState(
        robot_pos=np.array([0.0, 0.0]), robot_vel=np.zeros(2),
        robot_heading=0.0, robot_omega=0.0,
        ball_pos=np.array([0.5, 0.0]), ball_vel=np.zeros(2),
    )
    obs = build_robot_obs(st, opponent_pos=np.array([0.6, 0.0]))
    assert obs.shape == (22,)
    assert obs.dtype == np.float32
    # First OBS_DIM dims match the single-agent observation exactly.
    assert np.allclose(obs[:OBS_DIM], build_observation(st))


def test_mirror_keeps_b_the_reflection_of_a():
    """If A and B start as x-mirror images and B runs mirror_action(raw),
    they remain exact mirror images after a physics step — proving the trick."""
    phys = TwoRobotPhysics()
    phys.reset(seed=0)
    phys._a_pos = np.array([-0.5, 0.2])
    phys._a_heading = 0.3
    phys._a_vel = np.zeros(2)
    phys._a_omega = 0.0
    phys._b_pos = reflect_pos(phys._a_pos)
    phys._b_heading = reflect_heading(0.3)
    phys._b_vel = np.zeros(2)
    phys._b_omega = 0.0
    phys._ball_pos = np.array([0.0, 0.0])
    phys._ball_vel = np.zeros(2)

    raw = np.array([0.7, 0.4, 0.5])
    phys.step(raw, mirror_action(raw))

    a, b = phys.state_a(), phys.state_b()
    assert np.allclose(b.robot_pos, reflect_pos(a.robot_pos), atol=1e-6)
    assert np.allclose(b.robot_vel, reflect_vel(a.robot_vel), atol=1e-6)
    # B's heading is A's heading with the x-component negated (y-axis reflection).
    assert np.isclose(np.cos(b.robot_heading), -np.cos(a.robot_heading), atol=1e-6)
    assert np.isclose(np.sin(b.robot_heading), np.sin(a.robot_heading), atol=1e-6)
