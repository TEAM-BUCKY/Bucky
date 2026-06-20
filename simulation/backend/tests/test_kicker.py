"""Tests for the kicker (rule 3.8.1): physics-capped impulse, gating, cooldown."""
import numpy as np
import pytest

from bucky.physics.python_backend import (
    PyPhysics, TwoRobotPhysics, apply_kick,
    KICK_MAX_SPEED, KICK_RANGE, KICK_COOLDOWN_STEPS, KICK_DEADZONE,
)

ZERO4 = (0.0, 0.0, 0.0, 0.0)


def _place(p, robot_pos, heading, ball_pos):
    p._robot_pos = np.asarray(robot_pos, dtype=float)
    p._robot_heading = float(heading)
    p._robot_vel = np.zeros(2)
    p._robot_vel_cmd = np.zeros(2)
    p._ball_pos = np.asarray(ball_pos, dtype=float)
    p._ball_vel = np.zeros(2)


@pytest.fixture
def phys():
    p = PyPhysics()
    p.reset(seed=0)
    return p


def test_kick_imparts_speed_up_to_max(phys):
    _place(phys, [0.0, 0.0], 0.0, [0.14, 0.0])     # ball in front, within range, no contact
    _, info = phys.step(0.0, 0.0, 0.0, kick=1.0)
    assert info["kicked"] is True
    assert info["kick_speed"] == pytest.approx(KICK_MAX_SPEED, rel=1e-6)
    speed = np.linalg.norm(phys._ball_vel)
    assert speed <= KICK_MAX_SPEED + 1e-6
    assert speed == pytest.approx(KICK_MAX_SPEED, rel=1e-3)
    assert phys._ball_vel[0] > 0.0                  # kicked forward (+x heading)


def test_kick_force_is_selectable(phys):
    _place(phys, [0.0, 0.0], 0.0, [0.14, 0.0])
    _, info = phys.step(0.0, 0.0, 0.0, kick=0.5)
    assert info["kick_speed"] == pytest.approx(0.5 * KICK_MAX_SPEED, rel=1e-6)


def test_no_kick_when_ball_out_of_range(phys):
    _place(phys, [0.0, 0.0], 0.0, [0.5, 0.0])      # beyond KICK_RANGE
    _, info = phys.step(0.0, 0.0, 0.0, kick=1.0)
    assert info["kicked"] is False
    assert np.linalg.norm(phys._ball_vel) == 0.0


def test_no_kick_when_ball_behind(phys):
    _place(phys, [0.0, 0.0], 0.0, [-0.14, 0.0])    # behind robot (outside forward cone)
    _, info = phys.step(0.0, 0.0, 0.0, kick=1.0)
    assert info["kicked"] is False


def test_kick_deadzone(phys):
    _place(phys, [0.0, 0.0], 0.0, [0.14, 0.0])
    _, info = phys.step(0.0, 0.0, 0.0, kick=KICK_DEADZONE / 2)
    assert info["kicked"] is False


def test_cooldown_blocks_refire(phys):
    _place(phys, [0.0, 0.0], 0.0, [0.14, 0.0])
    _, info1 = phys.step(0.0, 0.0, 0.0, kick=1.0)
    assert info1["kicked"] is True
    # Re-seat the ball in front and try again immediately — cooldown must block it.
    _place(phys, [0.0, 0.0], 0.0, [0.14, 0.0])
    _, info2 = phys.step(0.0, 0.0, 0.0, kick=1.0)
    assert info2["kicked"] is False


def test_apply_kick_returns_cooldown_and_capped_speed():
    ball_vel = np.zeros(2)
    new_vel, cd, kicked, speed = apply_kick(
        np.array([0.14, 0.0]), ball_vel, np.array([0.0, 0.0]), 0.0, 1.0, 0)
    assert kicked is True
    assert cd == KICK_COOLDOWN_STEPS
    assert speed <= KICK_MAX_SPEED + 1e-9
    # cooldown active → no kick, counter decrements
    _, cd2, kicked2, _ = apply_kick(
        np.array([0.14, 0.0]), np.zeros(2), np.array([0.0, 0.0]), 0.0, 1.0, 5)
    assert kicked2 is False and cd2 == 4


def test_wall_bounce_flag(phys):
    # Ball at the +x arena wall moving outward → it must bounce and flag it.
    _place(phys, [0.0, 0.5], 0.0, [1.21, 0.5])
    phys._ball_vel = np.array([1.0, 0.0])
    _, info = phys.step(0.0, 0.0, 0.0)
    assert info["ball_wall_bounce"] is True


def test_two_robot_kick_a_and_b():
    p = TwoRobotPhysics()
    p.reset(seed=0)
    p._a_pos = np.array([0.0, 0.0]); p._a_heading = 0.0; p._a_vel = np.zeros(2)
    p._b_pos = np.array([0.8, 0.8]); p._b_vel = np.zeros(2)   # B well clear
    p._ball_pos = np.array([0.14, 0.0]); p._ball_vel = np.zeros(2)
    info = p.step((0.0, 0.0, 0.0, 1.0), ZERO4)
    assert info["kicked_a"] is True and info["kicked_b"] is False
    assert info["kick_speed_a"] == pytest.approx(KICK_MAX_SPEED, rel=1e-6)
    assert p.state_a().ball_vel[0] > 0.0


def test_two_robot_step_accepts_3_tuple_actions():
    # Backwards-compatible: a 3-tuple drive action implies no kick.
    p = TwoRobotPhysics()
    p.reset(seed=0)
    info = p.step((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert info["kicked_a"] is False and info["kicked_b"] is False
