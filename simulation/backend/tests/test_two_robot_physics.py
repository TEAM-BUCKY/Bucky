"""Tests for the 1v1 two-robot physics stepper."""
import numpy as np
import pytest

from bucky.physics.python_backend import (
    TwoRobotPhysics, FIELD_W, GOAL_WIDTH, ROBOT_RADIUS,
)
from bucky.game.field import in_penalty_area

ZERO = (0.0, 0.0, 0.0)
GOAL_HALF = GOAL_WIDTH / 2


@pytest.fixture
def phys():
    return TwoRobotPhysics()


def test_reset_a_left_b_right(phys):
    phys.reset(seed=0)
    a, b = phys.state_a(), phys.state_b()
    assert a.robot_pos[0] < 0.0           # A defends/-x, attacks +x
    assert b.robot_pos[0] > 0.0           # B defends/+x, attacks -x
    assert np.linalg.norm(phys.state_a().ball_pos) < 0.3   # ball near center


def test_reset_headings_face_opponent_goal(phys):
    phys.reset(seed=0)
    a, b = phys.state_a(), phys.state_b()
    # A faces +x (heading ~0), B faces -x (heading ~±pi)
    assert np.cos(a.robot_heading) > 0.5
    assert np.cos(b.robot_heading) < -0.5


def test_reset_kickoff_a_puts_a_on_the_ball_b_in_black_zone(phys):
    phys.reset(seed=1, kickoff="a")
    a, b = phys.state_a().robot_pos, phys.state_b().robot_pos
    ball = phys.state_a().ball_pos
    assert np.linalg.norm(a - ball) < np.linalg.norm(b - ball)   # A has kickoff control
    assert np.linalg.norm(a - ball) < 0.25
    assert b[0] > 0.0 and in_penalty_area(b, +1)                  # B in its own penalty area (rule 4.4)


def test_reset_kickoff_b_puts_b_on_the_ball_a_in_black_zone(phys):
    phys.reset(seed=1, kickoff="b")
    a, b = phys.state_a().robot_pos, phys.state_b().robot_pos
    ball = phys.state_a().ball_pos
    assert np.linalg.norm(b - ball) < np.linalg.norm(a - ball)   # B has kickoff control
    assert np.linalg.norm(b - ball) < 0.25
    assert a[0] < 0.0 and in_penalty_area(a, -1)                  # A in its own penalty area (rule 4.4)


def test_step_returns_goal_flags(phys):
    phys.reset(seed=0)
    info = phys.step(ZERO, ZERO)
    assert "goal_a" in info and "goal_b" in info and "ball_out" in info


def test_goal_a_when_ball_in_plus_x_goal(phys):
    phys.reset(seed=0)
    phys._ball_pos = np.array([FIELD_W / 2 + 0.05, 0.0])
    phys._ball_vel = np.zeros(2)
    info = phys.step(ZERO, ZERO)
    assert info["goal_a"] is True
    assert info["goal_b"] is False


def test_goal_b_when_ball_in_minus_x_goal(phys):
    phys.reset(seed=0)
    phys._ball_pos = np.array([-(FIELD_W / 2 + 0.05), 0.0])
    phys._ball_vel = np.zeros(2)
    info = phys.step(ZERO, ZERO)
    assert info["goal_b"] is True
    assert info["goal_a"] is False


@pytest.mark.parametrize("goal_sign", [+1, -1])
@pytest.mark.parametrize("y_sign", [+1, -1])
def test_ball_cannot_enter_goal_from_the_side(phys, goal_sign, y_sign):
    """A ball loose in the neutral band behind a goal line, outside the goal mouth,
    must not slip *laterally* into the goal strip and score — the goal has side walls,
    so the only way in is through the mouth at the goal line."""
    phys.reset(seed=0)
    phys._a_pos = np.array([0.0, 0.0])      # robots parked at centre, well clear of the goal
    phys._b_pos = np.array([0.0, 0.3])
    # Ball behind the goal line (|x| > HALF_W) but outside the mouth, drifting laterally
    # toward the goal centre line.
    phys._ball_pos = np.array([goal_sign * 1.0, y_sign * 0.30])
    phys._ball_vel = np.array([0.0, -y_sign * 0.5])
    for _ in range(80):
        info = phys.step(ZERO, ZERO)
        assert info["goal_a"] is False, "ball scored in +x goal from the side"
        assert info["goal_b"] is False, "ball scored in -x goal from the side"
        bx, by = phys.state_a().ball_pos
        if abs(bx) > FIELD_W / 2:           # while behind a goal line it must stay
            assert abs(by) >= GOAL_HALF - 1e-6  # out of the goal strip (blocked by side wall)


def test_ball_still_scores_through_the_mouth(phys):
    """Regression guard: the side walls must not block a legitimate goal scored by a ball
    entering through the mouth off-centre."""
    phys.reset(seed=0)
    phys._a_pos = np.array([0.0, 0.5])
    phys._b_pos = np.array([0.0, -0.5])
    phys._ball_pos = np.array([FIELD_W / 2 - 0.05, 0.1])   # just in front of the mouth, off-centre
    phys._ball_vel = np.array([3.0, 0.0])                  # driven straight through the opening
    info = phys.step(ZERO, ZERO)
    assert info["goal_a"] is True


def test_robot_a_moves_forward(phys):
    phys.reset(seed=0)
    phys._a_pos = np.array([0.0, 0.0])
    phys._a_heading = 0.0
    phys._a_vel = np.zeros(2)
    phys._b_pos = np.array([1.0, 1.0])      # keep B clear so we isolate A's drive
    phys._ball_pos = np.array([1.0, -1.0])
    phys.step((1.0, 0.0, 0.0), ZERO)
    assert phys.state_a().robot_pos[0] > 0.0


def test_robots_collide_and_separate(phys):
    phys.reset(seed=0)
    phys._a_pos = np.array([-0.05, 0.0])
    phys._b_pos = np.array([0.05, 0.0])       # overlapping (dist 0.1 < 0.22)
    phys._a_vel = np.zeros(2)
    phys._b_vel = np.zeros(2)
    phys.step(ZERO, ZERO)
    dist = np.linalg.norm(phys.state_a().robot_pos - phys.state_b().robot_pos)
    assert dist >= 2 * ROBOT_RADIUS - 1e-3


def test_ball_pushed_by_robot_b(phys):
    phys.reset(seed=0)
    phys._b_pos = np.array([0.0, 0.0])
    phys._b_heading = 0.0
    phys._b_vel = np.zeros(2)
    phys._ball_pos = np.array([0.12, 0.0])
    phys._ball_vel = np.zeros(2)
    phys.step(ZERO, (1.0, 0.0, 0.0))
    moved = (np.linalg.norm(phys.state_b().ball_pos) > 0.0
             and phys.state_b().ball_pos[0] != 0.12)
    assert np.linalg.norm(phys.state_b().ball_vel) > 0 or moved
