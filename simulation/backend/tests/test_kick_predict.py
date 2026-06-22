"""Tests for the predicted kick-outcome features (bucky.kick_predict)."""
import numpy as np
import pytest

from bucky.game import field
from bucky.kick_predict import N_KICK_PRED_FEATURES, predict_kick_outcome

DIAG = (field.FIELD_W**2 + field.FIELD_H**2) ** 0.5
IDENTITY = np.eye(2)        # world→robot when heading == 0 and robot at origin


def _predict(ball_pos, heading, robot_pos=(0.0, 0.0), opponent_pos=None):
    R = np.array([[np.cos(heading), np.sin(heading)], [-np.sin(heading), np.cos(heading)]])
    return predict_kick_outcome(np.asarray(ball_pos, float), heading, R,
                                np.asarray(robot_pos, float), opponent_pos,
                                field_diag=DIAG)


def _split(block):
    return block[:6], block[6:]


def test_block_width():
    block = _predict([0.0, 0.0], 0.0)
    assert block.shape == (N_KICK_PRED_FEATURES,)
    assert block.dtype == np.float32


def test_straight_shot_hits_goal_and_is_terminal():
    first, second = _split(_predict([0.0, 0.0], 0.0))
    assert (first[0], first[1], first[2]) == (1.0, 0.0, 0.0)   # goal
    assert first[5] > 0.0                                       # positive distance
    assert np.all(second == 0.0)                               # goal terminal → no 2nd contact


def test_shot_at_side_wall_bounces_to_a_second_wall():
    # Straight up: hits the +y arena wall, reflects down, hits the −y arena wall.
    first, second = _split(_predict([0.0, 0.0], np.pi / 2))
    assert first[2] == 1.0 and first[0] == 0.0 and first[1] == 0.0    # 1st = wall
    assert second[2] == 1.0                                            # 2nd = wall
    assert second[5] > first[5] > 0.0                                  # cumulative distance grows


def test_opponent_on_the_shot_line_is_a_robot_contact():
    # Opponent sits between the ball and the goal, on the shot ray → struck first.
    first, _ = _split(_predict([0.0, 0.0], 0.0, opponent_pos=[0.4, 0.0]))
    assert (first[0], first[1], first[2]) == (0.0, 1.0, 0.0)   # robot
    # Impact is at the opponent's near surface, before the goal line.
    assert 0.0 < first[3] * DIAG < 0.4


def test_no_opponent_means_no_robot_contact():
    block = _predict([0.0, 0.0], 0.0, opponent_pos=None)
    assert block[1] == 0.0 and block[7] == 0.0                 # is_robot never set


def test_impact_is_in_robot_frame_forward_is_positive_x():
    # Robot facing −x with the goal behind it in world terms: the −x goal is *in front*, so the
    # impact's robot-frame x must be positive (ahead of the robot), proving the frame rotation.
    first, _ = _split(_predict([0.0, 0.0], np.pi))
    assert first[0] == 1.0                                     # still a goal (the −x mouth)
    assert first[3] > 0.0                                      # in front of the robot
    assert abs(first[4]) < 1e-6                                # straight ahead → ~zero lateral


@pytest.mark.parametrize("heading", [0.0, np.pi / 2, np.pi, -np.pi / 3, 2.1])
def test_always_finds_a_first_contact_in_bounds(heading):
    block = _predict([0.1, -0.05], heading)
    first, _ = _split(block)
    assert first[0] + first[1] + first[2] == 1.0              # exactly one object type set
    assert 0.0 < first[5] < 2.0                               # normalized distance is sane
