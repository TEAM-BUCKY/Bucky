"""Tests for the 4-beam sonar opponent/wall sensing model."""
import numpy as np

from bucky.selfplay import sonar_ranges, MAX_SONAR_RANGE
from bucky.physics.python_backend import ARENA_HALF_X


def test_sonar_returns_four_normalized_beams():
    s = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=None)
    assert s.shape == (4,)
    assert np.all(s >= 0.0) and np.all(s <= 1.0)


def test_no_opponent_forward_beam_reads_the_wall():
    # No opponent, robot at center facing +x: the forward beam ranges to the +x arena
    # wall. The official field is smaller than the sonar range, so it reads the wall
    # distance (not a saturated "clear" 1.0).
    assert ARENA_HALF_X < MAX_SONAR_RANGE
    s = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=None)
    assert np.isclose(s[0], ARENA_HALF_X / MAX_SONAR_RANGE)


def test_opponent_ahead_lights_forward_beam():
    clear = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=None)
    seen = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=np.array([0.5, 0.0]))
    assert seen[0] < clear[0]          # forward beam now detects the opponent
    assert seen[0] < 1.0


def test_opponent_to_the_left_lights_left_beam_not_forward():
    # Beam order: [forward, left, right, back]. Opponent on robot's left (+y).
    clear = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=None)
    s = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=np.array([0.0, 0.5]))
    assert s[1] < clear[1]             # left beam detects it (closer than the wall)
    assert np.isclose(s[0], clear[0])  # forward beam unaffected (opponent out of its sector)


def test_opponent_distance_scales_reading():
    near = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=np.array([0.4, 0.0]))
    far = sonar_ranges(np.array([0.0, 0.0]), 0.0, opponent_pos=np.array([1.0, 0.0]))
    assert near[0] < far[0]            # closer opponent → smaller reading
