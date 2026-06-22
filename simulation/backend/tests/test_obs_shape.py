import numpy as np
import pytest
from bucky.game import field
from bucky.obs import OBS_DIM, build_observation
from bucky.physics.python_backend import PyPhysics

@pytest.fixture
def state():
    p = PyPhysics()
    return p.reset(seed=0)

def test_obs_dim_constant():
    assert OBS_DIM == 35


def _place(physics, robot_pos, heading, ball_pos, ball_vel=(0.0, 0.0)):
    physics._robot_pos = np.asarray(robot_pos, dtype=float)
    physics._robot_heading = float(heading)
    physics._ball_pos = np.asarray(ball_pos, dtype=float)
    physics._ball_vel = np.asarray(ball_vel, dtype=float)
    return physics._make_state()


def test_obs_boundary_features_in_bounds():
    # Every dim must stay within the observation_space clip range used by the env.
    p = PyPhysics(); p.reset(seed=0)
    for bx, by in [(0.0, 0.0), (0.9, 0.55), (1.1, 0.0), (0.0, -0.8)]:
        s = _place(p, [0.0, 0.0], 0.0, [bx, by], [2.0, 1.0])
        obs = build_observation(s)
        assert np.all(np.isfinite(obs))
        assert np.all(np.abs(obs) <= 3.0), f"out-of-range obs for ball ({bx},{by}): {obs}"


def test_obs_ball_out_flag():
    p = PyPhysics(); p.reset(seed=0)
    inside = build_observation(_place(p, [0.0, 0.0], 0.0, [0.0, 0.0]))
    assert inside[22] == 0.0
    out = build_observation(_place(p, [0.0, 0.0], 0.0, [1.1, 0.4]))   # past +x white line, outside mouth
    assert out[22] == 1.0


def test_obs_respawn_points_at_nearest_neutral_spot():
    # Ball near the +x penalty-area inner corner: respawn vector (robot frame, heading 0 so robot
    # frame == world) must point from the robot to that neutral spot.
    p = PyPhysics(); p.reset(seed=0)
    spot = field.NEUTRAL_SPOTS[1]                       # [0.465, 0.225]
    ball = spot + np.array([0.03, 0.02])
    robot = np.array([0.0, 0.0])
    obs = build_observation(_place(p, robot, 0.0, ball))
    diag = (field.FIELD_W**2 + field.FIELD_H**2) ** 0.5
    expected = (spot - robot) / diag
    assert np.allclose(obs[20:22], expected, atol=1e-6)


def test_obs_ball_vel_toward_line_sign():
    # Ball near the +x line moving outward (+x) → positive; moving inward (−x) → negative.
    p = PyPhysics(); p.reset(seed=0)
    outward = build_observation(_place(p, [0.0, 0.0], 0.0, [0.8, 0.0], [3.0, 0.0]))
    inward = build_observation(_place(p, [0.0, 0.0], 0.0, [0.8, 0.0], [-3.0, 0.0]))
    assert outward[19] > 0.0
    assert inward[19] < 0.0

def test_obs_kick_prediction_block_on_target():
    # Robot behind the ball facing +x: a kick fired now flies straight into the opponent goal.
    # The kick-prediction block [23:35] must flag the 1st contact as a goal and leave the 2nd empty.
    p = PyPhysics(); p.reset(seed=0)
    obs = build_observation(_place(p, [-0.2, 0.0], 0.0, [0.0, 0.0]))
    first, second = obs[23:29], obs[29:35]
    assert first[0] == 1.0 and first[1] == 0.0 and first[2] == 0.0      # 1st contact = goal
    assert first[5] > 0.0                                                # positive distance to it
    assert np.all(second == 0.0)                                         # goal is terminal


def test_obs_shape(state):
    obs = build_observation(state)
    assert obs.shape == (OBS_DIM,)

def test_obs_dtype(state):
    obs = build_observation(state)
    assert obs.dtype == np.float32

def test_obs_roughly_normalized(state):
    obs = build_observation(state)
    assert np.all(np.abs(obs) < 5.0), f"Extreme values: {obs}"

def test_obs_sin_cos_unit_norm(state):
    obs = build_observation(state)
    # indices 0,1 = ball bearing sin,cos
    assert abs(obs[0]**2 + obs[1]**2 - 1.0) < 1e-5

def test_obs_with_noise(state):
    obs_clean = build_observation(state, add_noise=False)
    obs_noisy = build_observation(state, add_noise=True, rng=np.random.default_rng(1))
    assert not np.allclose(obs_clean, obs_noisy)
    assert np.max(np.abs(obs_clean - obs_noisy)) < 0.3
