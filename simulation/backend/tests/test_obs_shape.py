import numpy as np
import pytest
from bucky.obs import OBS_DIM, build_observation
from bucky.physics.python_backend import PyPhysics

@pytest.fixture
def state():
    p = PyPhysics()
    return p.reset(seed=0)

def test_obs_dim_constant():
    assert OBS_DIM == 17

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
