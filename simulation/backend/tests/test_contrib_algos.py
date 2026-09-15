"""sb3-contrib algorithms register correctly and TQC exports a matching numpy opponent."""
import numpy as np
import pytest

from bucky.algos import registry
from bucky.selfplay import SELF_PLAY_OBS_DIM, export_policy_npz, load_numpy_opponent


def test_contrib_algos_registered():
    for name in ("tqc", "trpo", "ars", "qrdqn"):
        assert name in registry.available()
    assert registry.get_spec("qrdqn").action_kind == "discrete"
    assert registry.get_spec("qrdqn").env_wrapper == "discrete"
    assert registry.get_spec("tqc").supports_selfplay is True
    assert registry.get_spec("ars").supports_selfplay is False


def _dummy_env(act_dim=4):
    import gymnasium as gym
    from gymnasium import spaces

    class Dummy(gym.Env):
        observation_space = spaces.Box(-3.0, 3.0, (SELF_PLAY_OBS_DIM,), np.float32)
        action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)

        def reset(self, *a, **k):
            return self.observation_space.sample(), {}

        def step(self, a):
            return self.observation_space.sample(), 0.0, True, False, {}

    return Dummy()


def test_tqc_numpy_export_parity(tmp_path):
    spec = registry.get_spec("tqc")
    model = spec.build(env=_dummy_env(), params={"buffer_size": 1000}, net_arch=[64, 64],
                       seed=0, device="cpu", tensorboard_log=None, ent_coef="auto")
    npz = str(tmp_path / "tqc.npz")
    export_policy_npz(model, npz)
    data = np.load(npz)
    assert str(data["activation"]) == "relu" and bool(data["squash"]) is True
    opp = load_numpy_opponent(npz)
    rng = np.random.default_rng(0)
    for _ in range(150):
        obs = rng.uniform(-3.0, 3.0, SELF_PLAY_OBS_DIM).astype(np.float32)
        ref, _ = model.predict(obs, deterministic=True)
        got, _ = opp.predict(obs, deterministic=True)
        assert np.allclose(ref, got, atol=1e-4)


@pytest.mark.parametrize("algo", ["trpo"])
def test_onpolicy_contrib_export(tmp_path, algo):
    spec = registry.get_spec(algo)
    model = spec.build(env=_dummy_env(), params={"n_steps": 64}, net_arch=[64, 64],
                       seed=0, device="cpu", tensorboard_log=None, ent_coef=0.0)
    npz = str(tmp_path / f"{algo}.npz")
    export_policy_npz(model, npz)   # ActorCriticPolicy → tanh, no squash
    data = np.load(npz)
    assert bool(data["squash"]) is False
