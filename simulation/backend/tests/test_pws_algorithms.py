"""PWS §9 learned algorithms: C51, DSAC and the FSM + RL hybrid."""
import json

import numpy as np
import pytest
import torch as th

from bucky.algos import registry
from bucky.curriculum import Stage
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.envs.discrete_wrapper import DiscreteActionWrapper
from bucky.envs.fsm_hybrid_wrapper import N_MODES, FSMHybridObsWrapper
from bucky.policies.loader import load_policy
from bucky.selfplay import SELF_PLAY_OBS_DIM, export_policy_npz, load_numpy_opponent

SMALL = {"buffer_size": 2_000, "learning_starts": 64, "batch_size": 32}


def test_new_algos_registered():
    for name in ("c51", "dsac", "fsm_ppo", "fsm_sac"):
        assert name in registry.available()
    assert registry.get_spec("c51").action_kind == "discrete"
    assert registry.get_spec("c51").env_wrapper == "discrete"
    assert registry.get_spec("dsac").supports_selfplay is True
    assert registry.get_spec("fsm_ppo").env_wrapper == "fsm_hybrid"


# ── C51 ─────────────────────────────────────────────────────────────────────
def test_categorical_projection_conserves_mass_and_shifts():
    from bucky.algos.c51 import categorical_projection
    support = th.linspace(-10, 10, 51)
    rng = np.random.default_rng(0)
    p = th.tensor(rng.dirichlet(np.ones(51), size=8), dtype=th.float32)
    r = th.tensor(rng.uniform(-3, 3, (8, 1)), dtype=th.float32)
    d = th.tensor(rng.integers(0, 2, (8, 1)), dtype=th.float32)
    proj = categorical_projection(p, r, d, 0.99, support)
    assert th.allclose(proj.sum(1), th.ones(8), atol=1e-5)
    assert (proj >= -1e-7).all()
    # A terminal transition collapses onto the reward: the projected mean equals r.
    mean = (proj * support).sum(1, keepdim=True)
    assert th.allclose(mean[d.bool()], r[d.bool()], atol=1e-4)
    # Exactly on an atom (r = 0, done) → all mass on that atom.
    one = categorical_projection(p[:1], th.zeros(1, 1), th.ones(1, 1), 0.99, support)
    assert float(one[0, 25]) == pytest.approx(1.0, abs=1e-6)


def test_c51_learns_saves_and_plays(tmp_path):
    env = DiscreteActionWrapper(BuckySingleEnv(stage=Stage.APPROACH_STATIC_BALL, domain_rand=False))
    spec = registry.get_spec("c51")
    model = spec.build(env=env, params={**SMALL, "n_atoms": 21}, net_arch=[32, 32], seed=0,
                       device="cpu", tensorboard_log=None, ent_coef=0.0)
    assert model.q_net.n_atoms == 21
    model.learn(400)
    path = tmp_path / "run" / "model.zip"
    path.parent.mkdir()
    model.save(str(path))
    (path.parent / "config.json").write_text(json.dumps({"algo": "c51"}))
    pol = load_policy(str(path))               # → DiscretePolicyAdapter
    act, _ = pol.predict(np.zeros(SELF_PLAY_OBS_DIM, np.float32))
    assert np.asarray(act).shape == (4,)


# ── DSAC ────────────────────────────────────────────────────────────────────
def test_risk_measures_order():
    from bucky.algos.dsac import risk_measure
    q = th.arange(20, dtype=th.float32).reshape(1, 2, 10)
    low = risk_measure(q, "cvar", 0.25)
    mid = risk_measure(q, "neutral")
    high = risk_measure(q, "optimistic", 0.25)
    assert float(low) < float(mid) < float(high)
    with pytest.raises(ValueError):
        risk_measure(q, "yolo")


@pytest.mark.parametrize("risk", ["neutral", "cvar"])
def test_dsac_learns_and_exports(tmp_path, risk):
    from tests.test_contrib_algos import _dummy_env
    spec = registry.get_spec("dsac")
    model = spec.build(env=_dummy_env(), params={**SMALL, "risk": risk, "n_quantiles": 5},
                       net_arch=[32, 32], seed=0, device="cpu", tensorboard_log=None, ent_coef="auto")
    assert model.top_quantiles_to_drop_per_net == 0 and model.risk == risk
    model.learn(200)
    path = str(tmp_path / "dsac.zip")
    model.save(path)
    loaded = spec.load(path, device="cpu")
    assert loaded.risk == risk
    npz = str(tmp_path / "dsac.npz")
    export_policy_npz(model, npz)              # self-play opponent export works like TQC
    opp = load_numpy_opponent(npz)
    obs = np.zeros(SELF_PLAY_OBS_DIM, np.float32)
    assert np.allclose(model.predict(obs, deterministic=True)[0], opp.predict(obs)[0], atol=1e-4)


# ── FSM + RL hybrid ─────────────────────────────────────────────────────────
def test_fsm_hybrid_wrapper_prepends_mode():
    env = FSMHybridObsWrapper(BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False))
    obs, _ = env.reset(seed=0)
    assert obs.shape == (N_MODES + 39,)
    assert obs[:N_MODES].sum() == 1.0
    obs, *_ = env.step(np.zeros(4, np.float32))
    assert env.observation_space.contains(obs)


def test_fsm_ppo_trains_and_plays(tmp_path):
    env = FSMHybridObsWrapper(BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False))
    spec = registry.get_spec("fsm_ppo")
    model = spec.build(env=env, params={"n_steps": 64, "batch_size": 64, "n_epochs": 1},
                       net_arch=[32, 32], seed=0, device="cpu", tensorboard_log=None, ent_coef=0.0)
    model.learn(128)
    path = tmp_path / "run" / "model.zip"
    path.parent.mkdir()
    model.save(str(path))
    (path.parent / "config.json").write_text(json.dumps({"algo": "fsm_ppo"}))
    from bucky.policies.hybrid import HybridPolicy
    pol = load_policy(str(path))
    assert isinstance(pol, HybridPolicy)
    act, _ = pol.predict(np.zeros(SELF_PLAY_OBS_DIM, np.float32))   # 43-dim match obs
    assert act.shape == (4,) and np.all(np.abs(act) <= 1.0)
