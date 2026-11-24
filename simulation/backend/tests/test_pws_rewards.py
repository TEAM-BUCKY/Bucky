"""PWS reward profile (§9.3): integer weights, normalised terms, extensible registry."""
import numpy as np
import pytest

from bucky.curriculum import Stage
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.physics.backend import PhysicsState
from bucky.reward_terms import PROFILES, TERMS, reward_term, unregister
from bucky.reward_terms.pws import ball_distance_potential, ball_position_potential
from bucky.rewards import RewardConfig, RewardTerms, compute_rewards

PWS_FIVE = ["goal", "ball_distance", "ball_position", "kick_goal", "smoothness"]


def _state(robot=(0.0, 0.0), ball=(0.3, 0.0), ball_vel=(0.0, 0.0)):
    return PhysicsState(robot_pos=np.array(robot, float), robot_vel=np.zeros(2),
                        robot_heading=0.0, robot_omega=0.0,
                        ball_pos=np.array(ball, float), ball_vel=np.array(ball_vel, float))


def test_default_profile_is_pws_with_integer_weights():
    cfg = RewardConfig()
    assert cfg.profile == "pws"
    w = cfg.weight_dict()
    for name in PWS_FIVE + ["goal_against", "out_of_bounds", "defective", "lack_of_progress"]:
        assert isinstance(w[f"w_{name}"], int)
    assert cfg.w_goal == 10 and cfg.w_kick_goal == 5
    assert "w_front_align" not in w


def test_non_integer_weight_rejected_but_integer_floats_ok():
    assert RewardConfig.from_dict({"w_goal": 12.0}).w_goal == 12
    with pytest.raises(ValueError):
        RewardConfig.from_dict({"w_goal": 1.5})


def test_unknown_and_legacy_keys_are_ignored():
    cfg = RewardConfig.from_dict({"w_front_align": 5.5, "w_goal": 7})
    assert cfg.w_goal == 7


def test_goal_and_kick_goal_events():
    cfg = RewardConfig()
    s = _state()
    t = compute_rewards(s, s, cfg, {"goal_scored": True, "kicked_goal": True})
    assert t.goal == 10 and t.kick_goal == 5
    t = compute_rewards(s, s, cfg, {"goal_scored": True, "kicked_goal": False})
    assert t.goal == 10 and t.kick_goal == 0
    t = compute_rewards(s, s, cfg, {"goal_against": True, "out_of_bounds": True})
    assert t.goal_against == -10 and t.out_of_bounds == -5


def test_ball_distance_rewards_closing_in_and_telescopes():
    cfg = RewardConfig()
    far, near = _state(robot=(-0.6, 0.0)), _state(robot=(0.0, 0.0))
    assert compute_rewards(far, near, cfg, {}).ball_distance > 0
    assert compute_rewards(near, far, cfg, {}).ball_distance < 0
    # Potential-based → the episode sum is Φ(end) − Φ(start), bounded by [-1, 1] × weight.
    assert 0.0 <= ball_distance_potential((-1.2, -0.9), (1.2, 0.9)) <= 1.0


def test_ball_position_rewards_ball_toward_opponent_goal():
    cfg = RewardConfig()
    a, b = _state(ball=(0.0, 0.0)), _state(ball=(0.5, 0.0))
    assert compute_rewards(a, b, cfg, {}).ball_position > 0
    assert compute_rewards(b, a, cfg, {}).ball_position < 0
    assert -0.5 <= ball_position_potential((-0.915, 0.0)) <= ball_position_potential((0.915, 0.0)) <= 0.5


def test_smoothness_episode_sum_is_bounded():
    cfg = RewardConfig()
    s = _state()
    worst = 0.0
    a_prev = np.array([-1, -1, -1, 0], float)
    for i in range(1500):
        a = -a_prev                                  # maximal flip every step
        worst += compute_rewards(s, s, cfg, {"max_steps": 1500}, a, a_prev).smoothness
        a_prev = a
    assert worst == pytest.approx(-cfg.w_smoothness, rel=1e-6)   # exactly the weight, no more


def test_registering_a_new_term_extends_config_and_total():
    @reward_term("pws_test_bonus", weight=3, group="pws", description="test")
    def _bonus(ctx):
        return 1.0

    try:
        cfg = RewardConfig()
        assert cfg.weight_dict()["w_pws_test_bonus"] == 3
        s = _state()
        t = compute_rewards(s, s, cfg, {})
        assert t.as_dict()["pws_test_bonus"] == 3
        assert t.total == pytest.approx(sum(t.as_dict().values()))
    finally:
        unregister("pws_test_bonus")
    assert "pws_test_bonus" not in TERMS and "pws_test_bonus" not in PROFILES["pws"]


def test_legacy_profile_still_selectable():
    cfg = RewardConfig.from_dict({"w_goal": 75.5}, profile="legacy")
    assert cfg.uses_predicted_goal and cfg.w_goal == 75.5
    s = _state()
    t = compute_rewards(s, s, cfg, {"goal_scored": True})
    assert t.goal == 75.5 and "front_alignment" in t.as_dict()


def test_reward_terms_attribute_access():
    t = RewardTerms({"goal": 2.0, "smoothness": -1.0})
    assert t.goal == 2.0 and t.total == 1.0
    with pytest.raises(AttributeError):
        _ = t.nonexistent


def test_env_reward_equals_sum_of_active_pws_terms():
    env = BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False)
    env.reset(seed=0)
    rng = np.random.default_rng(0)
    for _ in range(50):
        _, r, term, trunc, info = env.step(rng.uniform(-1, 1, 4).astype(np.float32))
        active = env._stage_cfg.active_reward_terms
        assert r == pytest.approx(sum(info["reward_terms"][k] for k in active))
        assert set(info["reward_terms"]) == set(PROFILES["pws"])
        if term or trunc:
            env.reset()
    env.close()
