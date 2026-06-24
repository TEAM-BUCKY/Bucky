"""Foundational curriculum stages + single→self-play obs-expansion weight transfer."""
import numpy as np
import pytest

from bucky.curriculum import FULL_TRAINING_PHASES, Stage, get_stage_config
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.obs import OBS_DIM


def test_full_training_phase_plan():
    """FULL_TRAINING runs APPROACH → PUSH → AIM_AND_KICK → SELF_PLAY with ratios summing to 1."""
    stages = [s for s, _ in FULL_TRAINING_PHASES]
    assert stages == [Stage.APPROACH_STATIC_BALL, Stage.PUSH_TO_EMPTY_GOAL,
                      Stage.AIM_AND_KICK, Stage.SELF_PLAY_1V1]
    assert sum(r for _, r in FULL_TRAINING_PHASES) == 1.0
    # The meta-stage has a sane (self-play-mirroring) config so incidental lookups don't crash.
    assert get_stage_config(Stage.FULL_TRAINING).opponent_present is True


def test_step_split_distributes_total_with_remainder_to_last():
    """The step-budget split (mirrors train.py): round each non-last phase, remainder to the last."""
    total = 1000
    ratios = [r for _, r in FULL_TRAINING_PHASES]
    steps, remaining = [], total
    for i, r in enumerate(ratios):
        is_last = i == len(ratios) - 1
        s = remaining if is_last else int(round(total * r))
        remaining -= s
        steps.append(s)
    assert steps == [100, 200, 250, 450]
    assert sum(steps) == total


@pytest.mark.parametrize(
    "stage", [Stage.APPROACH_STATIC_BALL, Stage.PUSH_TO_EMPTY_GOAL, Stage.AIM_AND_KICK]
)
def test_foundational_stage_runs(stage):
    cfg = get_stage_config(stage)
    assert cfg.opponent_present is False
    env = BuckySingleEnv(stage=stage, domain_rand=False)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,)                 # 35-dim single-agent obs
    obs, r, term, trunc, info = env.step(np.zeros(4, dtype=np.float32))
    assert np.isfinite(r)
    # Every active reward term exists in the produced breakdown (so none are silently filtered out).
    active = set(env._stage_cfg.active_reward_terms)
    assert active.issubset(set(info["reward_terms"]))
    env.close()


def test_approach_stage_excludes_goal_and_kick_terms():
    """The approach drill should reward reaching the ball, not scoring/kicking."""
    active = set(get_stage_config(Stage.APPROACH_STATIC_BALL).active_reward_terms)
    assert "approach" in active
    for kick_term in ("goal", "predicted_goal", "kick_power_to_goal", "kick_goal"):
        assert kick_term not in active


def test_aim_and_kick_stage_drills_shooting_not_possession():
    cfg = get_stage_config(Stage.AIM_AND_KICK)
    active = set(cfg.active_reward_terms)
    assert cfg.spawn_mode == "kick_blend"
    assert "possession" not in active            # shoot, don't camp
    for kick_term in ("goal", "predicted_goal", "kick_power_to_goal", "kick_goal", "kick_attempt"):
        assert kick_term in active


def test_shoot_spawn_puts_ball_on_centre_line_robot_own_half():
    """PUSH's 'shoot' spawn: ball near the centre line at varied y, robot behind it on its own half."""
    from bucky.game.field import HALF_W
    env = BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False)
    ys = []
    for i in range(200):
        env.reset(seed=i)
        b, r = env._physics._ball_pos, env._physics._robot_pos
        assert abs(b[0]) <= 0.11                 # ball on/near centre line
        assert r[0] < 0                          # robot on its own (-x) half
        assert r[0] < b[0]                       # behind the ball (so a forward drive points goalward)
        assert abs(b[0]) < HALF_W
        ys.append(float(b[1]))
    assert max(ys) - min(ys) > 0.6               # ball genuinely varies left↔right
    env.close()


def test_kick_blend_spawn_places_robot_behind_ball():
    from bucky.game.field import ROBOT_RADIUS, BALL_RADIUS
    env = BuckySingleEnv(stage=Stage.AIM_AND_KICK, domain_rand=False)
    for i in range(200):
        env.reset(seed=i)
        b, r = env._physics._ball_pos, env._physics._robot_pos
        d = float(np.linalg.norm(b - r))
        assert r[0] <= b[0] + 1e-6               # robot on the goal-opposite side of the ball
        assert d >= ROBOT_RADIUS + BALL_RADIUS - 1e-6   # not spawned overlapping the ball
    env.close()


def test_push_stage_includes_goal_and_kick_terms():
    active = set(get_stage_config(Stage.PUSH_TO_EMPTY_GOAL).active_reward_terms)
    for t in ("goal", "predicted_goal", "kick_power_to_goal", "shot_on_goal", "ball_to_goal"):
        assert t in active


def test_obs_expansion_transfer_preserves_shared_weights(tmp_path):
    """A single-agent policy (OBS_DIM cols) transfers into a self-play policy (SELF_PLAY_OBS_DIM
    cols): shared input columns are copied verbatim, the new sonar columns are left at init, and
    the model still runs."""
    from stable_baselines3 import PPO
    from bucky.envs.bucky_selfplay import BuckySelfPlayEnv
    from bucky.selfplay import SELF_PLAY_OBS_DIM, transfer_weights_expand_obs

    single = BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False)
    old = PPO("MlpPolicy", single, policy_kwargs={"net_arch": [64, 64]}, device="cpu")
    old_path = tmp_path / "old_model"
    old.save(str(old_path))

    selfplay = BuckySelfPlayEnv(domain_rand=False)
    new = PPO("MlpPolicy", selfplay, policy_kwargs={"net_arch": [64, 64]}, device="cpu")

    # Capture the first-layer weights pre-transfer to verify the overlap copy.
    key = "mlp_extractor.policy_net.0.weight"
    old_w = old.policy.state_dict()[key].clone()         # [64, OBS_DIM]
    new_w_before = new.policy.state_dict()[key].clone()  # [64, SELF_PLAY_OBS_DIM]
    assert old_w.shape[1] == OBS_DIM and new_w_before.shape[1] == SELF_PLAY_OBS_DIM

    copied, padded = transfer_weights_expand_obs(new, str(old_path), device="cpu")
    assert padded >= 1            # first layer(s) expanded
    assert copied >= 1            # deeper layers copied verbatim

    new_w_after = new.policy.state_dict()[key]
    # Overlapping OBS_DIM input columns now equal the old weights; the new columns are untouched.
    assert np.allclose(new_w_after[:, :OBS_DIM].cpu().numpy(), old_w.cpu().numpy())
    assert np.allclose(new_w_after[:, OBS_DIM:].cpu().numpy(), new_w_before[:, OBS_DIM:].cpu().numpy())

    # And the transferred model still produces a valid action for a real self-play observation.
    obs, _ = selfplay.reset(seed=0)
    action, _ = new.predict(obs, deterministic=True)
    assert action.shape == (4,)
    assert np.all(np.isfinite(action))
    single.close()
    selfplay.close()
