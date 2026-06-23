"""Foundational curriculum stages + single→self-play obs-expansion weight transfer."""
import numpy as np
import pytest

from bucky.curriculum import FULL_TRAINING_PHASES, Stage, get_stage_config
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.obs import OBS_DIM


def test_full_training_phase_plan():
    """FULL_TRAINING runs APPROACH → PUSH → SELF_PLAY with ratios summing to 1."""
    stages = [s for s, _ in FULL_TRAINING_PHASES]
    assert stages == [Stage.APPROACH_STATIC_BALL, Stage.PUSH_TO_EMPTY_GOAL, Stage.SELF_PLAY_1V1]
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
    assert steps == [150, 250, 600]
    assert sum(steps) == total


@pytest.mark.parametrize("stage", [Stage.APPROACH_STATIC_BALL, Stage.PUSH_TO_EMPTY_GOAL])
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


def test_push_stage_includes_goal_and_kick_terms():
    active = set(get_stage_config(Stage.PUSH_TO_EMPTY_GOAL).active_reward_terms)
    for t in ("goal", "predicted_goal", "kick_power_to_goal", "shot_on_goal", "ball_to_goal"):
        assert t in active


def test_obs_expansion_transfer_preserves_shared_weights(tmp_path):
    """A 35-dim single-agent policy transfers into a 39-dim self-play policy: shared input columns
    are copied verbatim, the new sonar columns are left at init, and the model still runs."""
    from stable_baselines3 import PPO
    from bucky.envs.bucky_selfplay import BuckySelfPlayEnv
    from bucky.selfplay import transfer_weights_expand_obs

    single = BuckySingleEnv(stage=Stage.PUSH_TO_EMPTY_GOAL, domain_rand=False)
    old = PPO("MlpPolicy", single, policy_kwargs={"net_arch": [64, 64]}, device="cpu")
    old_path = tmp_path / "old_model"
    old.save(str(old_path))

    selfplay = BuckySelfPlayEnv(domain_rand=False)
    new = PPO("MlpPolicy", selfplay, policy_kwargs={"net_arch": [64, 64]}, device="cpu")

    # Capture the first-layer weights pre-transfer to verify the overlap copy.
    key = "mlp_extractor.policy_net.0.weight"
    old_w = old.policy.state_dict()[key].clone()         # [64, 35]
    new_w_before = new.policy.state_dict()[key].clone()  # [64, 39]
    assert old_w.shape[1] == 35 and new_w_before.shape[1] == 39

    copied, padded = transfer_weights_expand_obs(new, str(old_path), device="cpu")
    assert padded >= 1            # first layer(s) expanded
    assert copied >= 1            # deeper layers copied verbatim

    new_w_after = new.policy.state_dict()[key]
    # Overlapping 35 input columns now equal the old weights; the 4 new columns are untouched.
    assert np.allclose(new_w_after[:, :35].cpu().numpy(), old_w.cpu().numpy())
    assert np.allclose(new_w_after[:, 35:].cpu().numpy(), new_w_before[:, 35:].cpu().numpy())

    # And the transferred model still produces a valid action for a real self-play observation.
    obs, _ = selfplay.reset(seed=0)
    action, _ = new.predict(obs, deterministic=True)
    assert action.shape == (4,)
    assert np.all(np.isfinite(action))
    single.close()
    selfplay.close()
