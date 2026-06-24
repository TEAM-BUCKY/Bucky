"""The eval tool drives any legacy obs width by projecting the full 43-dim obs to the policy.

Single-agent widths {18, 23, 35, 39} are pure base prefixes (no sonar); opponent-aware widths
{22, 27, 39-legacy, 43} are a base prefix + the 4-beam sonar tail. The one width shared by two
eras (39) is disambiguated by the caller (eval_drill) — here we test both interpretations.
"""
import numpy as np
import pytest

from bucky.obs import LEGACY_OBS_DIM, OBS_DIM, PRE_GOAL_REL_OBS_DIM, PRE_KICK_PRED_OBS_DIM
from bucky.selfplay import (
    EVAL_COMPATIBLE_OBS_DIMS,
    SELF_PLAY_OBS_DIM,
    SINGLE_COMPATIBLE_OBS_DIMS,
    adapt_obs_to_policy,
    project_obs,
)

FULL = np.arange(SELF_PLAY_OBS_DIM, dtype=np.float32)  # a 43-dim "full" obs, 0..42


@pytest.mark.parametrize("dim", [LEGACY_OBS_DIM, PRE_KICK_PRED_OBS_DIM, PRE_GOAL_REL_OBS_DIM, OBS_DIM])
def test_single_agent_widths_are_pure_prefixes(dim):
    out = project_obs(FULL, dim, opponent_aware=False)
    assert out.shape == (dim,)
    assert np.array_equal(out, np.arange(dim))  # no sonar — just the base prefix


@pytest.mark.parametrize("dim", [22, 27, 39, SELF_PLAY_OBS_DIM])
def test_opponent_aware_widths_match_match_projection(dim):
    # Opponent-aware projection is exactly the (battle-tested) match projection.
    assert np.array_equal(project_obs(FULL, dim, opponent_aware=True), adapt_obs_to_policy(FULL, dim))


def test_thirty_nine_is_disambiguated_by_opponent_aware_flag():
    # Same width, two layouts: single 39 keeps [0:39]; legacy self-play 39 is [0:35] + sonar.
    single = project_obs(FULL, OBS_DIM, opponent_aware=False)
    selfplay = project_obs(FULL, OBS_DIM, opponent_aware=True)
    assert np.array_equal(single, np.arange(OBS_DIM))
    assert single.shape == selfplay.shape == (OBS_DIM,)
    assert not np.array_equal(single, selfplay)  # the last 4 dims differ (goal-rel vs sonar)
    assert np.array_equal(selfplay[-4:], np.arange(OBS_DIM, SELF_PLAY_OBS_DIM))  # sonar tail


def test_eval_supports_every_legacy_width():
    assert EVAL_COMPATIBLE_OBS_DIMS == SINGLE_COMPATIBLE_OBS_DIMS | {22, 27, 39, SELF_PLAY_OBS_DIM}
    assert SINGLE_COMPATIBLE_OBS_DIMS == {18, 23, 35, 39}


def test_unsupported_single_width_raises():
    with pytest.raises(ValueError):
        project_obs(FULL, 30, opponent_aware=False)
