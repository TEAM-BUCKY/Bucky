"""Backward compatibility: an older, narrower self-play policy can still play a current match.

The current opponent-aware obs is [base (OBS_DIM) | sonar (4)] = SELF_PLAY_OBS_DIM, and the base
grew by a ball-boundary block inserted between the old base and the sonar tail. An older policy
saw [pre-boundary base | sonar]; the match projects the current obs down to that layout so old
and new networks can be matched against each other.
"""
import numpy as np
import pytest

from bucky.match import MatchEngine
from bucky.obs import LEGACY_OBS_DIM, OBS_DIM
from bucky.selfplay import (
    MATCH_COMPATIBLE_OBS_DIMS,
    SELF_PLAY_OBS_DIM,
    CompatPolicy,
    adapt_obs_to_policy,
)

LEGACY_22 = LEGACY_OBS_DIM + 4   # pre-boundary base + sonar


class RecordingModel:
    """Records the width of every observation it is asked to predict on."""

    def __init__(self) -> None:
        self.seen_dims: list[int] = []

    def predict(self, obs, **kwargs):
        self.seen_dims.append(int(np.asarray(obs).shape[0]))
        return np.zeros(4, dtype=np.float32), None


def test_adapt_is_identity_for_native_dim():
    obs = np.arange(SELF_PLAY_OBS_DIM, dtype=np.float32)
    assert np.array_equal(adapt_obs_to_policy(obs, SELF_PLAY_OBS_DIM), obs)


def test_adapt_projects_to_legacy_layout():
    obs = np.arange(SELF_PLAY_OBS_DIM, dtype=np.float32)
    out = adapt_obs_to_policy(obs, LEGACY_22)
    # Keeps the pre-boundary base [0:18] and the sonar tail [OBS_DIM:27]; drops the boundary block.
    expected = np.concatenate([np.arange(LEGACY_OBS_DIM),
                               np.arange(OBS_DIM, SELF_PLAY_OBS_DIM)]).astype(np.float32)
    assert out.shape == (LEGACY_22,)
    assert np.array_equal(out, expected)


def test_adapt_rejects_unknown_width():
    with pytest.raises(ValueError):
        adapt_obs_to_policy(np.zeros(SELF_PLAY_OBS_DIM, dtype=np.float32), SELF_PLAY_OBS_DIM - 2)


def test_compat_policy_downprojects_before_predict():
    rec = RecordingModel()
    CompatPolicy(rec, LEGACY_22).predict(
        np.zeros(SELF_PLAY_OBS_DIM, dtype=np.float32), deterministic=True)
    assert rec.seen_dims == [LEGACY_22]


def test_supported_dims_include_native_and_legacy():
    assert SELF_PLAY_OBS_DIM in MATCH_COMPATIBLE_OBS_DIMS
    assert LEGACY_22 in MATCH_COMPATIBLE_OBS_DIMS


def test_legacy_and_current_policies_can_play_each_other():
    """The reported bug: a 22-dim policy B against a 27-dim policy A used to crash the match.
    Now the match runs and each policy is fed an observation of its own width."""
    model_a = RecordingModel()                       # native 27-dim policy
    legacy_b = RecordingModel()
    model_b = CompatPolicy(legacy_b, LEGACY_22)      # older 22-dim policy, adapted
    eng = MatchEngine(model_a, model_b, seed=0)
    eng.tick()
    assert model_a.seen_dims == [SELF_PLAY_OBS_DIM]   # A saw the full opponent-aware obs
    assert legacy_b.seen_dims == [LEGACY_22]          # B saw its narrower legacy obs
