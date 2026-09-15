"""Uniform policy loading: turn a saved artifact of any model type into something that
satisfies the one contract the match engine / eval / tournament rely on::

    action, _ = policy.predict(obs, deterministic=True)

Every learning approach in this project trains to a different on-disk artifact (SB3 ``.zip``,
numpy ``.npz``, sklearn ``.joblib``, a tabular ``.npz``, a controller parameter set, …). The
loaders here normalize them all to that contract so they can be compared head-to-head.
"""
from __future__ import annotations

from bucky.policies.loader import PolicyRef, load_policy

__all__ = ["PolicyRef", "load_policy"]
