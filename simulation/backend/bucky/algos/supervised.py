"""Supervised state→action models (scikit-learn): decision tree, random forest, MLP regressor.

These learn a direct obs→action map from a logged dataset (see :mod:`bucky.data`). Used both as
standalone supervised policies and as the learner behind behavioral cloning (:mod:`bucky.algos.
imitation`). Multi-output regression predicts all 4 action dims at once. Artifacts save to
``.joblib`` and drive matches via :class:`bucky.policies.sklearn_policy.SklearnPolicy`.
"""
from __future__ import annotations

import numpy as np


def make_regressor(kind: str, **kw):
    """Build an (untrained) multi-output regressor. ``kind`` ∈ {tree, forest, mlp}."""
    seed = int(kw.get("seed", 0))
    if kind == "tree":
        from sklearn.tree import DecisionTreeRegressor
        return DecisionTreeRegressor(max_depth=kw.get("max_depth", 12), random_state=seed)
    if kind == "forest":
        from sklearn.ensemble import RandomForestRegressor
        return RandomForestRegressor(
            n_estimators=int(kw.get("n_estimators", 100)), max_depth=kw.get("max_depth"),
            random_state=seed, n_jobs=-1)
    if kind == "mlp":
        from sklearn.neural_network import MLPRegressor
        return MLPRegressor(
            hidden_layer_sizes=tuple(kw.get("hidden", (64, 64))),
            max_iter=int(kw.get("max_iter", 300)), random_state=seed)
    raise ValueError(f"unknown supervised kind {kind!r} (tree | forest | mlp)")


def fit_supervised(X, Y, kind: str = "forest", **kw):
    """Fit a regressor mapping observations ``X`` (N, obs) to actions ``Y`` (N, 4)."""
    model = make_regressor(kind, **kw)
    model.fit(np.asarray(X, dtype=np.float32), np.asarray(Y, dtype=np.float32))
    return model


def save_model(model, path: str) -> None:
    import joblib
    joblib.dump(model, path)


def load_model(path: str):
    # joblib uses pickle. These artifacts are produced by our own training scripts and live
    # under the checkpoints tree — the same trust boundary as the SB3 `.zip` checkpoints the
    # app already loads. Only load `.joblib` files this system wrote, never uploaded/untrusted ones.
    import joblib
    return joblib.load(path)
