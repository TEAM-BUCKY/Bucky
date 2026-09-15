#!/usr/bin/env python
"""Train a supervised state→action policy (decision tree / random forest / MLP) from a dataset.

Usage:
    uv run python scripts/collect_dataset.py --expert classical --stage AIM_AND_KICK \
        --n-steps 100000 --out data/expert.npz
    uv run python scripts/train_supervised.py --dataset data/expert.npz --kind forest \
        --out checkpoints/sup_forest/policy.joblib

Writes a ``.joblib`` the tournament / match engine loads via SklearnPolicy.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.algos.supervised import fit_supervised, save_model  # noqa: E402
from bucky.data import Transitions  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True, help="Transitions .npz (see collect_dataset).")
    parser.add_argument("--kind", default="forest", choices=["tree", "forest", "mlp"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True, help="Output .joblib path.")
    args = parser.parse_args()

    data = Transitions.load_npz(args.dataset)
    model = fit_supervised(data.obs, data.actions, kind=args.kind, seed=args.seed)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    save_model(model, args.out)
    score = model.score(data.obs, data.actions)
    print(f"Trained {args.kind} on {len(data)} samples (R²={score:.3f}) → {args.out}")


if __name__ == "__main__":
    main()
