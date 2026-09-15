#!/usr/bin/env python
"""Train an offline-RL policy (CQL / IQL / BCQ) from a logged dataset, via d3rlpy.

Requires the optional dependency group:  uv sync --extra learners-extra

Usage:
    uv run python scripts/collect_dataset.py --expert checkpoints/champ/best_model.zip \
        --stage SELF_PLAY_1V1 --opponent checkpoints/champ/opponent_snapshot.npz \
        --n-steps 200000 --out data/champ.npz
    uv run python scripts/train_offline.py --dataset data/champ.npz --algo iql \
        --n-steps 50000 --out checkpoints/offline_iql/policy.d3

Writes a ``.d3`` bundle the tournament / match engine loads via D3rlpyPolicy.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.algos.offline import save_offline, train_offline  # noqa: E402
from bucky.data import Transitions  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True, help="Transitions .npz (see collect_dataset).")
    parser.add_argument("--algo", default="cql", choices=["cql", "iql", "bcq"])
    parser.add_argument("--n-steps", type=int, default=20_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True, help="Output .d3 path.")
    args = parser.parse_args()

    data = Transitions.load_npz(args.dataset)
    print(f"Training {args.algo.upper()} on {len(data)} transitions for {args.n_steps} steps…")
    model = train_offline(data, algo=args.algo, n_steps=args.n_steps, seed=args.seed)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    save_offline(model, args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
