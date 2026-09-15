#!/usr/bin/env python
"""Train a tabular Q-learning or SARSA(λ) policy on a single-agent drill.

Usage:
    uv run python scripts/train_tabular.py --algo q --stage PUSH_TO_EMPTY_GOAL \
        --episodes 4000 --out checkpoints/tab_q/policy.qtable.npz
    uv run python scripts/train_tabular.py --algo sarsa --episodes 4000 \
        --out checkpoints/tab_sarsa/policy.qtable.npz

Writes a ``*.qtable.npz`` that the tournament / match engine loads via
:class:`bucky.policies.tabular.TabularPolicy`.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.algos.tabular import save_qtable, train_tabular  # noqa: E402
from bucky.curriculum import Stage  # noqa: E402
from bucky.envs.bucky_single import BuckySingleEnv  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--algo", default="q", choices=["q", "sarsa"])
    parser.add_argument("--stage", default="PUSH_TO_EMPTY_GOAL", choices=[s.value for s in Stage])
    parser.add_argument("--episodes", type=int, default=4000)
    parser.add_argument("--alpha", type=float, default=0.2)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--lam", type=float, default=0.9)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True, help="Output *.qtable.npz path.")
    args = parser.parse_args()

    env = BuckySingleEnv(stage=args.stage, domain_rand=False)

    def _progress(done, total):
        print(f"\r  episode {done}/{total}", end="", flush=True)

    q, disc = train_tabular(env, algo=args.algo, episodes=args.episodes, alpha=args.alpha,
                            gamma=args.gamma, lam=args.lam, seed=args.seed, on_progress=_progress)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    save_qtable(args.out, q, disc)
    visited = int((q != 0).any(axis=1).sum())
    print(f"\nTrained {args.algo} on {args.stage}: {disc.n_states} states "
          f"({visited} visited) → {args.out}")


if __name__ == "__main__":
    main()
