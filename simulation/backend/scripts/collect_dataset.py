#!/usr/bin/env python
"""Collect an expert-play dataset for imitation / supervised / offline learning.

Usage:
    # classical controller demonstrating the empty-goal drill
    uv run python scripts/collect_dataset.py --expert classical \
        --stage PUSH_TO_EMPTY_GOAL --n-steps 100000 --out data/classical_push.npz

    # a trained champion demonstrating self-play (needs a frozen opponent .npz)
    uv run python scripts/collect_dataset.py --expert checkpoints/champ/best_model.zip \
        --stage SELF_PLAY_1V1 --opponent checkpoints/champ/opponent_snapshot.npz \
        --n-steps 200000 --out data/champ_selfplay.npz

The ``--expert`` is anything :func:`bucky.policies.load_policy` understands (``classical``, an
SB3 ``.zip``, a numpy ``.npz``, a ``*.controller.json``). Writes a flat ``.npz`` of transitions.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.curriculum import Stage, get_stage_config  # noqa: E402
from bucky.data import collect  # noqa: E402
from bucky.policies import PolicyRef, load_policy  # noqa: E402


def _build_env(stage: str, opponent: str | None, domain_rand: bool):
    if get_stage_config(stage).opponent_present:
        from bucky.envs.bucky_selfplay import BuckySelfPlayEnv
        env = BuckySelfPlayEnv(stage=stage, domain_rand=domain_rand)
        if opponent:
            env.set_opponent(opponent)
        else:
            print(f"WARNING: stage {stage} has an opponent but no --opponent given; "
                  f"the expert will demonstrate against the env's default opponent.")
        return env
    from bucky.envs.bucky_single import BuckySingleEnv
    return BuckySingleEnv(stage=stage, domain_rand=domain_rand)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--expert", required=True, help="Expert policy spec (classical / path).")
    parser.add_argument("--stage", default="PUSH_TO_EMPTY_GOAL", choices=[s.value for s in Stage])
    parser.add_argument("--opponent", default=None, help="Frozen opponent .npz (self-play stages).")
    parser.add_argument("--n-steps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--domain-rand", action="store_true", help="Randomize during collection.")
    parser.add_argument("--stochastic", action="store_true", help="Sample expert actions.")
    parser.add_argument("--out", required=True, help="Output .npz path.")
    args = parser.parse_args()

    expert = load_policy(PolicyRef(path=args.expert, label="expert"))
    env = _build_env(args.stage, args.opponent, args.domain_rand)

    def _progress(done, total):
        print(f"\r  collected {done}/{total} transitions", end="", flush=True)

    data = collect(expert, env, args.n_steps, seed=args.seed,
                   deterministic=not args.stochastic, on_progress=_progress)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    data.save_npz(args.out)
    print(f"\nWrote {len(data)} transitions (obs {data.obs.shape[1]}d, "
          f"act {data.actions.shape[1]}d) to {args.out}")


if __name__ == "__main__":
    main()
