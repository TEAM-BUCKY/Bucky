#!/usr/bin/env python
"""Imitate an expert via behavioral cloning (BC) or DAgger.

Usage:
    # BC from a pre-collected dataset
    uv run python scripts/train_imitation.py --method bc --dataset data/expert.npz \
        --out checkpoints/bc_mlp/policy.joblib

    # DAgger against the classical controller (auto-relabels the clone's own states)
    uv run python scripts/train_imitation.py --method dagger --expert classical \
        --stage AIM_AND_KICK --rounds 5 --out checkpoints/dagger_mlp/policy.joblib

The expert is anything bucky.policies.load_policy understands (classical / SB3 zip / npz).
Writes a ``.joblib`` that loads via SklearnPolicy.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.algos.imitation import behavioral_cloning, dagger  # noqa: E402
from bucky.algos.supervised import save_model  # noqa: E402
from bucky.curriculum import Stage, get_stage_config  # noqa: E402
from bucky.data import Transitions  # noqa: E402
from bucky.policies import PolicyRef, load_policy  # noqa: E402


def _build_env(stage: str, opponent: str | None):
    if get_stage_config(stage).opponent_present:
        from bucky.envs.bucky_selfplay import BuckySelfPlayEnv
        env = BuckySelfPlayEnv(stage=stage, domain_rand=False)
        if opponent:
            env.set_opponent(opponent)
        else:
            print(f"WARNING: stage {stage} has an opponent but no --opponent given; "
                  f"imitating against the env's default opponent.")
        return env
    from bucky.envs.bucky_single import BuckySingleEnv
    return BuckySingleEnv(stage=stage, domain_rand=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", default="bc", choices=["bc", "dagger"])
    parser.add_argument("--kind", default="mlp", choices=["tree", "forest", "mlp"])
    parser.add_argument("--dataset", default=None, help="BC: pre-collected Transitions .npz.")
    parser.add_argument("--expert", default=None, help="Expert spec (DAgger, or BC w/o dataset).")
    parser.add_argument("--stage", default="AIM_AND_KICK", choices=[s.value for s in Stage])
    parser.add_argument("--opponent", default=None, help="Frozen opponent .npz (self-play stages).")
    parser.add_argument("--rounds", type=int, default=5, help="DAgger rounds.")
    parser.add_argument("--steps", type=int, default=4000, help="Transitions collected per round.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True, help="Output .joblib path.")
    args = parser.parse_args()

    if args.method == "bc":
        if args.dataset:
            data = Transitions.load_npz(args.dataset)
        else:
            if not args.expert:
                parser.error("BC needs --dataset or --expert")
            from bucky.data import collect
            expert = load_policy(PolicyRef(path=args.expert, label="expert"))
            env = _build_env(args.stage, args.opponent)
            data = collect(expert, env, args.steps, seed=args.seed)
        model = behavioral_cloning(data, kind=args.kind, seed=args.seed)
        print(f"BC ({args.kind}) on {len(data)} samples")
    else:
        if not args.expert:
            parser.error("DAgger needs --expert")
        expert = load_policy(PolicyRef(path=args.expert, label="expert"))
        env = _build_env(args.stage, args.opponent)

        def _on_round(r, n):
            print(f"  DAgger round {r}: {n} aggregated samples")

        model = dagger(expert, env, rounds=args.rounds, steps_per_round=args.steps,
                       kind=args.kind, seed=args.seed, on_round=_on_round)
        print(f"DAgger ({args.kind}) done after {args.rounds} rounds")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    save_model(model, args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
