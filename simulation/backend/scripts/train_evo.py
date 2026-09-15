#!/usr/bin/env python
"""Evolve a policy with a black-box optimizer (CEM/ES or GA).

Targets:
  * ``controller`` — tune the classical reactive controller's parameters (saves *.controller.json)
  * ``mlp``        — neuroevolve a small tanh MLP policy (saves numpy-opponent .npz)

Usage:
    uv run python scripts/train_evo.py --target controller --method es \
        --stage AIM_AND_KICK --generations 30 --out checkpoints/evo_ctrl/tuned.controller.json
    uv run python scripts/train_evo.py --target mlp --method es --hidden 32 \
        --stage PUSH_TO_EMPTY_GOAL --generations 40 --out checkpoints/evo_mlp/policy.npz

Both artifacts load via bucky.policies.load_policy, so they drop straight into the tournament.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.algos.evolution import (  # noqa: E402
    OPTIMIZERS,
    ControllerGenome,
    MLPGenome,
    rollout_fitness,
)
from bucky.curriculum import Stage  # noqa: E402
from bucky.envs.bucky_single import BuckySingleEnv  # noqa: E402
from bucky.obs import OBS_DIM  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", default="controller", choices=["controller", "mlp"])
    parser.add_argument("--method", default="es", choices=list(OPTIMIZERS))
    parser.add_argument("--stage", default="AIM_AND_KICK", choices=[s.value for s in Stage])
    parser.add_argument("--hidden", type=int, default=32, help="Hidden width (mlp target).")
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--popsize", type=int, default=32)
    parser.add_argument("--episodes", type=int, default=4, help="Fitness episodes per candidate.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    env = BuckySingleEnv(stage=args.stage, domain_rand=False)
    if args.target == "controller":
        genome = ControllerGenome()
        x0, sigma = genome.x0(), genome.sigma0()
    else:
        genome = MLPGenome([OBS_DIM, args.hidden, 4])
        x0, sigma = genome.x0(args.seed), 0.3

    def fitness(vec):
        return rollout_fitness(genome.to_policy(vec), env, args.episodes, args.seed)

    def _on_gen(g, best, mean):
        print(f"\r  gen {g + 1}/{args.generations}  best={best:8.2f}  mean={mean:8.2f}",
              end="", flush=True)

    optimizer = OPTIMIZERS[args.method]
    best_vec, best_fit = optimizer(
        fitness, genome.dim, x0=x0, sigma=sigma, generations=args.generations,
        popsize=args.popsize, seed=args.seed, on_gen=_on_gen)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    genome.save(best_vec, args.out)
    print(f"\nEvolved {args.target} ({args.method}) best fitness {best_fit:.2f} → {args.out}")


if __name__ == "__main__":
    main()
