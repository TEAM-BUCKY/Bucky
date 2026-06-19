#!/usr/bin/env python
"""Evaluate a trained Bucky checkpoint.

Usage:
    uv run python scripts/eval.py --checkpoint checkpoints/APPROACH_STATIC_BALL_seed0/final_model
"""
from __future__ import annotations
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import numpy as np
from stable_baselines3 import PPO
from bucky.curriculum import Stage
from bucky.envs.bucky_single import BuckySingleEnv


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--stage", default="APPROACH_STATIC_BALL")
    parser.add_argument("--n-episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=999)
    args = parser.parse_args()

    env = BuckySingleEnv(stage=args.stage, domain_rand=False)
    model = PPO.load(args.checkpoint, env=env)

    returns = []
    per_term: dict[str, list[float]] = {}

    for ep in range(args.n_episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        ep_return, done = 0.0, False
        ep_terms: dict[str, float] = {}

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, term, trunc, info = env.step(action)
            ep_return += reward
            done = term or trunc
            for k, v in info.get("reward_terms", {}).items():
                ep_terms[k] = ep_terms.get(k, 0.0) + v

        returns.append(ep_return)
        for k, v in ep_terms.items():
            per_term.setdefault(k, []).append(v)

    print(f"\n{'='*50}")
    print(f"Checkpoint: {args.checkpoint}")
    print(f"Episodes:   {args.n_episodes}")
    print(f"Mean return: {np.mean(returns):.3f} ± {np.std(returns):.3f}")
    print(f"\nPer reward-term totals (mean over episodes):")
    for k, vals in sorted(per_term.items()):
        print(f"  {k:20s}: {np.mean(vals):+.4f}")
    env.close()


if __name__ == "__main__":
    main()
