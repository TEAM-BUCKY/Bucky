#!/usr/bin/env python
"""Run a headless round-robin tournament between trained models and print a league table.

Usage:
    uv run python scripts/tournament.py \
        striker_v1/best_model.zip=Striker \
        goalie_v2/final_model.zip=Goalie \
        checkpoints/champ/opponent_snapshot.npz \
        --matches 5 --seed 0 --json out.json

Each positional arg is a model spec ``PATH[=LABEL]``:
  * ``PATH`` is resolved relative to the checkpoints root when it isn't an existing path, so
    ``run/checkpoint.zip`` and ``checkpoints/run/checkpoint.zip`` both work.
  * ``LABEL`` (optional) names the entrant in the table; defaults to the run directory name.

Any artifact :func:`bucky.policies.load_policy` understands (SB3 ``.zip`` of any algo, numpy
``.npz``) can enter. Models play best-of-N with alternating sides; results roll up into a
football-style table (points → goal difference → goals for) plus an Elo rating.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.eval.tournament import format_table, run_round_robin  # noqa: E402
from bucky.policies import PolicyRef, load_policy  # noqa: E402


def _resolve(spec: str, ckpt_root: str) -> PolicyRef:
    path, _, label = spec.partition("=")
    if not os.path.exists(path):
        candidate = os.path.join(ckpt_root, path)
        if os.path.exists(candidate):
            path = candidate
    return PolicyRef(path=path, label=label)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("models", nargs="+", help="Model specs PATH[=LABEL] (2 or more).")
    parser.add_argument("--matches", type=int, default=5, help="Matches per pairing (best-of-N).")
    parser.add_argument("--max-steps", type=int, default=200_000,
                        help="Per-match step cap (default = full 2×7-min match; e.g. 800 = quick).")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--ckpt-root", default="checkpoints",
                        help="Root to resolve bare run/checkpoint paths against.")
    parser.add_argument("--json", default=None, help="Write full results JSON to this path.")
    args = parser.parse_args()

    if len(args.models) < 2:
        parser.error("need at least 2 models for a tournament")

    refs = [_resolve(spec, args.ckpt_root) for spec in args.models]
    entrants = []
    for ref in refs:
        print(f"loading {ref.display}  ({ref.path})")
        entrants.append((ref.display, load_policy(ref)))

    n_pairs = len(entrants) * (len(entrants) - 1) // 2
    print(f"\nRunning round-robin: {len(entrants)} entrants, {n_pairs} pairings × "
          f"{args.matches} matches = {n_pairs * args.matches} matches\n")

    def _progress(_live, p):
        print(f"\r  match {p['match']}/{p['total']}  ({p['pairing']})", end="", flush=True)

    result = run_round_robin(entrants, matches_per_pairing=args.matches, seed=args.seed,
                             max_steps=args.max_steps, on_update=_progress)
    print("\n\n" + format_table(result))

    if args.json:
        with open(args.json, "w") as f:
            json.dump(result.to_dict(), f, indent=2)
        print(f"\nWrote {args.json}")


if __name__ == "__main__":
    main()
