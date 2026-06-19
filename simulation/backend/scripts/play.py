#!/usr/bin/env python
"""Run a 1v1 match: two trained policies play against each other.

Usage:
    uv run python scripts/play.py \
        --policy-a checkpoints/RUN_A/final_model.zip \
        --policy-b checkpoints/RUN_B/final_model.zip \
        --stream-url ws://localhost:8765/ingest

The hub (scripts/serve.py) normally launches this for you when you pick two networks in
the "Match" panel and click Launch. Each tick steps the shared 1v1 physics in real time
and streams a frame (both robot poses + ball + score) back to the hub.

Match policies must be opponent-aware (21-dim observation, trained on SELF_PLAY_1V1).
Loading a legacy 17-dim single-agent checkpoint here is rejected with a clear message.
"""
from __future__ import annotations
import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO

from bucky.match import MatchEngine
from bucky.physics.python_backend import DT
from bucky.selfplay import SELF_PLAY_OBS_DIM


def _obs_dim(model) -> int:
    return int(model.observation_space.shape[0])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy-a", required=True, help="Path to bot A's .zip checkpoint")
    parser.add_argument("--policy-b", required=True, help="Path to bot B's .zip checkpoint")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--domain-rand", action="store_true",
                        help="Enable sonar/sensor noise during the match")
    parser.add_argument("--stream-url", default=None,
                        help="ws:// URL of the viz hub ingest endpoint")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal.default_int_handler)

    stream = None
    if args.stream_url:
        from bucky.stream_client import StreamClient
        stream = StreamClient(args.stream_url)
        stream.start()

    def fail(message: str) -> None:
        print(f"play.py: {message}", file=sys.stderr)
        if stream:
            stream.send({"type": "trainer_status", "phase": "error", "message": message})
            time.sleep(0.3)
            stream.stop()

    try:
        model_a = PPO.load(args.policy_a, device="cpu")
        model_b = PPO.load(args.policy_b, device="cpu")
    except Exception as exc:  # noqa: BLE001
        fail(f"could not load a checkpoint: {exc}")
        sys.exit(1)

    for name, model in (("A", model_a), ("B", model_b)):
        if _obs_dim(model) != SELF_PLAY_OBS_DIM:
            fail(f"policy {name} has a {_obs_dim(model)}-dim observation but match mode needs "
                 f"{SELF_PLAY_OBS_DIM}-dim (opponent-aware). Train it on SELF_PLAY_1V1 first.")
            sys.exit(1)

    engine = MatchEngine(model_a, model_b, seed=args.seed, domain_rand=args.domain_rand)
    if stream:
        stream.send({"type": "trainer_status", "phase": "started", "run_name": "match", "run_type": "match"})

    print(f"Match: {args.policy_a}  vs  {args.policy_b}")
    period = DT
    next_t = time.perf_counter()
    try:
        while True:
            frame = engine.tick()
            if stream:
                stream.send(frame)
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()   # fell behind; resync the clock
    except (KeyboardInterrupt, EOFError, BrokenPipeError, ConnectionResetError):
        print("\nMatch stopped.")
        if stream:
            stream.send({"type": "trainer_status", "phase": "done", "run_name": "match", "run_type": "match"})
    finally:
        if stream:
            time.sleep(0.3)   # let the final frame flush
            stream.stop()


if __name__ == "__main__":
    main()
