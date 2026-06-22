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

Match policies are opponent-aware (SELF_PLAY_OBS_DIM observation, trained on SELF_PLAY_1V1).
An older, narrower self-play checkpoint is still accepted: its observation is projected down
to the layout it was trained on (see bucky.selfplay.CompatPolicy) so old and new networks can
play each other. A width we can't map to is rejected with a clear message.
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
from bucky.selfplay import (
    MATCH_COMPATIBLE_OBS_DIMS,
    SELF_PLAY_OBS_DIM,
    CompatPolicy,
)


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
    parser.add_argument("--control-url", default=None,
                        help="ws:// URL of the hub control-sink endpoint (human control). When "
                             "set, either side may be driven by a human (online play / manual test)")
    parser.add_argument("--manual-red", action="store_true",
                        help="Legacy: start red (robot B) under human control by default "
                             "(single-human 'test against the AI' feature)")
    parser.add_argument("--mode", choices=("match", "casual"), default="match",
                        help="match → full 2×7 min referee; casual → endless human-vs-human play")
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

    # Match mode runs the current opponent-aware observation (SELF_PLAY_OBS_DIM). A policy of a
    # different width is either an older, narrower checkpoint we can project the obs down to
    # (wrapped in CompatPolicy so old and new networks can play each other) or one we can't —
    # only the latter is rejected.
    def adapt(name: str, model):
        dim = _obs_dim(model)
        if dim == SELF_PLAY_OBS_DIM:
            return model
        if dim in MATCH_COMPATIBLE_OBS_DIMS:
            print(f"play.py: policy {name} is a legacy {dim}-dim checkpoint; adapting its "
                  f"observation to the {SELF_PLAY_OBS_DIM}-dim opponent-aware match obs.",
                  file=sys.stderr)
            return CompatPolicy(model, dim)
        fail(f"policy {name} has a {dim}-dim observation that match mode can't adapt to "
             f"{SELF_PLAY_OBS_DIM}-dim (opponent-aware). Train it on SELF_PLAY_1V1 first.")
        sys.exit(1)

    model_a = adapt("A", model_a)
    model_b = adapt("B", model_b)

    # Manual control: subscribe to the hub's control-sink so humans can drive either side.
    # The legacy single-human feature (--manual-red) starts red (B) under human control; an
    # online game leaves both sides on "ai" until each browser starts sending human input.
    control = None
    control_source = None
    if args.control_url:
        from bucky.stream_client import ControlClient
        default_modes = {"a": "ai", "b": "human"} if args.manual_red else None
        control = ControlClient(args.control_url, default_modes=default_modes)
        control.start()
        control_source = control.latest

    engine = MatchEngine(model_a, model_b, seed=args.seed, domain_rand=args.domain_rand,
                         control_source=control_source, mode=args.mode)
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
        if control:
            control.stop()
        if stream:
            time.sleep(0.3)   # let the final frame flush
            stream.stop()


if __name__ == "__main__":
    main()
