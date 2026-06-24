#!/usr/bin/env python
"""Run a trained checkpoint through a single *drill* and stream the reward build-up.

A drill is just a curriculum stage run in evaluation: the model plays it deterministically
while we stream a live ``eval_step`` frame every step (robot/ball poses + instantaneous AND
per-episode-cumulative reward terms) and, at the end, one ``eval_summary`` frame with the
N-episode aggregate (mean±std return, success rate, per-term mean/std).

This is the live, per-drill, reward-broken-down counterpart of ``scripts/eval.py`` — built so
the ``/eval`` frontend can show whether a reward-shaping change actually helped.

It rides the same streaming path as training/match (StreamClient → ``/api/ingest`` →
``/api/stream``), tagged with its own run name, but emits the distinct ``eval_step`` /
``eval_summary`` frame types so it never disturbs the live training viewer.

Usage (standalone, no hub):
    uv run python scripts/eval_drill.py \
        --checkpoint checkpoints/RUN/final_model.zip --stage APPROACH_STATIC_BALL --n-episodes 2
"""
from __future__ import annotations
import argparse
import os
import signal
import sys
import time
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import numpy as np
from stable_baselines3 import PPO

from bucky.curriculum import Stage
from bucky.obs import OBS_DIM
from bucky.physics.python_backend import DT
from bucky.play_events import CONTACT_DIST
from bucky.selfplay import (
    EVAL_COMPATIBLE_OBS_DIMS,
    SELF_PLAY_OBS_DIM,
    export_policy_npz,
    project_obs,
    sonar_ranges,
)

SINGLE_STAGES = {
    Stage.APPROACH_STATIC_BALL.value,
    Stage.PUSH_TO_EMPTY_GOAL.value,
    Stage.AIM_AND_KICK.value,
}
GOAL_DRILLS = {Stage.PUSH_TO_EMPTY_GOAL.value, Stage.AIM_AND_KICK.value}
# Explicit opponent-aware stages (used only to disambiguate a snapshot-less 39-dim checkpoint).
SELF_PLAY_STAGES = {Stage.SELF_PLAY_1V1.value, Stage.SELF_PLAY_2V2.value}


SPEED_MIN, SPEED_MAX = 0.1, 16.0


def _clamp_speed(v) -> float:
    try:
        return float(min(SPEED_MAX, max(SPEED_MIN, float(v))))
    except (TypeError, ValueError):
        return 1.0


def _obs_dim(model) -> int:
    return int(model.observation_space.shape[0])


def _state_dict(s) -> dict:
    """A PhysicsState as plain rounded numbers (world frame) — a reward-fn state argument."""
    def p(v):
        return [round(float(v[0]), 4), round(float(v[1]), 4)]
    return {
        "robot_pos": p(s.robot_pos),
        "robot_vel": p(s.robot_vel),
        "robot_heading": round(float(s.robot_heading), 4),
        "robot_omega": round(float(s.robot_omega), 4),
        "ball_pos": p(s.ball_pos),
        "ball_vel": p(s.ball_vel),
    }


def _reward_inputs(info: dict) -> dict:
    """A JSON-safe view of the signals the reward function received this step.

    Keeps scalar flags/values (rounding floats) and string event lists; drops the already-shown
    ``reward_terms`` and anything non-scalar (arrays/objects) so the debug payload stays small.
    """
    out: dict = {}
    for k, v in info.items():
        if k == "reward_terms":
            continue
        if isinstance(v, (bool, np.bool_)):
            out[k] = bool(v)
        elif isinstance(v, (int, float, np.integer, np.floating)):
            out[k] = round(float(v), 4)
        elif isinstance(v, (list, tuple)) and v and all(isinstance(x, str) for x in v):
            out[k] = list(v)
    return out


def _read_trained_stage(checkpoint: str) -> str | None:
    """The stage a checkpoint was trained on, from its sibling config.json / meta.json."""
    import json

    d = os.path.dirname(os.path.abspath(checkpoint))
    for fname in ("config.json", "meta.json"):
        p = os.path.join(d, fname)
        if os.path.isfile(p):
            try:
                with open(p) as f:
                    stage = json.load(f).get("stage")
                if stage:
                    return str(stage)
            except Exception:  # noqa: BLE001 — best effort
                pass
    return None


def _is_opponent_aware(dim: int, checkpoint: str) -> bool:
    """Whether a ``dim``-wide policy expects the 4-beam sonar tail.

    Every width except 39 is unambiguous: {22, 27, 43} carry sonar, {18, 23, 35} don't. A
    39-dim policy is either the *current* single-agent base (39, no sonar) or a *legacy*
    opponent-aware policy (35 base + 4 sonar). The run's ``opponent_snapshot.npz`` has that
    run's self-play obs width — if it equals this checkpoint's width, this checkpoint is *at*
    the self-play width (opponent-aware); a wider snapshot (43) means this 39-dim checkpoint is
    a single-agent intermediate. With no snapshot, fall back to an explicit self-play stage.
    """
    if dim in (22, 27, SELF_PLAY_OBS_DIM):
        return True
    if dim in (18, 23, 35):
        return False
    if dim == OBS_DIM:  # 39 — ambiguous
        sibling = os.path.join(os.path.dirname(os.path.abspath(checkpoint)), "opponent_snapshot.npz")
        snap = _npz_obs_dim(sibling) if os.path.isfile(sibling) else None
        if snap is not None:
            return snap == dim
        return _read_trained_stage(checkpoint) in SELF_PLAY_STAGES
    return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, help="Path to a .zip checkpoint")
    parser.add_argument("--stage", default=Stage.APPROACH_STATIC_BALL.value,
                        help="Drill = curriculum stage to run")
    parser.add_argument("--n-episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=999)
    parser.add_argument("--stream-url", default=None,
                        help="ws:// URL of the viz hub ingest endpoint (omit to run silent)")
    parser.add_argument("--opponent", default=None,
                        help="Self-play only: path to an opponent_snapshot.npz")
    parser.add_argument("--control-url", default=None,
                        help="ws:// URL of the hub control-sink endpoint (live playback speed)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Initial playback speed multiplier (1.0 = real time)")
    parser.add_argument("--deterministic", dest="deterministic", action="store_true", default=True)
    parser.add_argument("--no-deterministic", dest="deterministic", action="store_false")
    parser.add_argument("--every", type=int, default=1,
                        help="Stream every Nth step (every step is still counted)")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false", default=True,
                        help="Run as fast as possible instead of pacing to the physics clock")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal.default_int_handler)

    stream = None
    if args.stream_url:
        from bucky.stream_client import StreamClient
        stream = StreamClient(args.stream_url)
        stream.start()

    # Live transport: a thread-safe holder fed by the hub's control sink so the browser can drive
    # playback while it watches — change speed, pause/resume, or single-step. The receiver thread
    # only writes; the main loop reads it each step (scale the delay, gate on pause / step credits).
    ctl = {"speed": _clamp_speed(args.speed), "paused": False, "step": 0}
    control = None
    if args.control_url:
        from bucky.stream_client import JsonRecvClient

        def on_control(msg: dict) -> None:
            if "speed" in msg:
                ctl["speed"] = _clamp_speed(msg.get("speed"))
            if "paused" in msg:
                ctl["paused"] = bool(msg.get("paused"))
            if "step" in msg:
                try:
                    ctl["step"] += max(0, int(msg.get("step")))
                except (TypeError, ValueError):
                    pass
                ctl["paused"] = True  # single-stepping implies a paused rollout

        control = JsonRecvClient(args.control_url, on_control)
        control.start()

    def fail(message: str) -> None:
        print(f"eval_drill.py: {message}", file=sys.stderr)
        if stream:
            stream.send({"type": "trainer_status", "phase": "error",
                         "run_type": "eval", "message": message})
            time.sleep(0.3)
            stream.stop()

    stage = args.stage
    self_play = stage == Stage.SELF_PLAY_1V1.value
    if not self_play and stage not in SINGLE_STAGES:
        fail(f"unknown stage {stage!r}")
        sys.exit(1)

    try:
        model = PPO.load(args.checkpoint, device="cpu")
    except Exception as exc:  # noqa: BLE001
        fail(f"could not load checkpoint: {exc}")
        sys.exit(1)
    dim = _obs_dim(model)
    if dim not in EVAL_COMPATIBLE_OBS_DIMS:
        fail(f"checkpoint has a {dim}-dim obs the eval tool can't drive "
             f"(supported: {sorted(EVAL_COMPATIBLE_OBS_DIMS)}).")
        sys.exit(1)
    opponent_aware = _is_opponent_aware(dim, args.checkpoint)
    if dim != SELF_PLAY_OBS_DIM:
        print(f"eval_drill.py: driving a {dim}-dim "
              f"{'opponent-aware' if opponent_aware else 'single-agent'} checkpoint via obs "
              f"projection from the {SELF_PLAY_OBS_DIM}-dim base.", file=sys.stderr)

    # ── build env + obs adapter for the drill ───────────────────────────────────
    # Every policy is driven by projecting the full 43-dim opponent-aware obs to its own width
    # (``project_obs``). ``to_full`` returns that 43-dim obs for the drill: in self-play the env
    # already produces it; in a single drill we append wall-only sonar (no opponent on the field).
    every = max(1, args.every)
    if self_play:
        from bucky.envs.bucky_selfplay import BuckySelfPlayEnv

        env = BuckySelfPlayEnv(stage=stage, domain_rand=False)
        opponent_label = _resolve_opponent(env, args, model, dim)

        def to_full(obs):
            return np.asarray(obs, dtype=np.float32)
    else:
        from bucky.envs.bucky_single import BuckySingleEnv

        captured: dict = {}

        def capture(state, terms):
            captured["state"] = state
            captured["terms"] = terms

        env = BuckySingleEnv(stage=stage, domain_rand=False, viz_callback=capture)
        opponent_label = None

        def to_full(obs):
            # The obs pairs with the *current* (pre-step) physics state — read it straight from
            # the env (the viz-callback only captures the post-step state for the frame).
            st = env._physics._make_state()
            sonar = sonar_ranges(st.robot_pos, st.robot_heading, None)
            return np.concatenate([np.asarray(obs, dtype=np.float32), sonar])

    def act(obs):
        projected = project_obs(to_full(obs), dim, opponent_aware)
        a, _ = model.predict(projected, deterministic=args.deterministic)
        return a

    if stream:
        stream.send({"type": "trainer_status", "phase": "started",
                     "run_type": "eval", "message": f"eval {stage}"})

    # ── run N episodes, streaming the build-up ──────────────────────────────────
    returns: list[float] = []
    per_term: dict[str, list[float]] = {}
    successes = 0
    period = DT
    next_t = time.perf_counter()
    interrupted = False

    print(f"Eval: {args.checkpoint}  drill={stage}  episodes={args.n_episodes}")
    try:
        for ep in range(args.n_episodes):
            obs, _ = env.reset(seed=args.seed + ep)
            cum: dict[str, float] = {}
            ep_return = 0.0
            step = 0
            done = False
            min_ball_dist = float("inf")
            scored = False
            goals_for = goals_against = 0
            prev_action = np.zeros(4, dtype=np.float32)  # the reward fn's prev_action arg

            while not done:
                # Transport gate: hold here while paused (the rollout freezes — no frames stream)
                # until the browser resumes or grants a single-step credit. Only when paced live.
                if args.realtime:
                    while ctl["paused"] and ctl["step"] <= 0:
                        time.sleep(0.02)
                    if ctl["step"] > 0:
                        ctl["step"] -= 1  # consume one credit; stay paused for the next frame

                action = act(obs)
                # The state the reward fn sees as s0 — the ground-truth physics state *before*
                # this step (compute_rewards is called with state0 captured at the top of step()).
                s0 = env.physics.state_a() if self_play else env._physics._make_state()
                if self_play:
                    obs, reward, terminated, truncated, info = env.step(action)
                    sa = env.physics.state_a()
                    sb = env.physics.state_b()
                    terms = env.last_terms
                    goals_for += int(bool(info.get("goal_a")))
                    goals_against += int(bool(info.get("goal_b")))
                else:
                    obs, reward, terminated, truncated, info = env.step(action)
                    state = captured["state"]
                    terms = captured["terms"]
                    sa, sb = state, None
                    d = float(np.linalg.norm(state.ball_pos - state.robot_pos))
                    min_ball_dist = min(min_ball_dist, d)
                    if info.get("goal_scored") or info.get("predicted_goal"):
                        scored = True

                done = bool(terminated or truncated)
                step += 1
                ep_return += float(reward)
                terms_d = terms.as_dict()
                for k, v in terms_d.items():
                    cum[k] = cum.get(k, 0.0) + float(v)

                if (step % every == 0) or done:
                    # The exact signals the reward function saw this step (for the debug dialog).
                    # Single env returns its reward-input dict as `info`; self-play exposes it.
                    if self_play:
                        rinfo = {**env.last_reward_info, "goal_a": info.get("goal_a"),
                                 "goal_b": info.get("goal_b"),
                                 "referee_events": info.get("referee_events")}
                    else:
                        rinfo = info
                    frame = {
                        "type": "eval_step",
                        "mode": "play" if self_play else "train",
                        "robot_pos": sa.robot_pos.tolist(),
                        "robot_heading": float(sa.robot_heading),
                        "ball_pos": sa.ball_pos.tolist(),
                        "reward_terms": terms_d,
                        "reward_total": float(terms.total),
                        "reward_cumulative": dict(cum),
                        "reward_inputs": _reward_inputs(rinfo),
                        "reward_states": {
                            "s0": _state_dict(s0),
                            "s1": _state_dict(sa),
                            "action": [round(float(x), 4) for x in np.asarray(action).reshape(-1)],
                            "prev_action": [round(float(x), 4) for x in prev_action],
                        },
                        "obs": [float(x) for x in np.asarray(obs).reshape(-1)],
                        "episode": ep,
                        "step": step,
                        "total_return": ep_return,
                        "episodes_total": args.n_episodes,
                    }
                    if self_play and sb is not None:
                        frame["robot2_pos"] = sb.robot_pos.tolist()
                        frame["robot2_heading"] = float(sb.robot_heading)
                    if stream:
                        stream.send(frame)

                prev_action = np.asarray(action, dtype=np.float32).reshape(-1)

                if args.realtime:
                    # Pace to the physics clock, scaled by the live speed multiplier.
                    next_t += period / ctl["speed"]
                    delay = next_t - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        next_t = time.perf_counter()  # fell behind; resync the clock

            # episode bookkeeping
            returns.append(ep_return)
            for k, v in cum.items():
                per_term.setdefault(k, []).append(v)
            if self_play:
                success = goals_for > goals_against
            elif stage in GOAL_DRILLS:
                success = scored
            else:  # APPROACH_STATIC_BALL: reached the ball
                success = min_ball_dist <= CONTACT_DIST
            successes += int(success)
            print(f"  ep {ep}: return={ep_return:+.3f}  success={success}")
    except (KeyboardInterrupt, EOFError, BrokenPipeError, ConnectionResetError):
        interrupted = True
        print("\nEval stopped.")

    # ── aggregate summary ───────────────────────────────────────────────────────
    n = len(returns)
    if n > 0 and stream:
        summary = {
            "type": "eval_summary",
            "checkpoint": args.checkpoint,
            "stage": stage,
            "n_episodes": n,
            "deterministic": bool(args.deterministic),
            "return_mean": float(np.mean(returns)),
            "return_std": float(np.std(returns)),
            "success_rate": float(successes / n),
            "per_term_mean": {k: float(np.mean(v)) for k, v in per_term.items()},
            "per_term_std": {k: float(np.std(v)) for k, v in per_term.items()},
            "episode_returns": [float(r) for r in returns],
        }
        if opponent_label is not None:
            summary["opponent"] = opponent_label
        stream.send(summary)

    if n > 0:
        print(f"\nMean return: {np.mean(returns):+.3f} ± {np.std(returns):.3f}  "
              f"success={successes}/{n}")

    env.close()
    if control:
        control.stop()
    if stream:
        phase = "stopped" if interrupted else "done"
        stream.send({"type": "trainer_status", "phase": phase, "run_type": "eval"})
        try:
            time.sleep(0.3)  # let the final frames flush
        except KeyboardInterrupt:
            pass  # a stop landing during the flush — the summary is already queued
        stream.stop()


def _npz_obs_dim(path: str) -> int | None:
    """Input width of a frozen-opponent ``.npz`` (its first Linear's column count), or None."""
    try:
        with np.load(path) as data:
            return int(data["h0_W"].shape[1])
    except Exception:  # noqa: BLE001
        return None


def _resolve_opponent(env, args, model, dim: int) -> str:
    """Pick the self-play drill's frozen opponent and load it into ``env``.

    Order: an explicit ``--opponent`` path → the checkpoint's own ``opponent_snapshot.npz``
    → a mirror of the (43-dim) policy itself → a passive opponent that stands still. Returns a
    short label of which source was used (surfaced in the summary). Snapshots are accepted only
    if they speak the current 43-dim self-play obs — the opponent runs in pure numpy with no
    obs projection, so a legacy-width snapshot would crash the env.
    """
    # 1. explicit path
    if args.opponent and os.path.isfile(args.opponent) and _npz_obs_dim(args.opponent) == SELF_PLAY_OBS_DIM:
        env.set_opponent(args.opponent)
        return "snapshot"
    # 2. the checkpoint's sibling opponent snapshot
    sibling = os.path.join(os.path.dirname(os.path.abspath(args.checkpoint)), "opponent_snapshot.npz")
    if os.path.isfile(sibling) and _npz_obs_dim(sibling) == SELF_PLAY_OBS_DIM:
        env.set_opponent(sibling)
        return "snapshot"
    # 3. mirror-self — only possible when the raw policy already speaks the 43-dim self-play obs
    if dim == SELF_PLAY_OBS_DIM:
        try:
            tmp = os.path.join(tempfile.gettempdir(), f"eval_mirror_{os.getpid()}.npz")
            export_policy_npz(model, tmp)
            env.set_opponent(tmp)
            return "mirror_self"
        except Exception as exc:  # noqa: BLE001 — fall through to passive
            print(f"eval_drill.py: mirror-self opponent failed ({exc}); using passive.",
                  file=sys.stderr)
    # 4. passive
    env.set_opponent(None)
    return "passive"


if __name__ == "__main__":
    main()
