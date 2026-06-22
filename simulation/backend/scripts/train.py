#!/usr/bin/env python
"""Train Bucky agent with PPO.

Usage:
    uv run python scripts/train.py --stage APPROACH_STATIC_BALL --timesteps 200000
    uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --timesteps 1000000 --n-envs 8

Live visualization is driven by the FastAPI backend (app/): pass --stream-url and this
trainer streams frames + metrics back to its /api/ingest endpoint. The backend normally
launches this script for you when you click "Launch" in the UI, so you rarely run
--stream-url by hand.
"""
from __future__ import annotations
import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Pin numerical libraries to a single thread *before* importing numpy/torch. Each
# SubprocVecEnv worker steps a NumPy physics env, and NumPy's BLAS (OpenBLAS/MKL)
# is multithreaded by default — N workers each spawning a core's worth of BLAS
# threads massively oversubscribes the CPU and tanks rollout throughput. These are
# read at library load, so they must be set here (and they propagate to the forked
# workers). torch.set_num_threads(1) below only governs torch in the main process,
# not NumPy in the workers, so it does not cover this. setdefault() lets an operator
# still override from the environment.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback, CheckpointCallback


class StopAtTime(BaseCallback):
    """Stop training at a wall-clock deadline (epoch seconds).

    Returning False from a callback makes SB3's ``learn`` exit cleanly between
    steps, so the normal post-training save runs — a time-limited run finishes
    and checkpoints exactly like a step-limited one.
    """

    def __init__(self, deadline: float) -> None:
        super().__init__()
        self._deadline = deadline

    def _on_step(self) -> bool:
        return time.time() < self._deadline

from bucky.curriculum import Stage, get_stage_config
from bucky.envs.bucky_single import BuckySingleEnv
from bucky.envs.bucky_selfplay import BuckySelfPlayEnv


def make_env(stage: str, domain_rand: bool, reward_config=None):
    self_play = stage == Stage.SELF_PLAY_1V1.value

    def _init():
        if self_play:
            return BuckySelfPlayEnv(stage=stage, domain_rand=domain_rand,
                                    reward_config=reward_config)
        return BuckySingleEnv(stage=stage, domain_rand=domain_rand,
                              reward_config=reward_config)
    return _init


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default="APPROACH_STATIC_BALL",
                        choices=[s.value for s in Stage])
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--duration", type=float, default=None,
                        help="Wall-clock seconds to train, then stop and save. "
                             "Overrides --timesteps as the stop condition.")
    parser.add_argument("--until", type=float, default=None,
                        help="Absolute epoch time to train until, then stop and save. "
                             "Overrides --timesteps and --duration.")
    parser.add_argument("--n-envs", type=int, default=None,
                        help="Parallel rollout envs. Defaults to the CPU core count "
                             "(clamped to 4..32) when unset, so rollout parallelism "
                             "matches the machine instead of a fixed 16.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--no-domain-rand", action="store_true")
    parser.add_argument("--resume-from", default=None,
                        help="Path to a .zip checkpoint to seed weights from "
                             "(continues training in this new run)")
    parser.add_argument("--config", default=None,
                        help="Path to a ModelConfig JSON. When set, its hyperparameters, "
                             "net_arch and reward weights drive the run (and its "
                             "stage/n_envs/seed/domain_rand override the matching flags).")
    parser.add_argument("--stream-url", default=None,
                        help="ws:// URL of the viz hub ingest endpoint (enables live viz)")
    parser.add_argument("--stream-token", default=None,
                        help="Auth token for the stream, sent as the X-Device-Token "
                             "handshake header (keeps it out of the URL/logs).")
    parser.add_argument("--viz", action="store_true",
                        help="Animate the live field in the UI. Off by default: it runs an "
                             "extra in-process 'shadow' rollout on the training thread, so it "
                             "costs throughput. Training metrics/status still stream without it.")
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"],
                        help="Torch device. CPU is fastest for this tiny MLP — the GPU's "
                             "per-step host<->device copies outweigh its compute here. "
                             "'auto' selects cuda when available (opt-in, not the default).")
    parser.add_argument("--compile", action="store_true",
                        help="EXPERIMENTAL: torch.compile the policy. Marginal for this tiny "
                             "MLP and adds warmup; mainly useful on GPU. Verify checkpoint "
                             "save/load and ONNX export still work before relying on it.")
    # Federated distributed training: this run is one shard of a group that periodically
    # averages policy weights through the coordinator (see bucky.fedavg).
    parser.add_argument("--fed-server", default=None,
                        help="Coordinator API base (…/api) for federated weight sync.")
    parser.add_argument("--fed-token", default=None, help="Token for the fed sync endpoints.")
    parser.add_argument("--fed-group", default=None, help="Distributed-run group id (sync key).")
    parser.add_argument("--fed-every", type=int, default=50_000,
                        help="Steps between federated weight-averaging syncs.")
    parser.add_argument("--fed-shards", type=int, default=1, help="Number of shards in the group.")
    args = parser.parse_args()

    # The hub may launch us from a backgrounded process whose SIGINT is set to
    # SIG_IGN (POSIX background-job behaviour); restore the default so a "Kill"
    # SIGINT reliably raises KeyboardInterrupt and we can checkpoint before exit.
    signal.signal(signal.SIGINT, signal.default_int_handler)

    # Optional ModelConfig: when present it is authoritative for the run shape, so
    # a model "kind" (network size, reward weights, PPO knobs) reproduces exactly.
    from bucky.rewards import RewardConfig
    cfg = {}
    if args.config:
        with open(args.config) as f:
            cfg = json.load(f)
        args.stage = cfg.get("stage", args.stage)
        if "n_envs" in cfg:
            args.n_envs = int(cfg["n_envs"])
        args.seed = int(cfg.get("seed", args.seed))
        if "domain_rand" in cfg:
            args.no_domain_rand = not bool(cfg["domain_rand"])
    hyperparams = cfg.get("hyperparams", {})
    net_arch = cfg.get("net_arch", [64, 64])
    reward_config = RewardConfig.from_dict(cfg.get("reward_weights"))

    # Resolve n_envs only if neither the flag nor the config set it: match rollout
    # parallelism to the host's core count (clamped so a 2-core VPS or a 64-core box
    # both land somewhere sane). Beyond ~32 the SubprocVecEnv IPC overhead outweighs
    # extra parallelism for these tiny envs.
    if args.n_envs is None:
        args.n_envs = max(4, min(32, os.cpu_count() or 16))
        print(f"n_envs auto-set to {args.n_envs} (CPU cores={os.cpu_count()})")

    run_name = args.run_name or f"{args.stage}_seed{args.seed}"
    log_dir = f"runs/{run_name}"
    ckpt_dir = f"checkpoints/{run_name}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(ckpt_dir, exist_ok=True)

    domain_rand = not args.no_domain_rand
    self_play = args.stage == Stage.SELF_PLAY_1V1.value
    snapshot_base = f"{ckpt_dir}/opponent_snapshot"   # model.save() writes {base}.zip
    # Workers drive the frozen opponent from this pure-numpy weight dump (no torch in the
    # workers — see bucky.selfplay), refreshed alongside the .zip snapshot.
    snapshot_npz = f"{snapshot_base}.npz"

    # The policy net is tiny ([64,64] MLP on 17-dim obs). PyTorch's default inter-op
    # parallelism spawns more threads than the kernel can usefully schedule alongside
    # the SubprocVecEnv workers, so pin it to 1 to eliminate that contention.
    torch.set_num_threads(1)

    # Anomaly detection is a debug-only autograd hook (slow); keep it explicitly off
    # for training runs (this is the default, but make the intent clear).
    torch.autograd.set_detect_anomaly(False)

    # Resolve the device up front (SB3's "auto" picks cuda when available). The CPU-only
    # torch wheel reports no cuda, so this stays cpu unless a CUDA build is installed.
    from stable_baselines3.common.utils import get_device
    device = get_device(args.device)
    if device.type == "cuda":
        # GPU-only tuning-guide items: let matmuls use TF32 tensor cores, and let cuDNN
        # autotune (harmless here — no conv layers). Note: for this [64,64] MLP the GPU
        # is usually *slower* than CPU because per-rollout host<->device copies dominate,
        # so cuda is opt-in via --device, never the default.
        torch.set_float32_matmul_precision("high")
        torch.backends.cudnn.benchmark = True
        print("CUDA device active: TF32 matmul + cudnn.benchmark enabled.")

    # Live streaming to the hub. Training metrics + status always stream (cheap, and the
    # queue/UI needs the progress). The animated field is opt-in (--viz): it is fed by a
    # dedicated in-process "shadow" rollout (LiveVizCallback) that runs on the training
    # thread, so it costs throughput — headless by default keeps training fast.
    stream = None
    extra_callbacks = []
    if args.stream_url:
        from bucky.stream_client import StreamClient
        from bucky.callbacks import MetricsCallback
        stream_headers = {"X-Device-Token": args.stream_token} if args.stream_token else None
        stream = StreamClient(args.stream_url, headers=stream_headers)
        stream.start()
        extra_callbacks = [MetricsCallback(stream)]
        if args.viz:
            from bucky.callbacks import LiveVizCallback, SelfPlayVizCallback
            viz = (SelfPlayVizCallback(stream, opponent_path=snapshot_npz)
                   if self_play else
                   LiveVizCallback(stream, stage=args.stage, domain_rand=False))
            extra_callbacks.insert(0, viz)

    # Self-play envs are tiny pure-numpy steppers and the frozen opponent now runs in numpy
    # too (no torch in the env), so run them in-process with DummyVecEnv: torch is loaded
    # once instead of once per SubprocVecEnv worker — ~0.7 GB vs ~5.5 GB for 16 envs, the
    # difference between fitting an 8 GB host and being OOM-killed — and it is actually
    # faster here, since the per-step IPC of SubprocVecEnv outweighs the trivial env compute.
    # Single-agent stages keep SubprocVecEnv (their working, memory-light path is unchanged).
    vec_cls = DummyVecEnv if self_play else SubprocVecEnv

    train_env = make_vec_env(
        make_env(args.stage, domain_rand, reward_config=reward_config),
        n_envs=args.n_envs,
        seed=args.seed,
        vec_env_cls=vec_cls,
    )

    # Same vec_env_cls as train_env so SB3's EvalCallback doesn't warn about a type
    # mismatch (SubprocVecEnv vs the default DummyVecEnv).
    eval_env = make_vec_env(
        make_env(args.stage, domain_rand=False, reward_config=reward_config),
        n_envs=1,
        seed=args.seed + 1000,
        vec_env_cls=vec_cls,
    )

    # PPO hyperparameters tuned for low-dim continuous control (17-dim obs, 3-dim act).
    # net_arch=[64,64] chosen to fit Cortex-M4F after int8 quantization.
    # n_steps=512, batch_size=256 → 8 minibatches per update.
    if args.resume_from:
        # Seed weights + optimizer state from an existing checkpoint and keep training.
        # Resume requires the checkpoint's obs/action spaces to match the current env's.
        # That holds for curriculum transfer between today's stages (they share the
        # 17-dim obs), but NOT for checkpoints saved before the observation layout
        # changed — those have a different input size and cannot be loaded.
        print(f"Resuming from {args.resume_from}")
        try:
            model = PPO.load(
                args.resume_from,
                env=train_env,
                tensorboard_log=log_dir,
                device=device,
            )
        except ValueError as e:
            if "spaces do not match" not in str(e):
                raise
            sys.exit(
                f"\nCannot resume from {args.resume_from}: it was trained with a "
                f"different observation/action space than the current environment.\n"
                f"  {e}\n"
                f"This checkpoint is stale (the observation layout changed). Train a "
                f"fresh model instead by launching without 'resume from' / --resume-from.\n"
            )
    else:
        # Hyperparameters come from the ModelConfig when supplied, else fall back to
        # the tuned defaults (the values that were previously hardcoded here).
        def hp(key, default):
            return hyperparams.get(key, default)

        # Entropy coefficient defaults to the stage's recommended value (kick/self-play stages
        # want more exploration) unless the model config sets one explicitly.
        stage_ent_coef = get_stage_config(args.stage).recommended_ent_coef

        model = PPO(
            policy="MlpPolicy",
            env=train_env,
            verbose=1,
            tensorboard_log=log_dir,
            seed=args.seed,
            learning_rate=hp("learning_rate", 3e-4),
            n_steps=hp("n_steps", 1024),
            batch_size=hp("batch_size", 512),
            n_epochs=hp("n_epochs", 10),
            gamma=hp("gamma", 0.99),
            gae_lambda=hp("gae_lambda", 0.95),
            clip_range=hp("clip_range", 0.2),
            ent_coef=hp("ent_coef", stage_ent_coef),
            vf_coef=hp("vf_coef", 0.5),
            max_grad_norm=hp("max_grad_norm", 0.5),
            policy_kwargs={"net_arch": list(net_arch)},
            device=device,
        )

    # EXPERIMENTAL opt-in: torch.compile the policy. On GPU "reduce-overhead" uses CUDA
    # graphs to cut kernel-launch cost; on CPU the upside for a [64,64] MLP is marginal.
    # SB3 save/load and ONNX export read policy.state_dict(), which torch.compile prefixes
    # with "_orig_mod." — verify both still round-trip before using this for real runs.
    if args.compile:
        mode = "reduce-overhead" if device.type == "cuda" else "default"
        model.policy = torch.compile(model.policy, mode=mode)
        print(f"Policy compiled with torch.compile (mode={mode}).")

    # Self-play bootstrap: freeze the freshly-built policy as the initial opponent and
    # hand it to every worker before training starts, then refresh it periodically.
    if self_play:
        from bucky.callbacks import SelfPlaySnapshotCallback
        from bucky.selfplay import export_policy_npz
        pool_size = int(cfg.get("opponent_pool_size", 5))
        model.save(snapshot_base)
        export_policy_npz(model, snapshot_npz)
        # Seed the opponent pool with slot 0 so the very first episodes already have an
        # opponent; SelfPlaySnapshotCallback then grows/refreshes the pool during training.
        seed_npz = f"{snapshot_base}_0.npz"
        export_policy_npz(model, seed_npz)
        train_env.env_method("set_opponent_pool", [seed_npz])
        eval_env.env_method("set_opponent_pool", [seed_npz])
        extra_callbacks.append(
            SelfPlaySnapshotCallback(
                snapshot_base, every=max(50_000 // args.n_envs, 1), pool_size=pool_size,
            )
        )

    callbacks = [
        EvalCallback(
            eval_env,
            best_model_save_path=ckpt_dir,
            log_path=log_dir,
            eval_freq=max(10_000 // args.n_envs, 1),
            n_eval_episodes=20,
            deterministic=True,
        ),
        *extra_callbacks,
    ]
    # Periodic step snapshots (model_<N>_steps.zip) are opt-in — they pile up disk
    # fast and the best/final checkpoints are usually all you need.
    if cfg.get("save_step_checkpoints"):
        callbacks.append(
            CheckpointCallback(
                save_freq=max(50_000 // args.n_envs, 1),
                save_path=ckpt_dir,
                name_prefix="model",
            )
        )

    # Federated distributed training: sync this shard's weights with its group.
    if args.fed_server and args.fed_group:
        from bucky.fedavg import FedSyncCallback
        callbacks.append(FedSyncCallback(
            args.fed_server, args.fed_token or "", args.fed_group, run_name,
            every=args.fed_every, shards=args.fed_shards, weight=args.n_envs, verbose=1,
        ))

    # Optional wall-clock stop. --until wins over --duration; either one runs the
    # step budget up to a very large number so time is the binding constraint.
    deadline = None
    if args.until is not None:
        deadline = float(args.until)
    elif args.duration is not None:
        deadline = time.time() + float(args.duration)
    if deadline is not None:
        callbacks.append(StopAtTime(deadline))

    if deadline is not None:
        mins = max(0.0, (deadline - time.time()) / 60)
        print(f"\nTraining {run_name} | stage={args.stage} | ~{mins:.1f} min (until time) | {args.n_envs} envs")
    else:
        print(f"\nTraining {run_name} | stage={args.stage} | {args.timesteps:,} steps | {args.n_envs} envs")
    if stream:
        stream.send({"type": "trainer_status", "phase": "started", "run_name": run_name, "run_type": "train"})

    def update_meta(status: str) -> None:
        """Best-effort refresh of the run's meta.json after a save (status, steps,
        best eval). Lets the admin panel — and the guest-worker upload — reflect how
        the run actually finished."""
        try:
            from pathlib import Path

            from app.models import best_eval, write_meta
            meta = {
                "status": status,
                "timesteps_trained": int(getattr(model, "num_timesteps", 0) or 0),
                "best_eval": best_eval(Path("runs"), run_name),
            }
            if cfg:
                meta.update({"name": cfg.get("name"), "version": cfg.get("version"), "config": cfg})
            write_meta(Path("checkpoints"), run_name, meta)
        except Exception:  # noqa: BLE001 — metadata is non-critical
            pass

    save_path = f"{ckpt_dir}/final_model"
    try:
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except (KeyboardInterrupt, EOFError, BrokenPipeError, ConnectionResetError):
        # Hub "Kill" sends SIGINT precisely so we can checkpoint before exiting.
        # (A broken worker pipe surfaces as EOFError/BrokenPipeError, so catch those too.)
        print("\nInterrupted — saving model…")
        save_path = f"{ckpt_dir}/interrupted_model"
        if stream:
            stream.send({"type": "trainer_status", "phase": "saving", "run_name": run_name, "run_type": "train"})
        try:
            model.save(save_path)
            print(f"Saved to {save_path}.zip")
        except Exception as exc:  # noqa: BLE001
            print(f"Could not save on interrupt: {exc}")
        update_meta("interrupted")
        if stream:
            stream.send({"type": "trainer_status", "phase": "interrupted", "run_name": run_name, "run_type": "train"})
    else:
        model.save(save_path)
        update_meta("done")
        if stream:
            stream.send({"type": "trainer_status", "phase": "done", "run_name": run_name, "run_type": "train"})
        print(f"\nDone. Model saved to {save_path}.zip")
    finally:
        for env in (train_env, eval_env):
            try:
                env.close()
            except Exception:  # noqa: BLE001 — best-effort worker cleanup
                pass
        if stream:
            time.sleep(0.3)  # let the final status frame flush to the hub
            stream.stop()


if __name__ == "__main__":
    main()
