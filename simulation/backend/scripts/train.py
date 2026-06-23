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

from bucky.curriculum import FULL_TRAINING_PHASES, Stage, get_stage_config
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
    snapshot_base = f"{ckpt_dir}/opponent_snapshot"   # model.save() writes {base}.zip
    # Workers drive the frozen opponent from this pure-numpy weight dump (no torch in the
    # workers — see bucky.selfplay), refreshed alongside the .zip snapshot.
    snapshot_npz = f"{snapshot_base}.npz"

    # Tiny [64,64] MLP: pin torch threads to 1 so they don't contend with the SubprocVecEnv workers.
    torch.set_num_threads(1)
    torch.autograd.set_detect_anomaly(False)

    from stable_baselines3.common.utils import get_device
    device = get_device(args.device)
    if device.type == "cuda":
        torch.set_float32_matmul_precision("high")
        torch.backends.cudnn.benchmark = True
        print("CUDA device active: TF32 matmul + cudnn.benchmark enabled.")

    # Live streaming to the hub (metrics + status). Created once; each phase builds its own callbacks.
    stream = None
    if args.stream_url:
        from bucky.stream_client import StreamClient
        stream_headers = {"X-Device-Token": args.stream_token} if args.stream_token else None
        stream = StreamClient(args.stream_url, headers=stream_headers)
        stream.start()

    # Mutable handle to the model currently training, so an interrupt saves the live phase's weights.
    state: dict = {"model": None}

    def hp(key, default):
        return hyperparams.get(key, default)

    def update_meta(status: str) -> None:
        """Best-effort refresh of the run's meta.json after a save."""
        try:
            from pathlib import Path
            from app.models import best_eval, write_meta
            meta = {
                "status": status,
                "timesteps_trained": int(getattr(state["model"], "num_timesteps", 0) or 0),
                "best_eval": best_eval(Path("runs"), run_name),
            }
            if cfg:
                meta.update({"name": cfg.get("name"), "version": cfg.get("version"), "config": cfg})
            write_meta(Path("checkpoints"), run_name, meta)
        except Exception:  # noqa: BLE001 — metadata is non-critical
            pass

    def run_phase(stage, *, total_steps, deadline, prev_model, prev_ckpt_path,
                  reset_num_timesteps, is_last):
        """Train one curriculum stage; return (model, phase_ckpt_path). Reused by the single-stage
        path and by FULL_TRAINING. ``prev_model`` continues an in-memory model when the obs space
        matches; when it grew (single→self-play) the weights are expanded from ``prev_ckpt_path``."""
        self_play = stage == Stage.SELF_PLAY_1V1.value
        # Self-play runs in-process (DummyVecEnv: torch loaded once, fits an 8 GB host); single-agent
        # stages keep SubprocVecEnv for cheap parallelism.
        vec_cls = DummyVecEnv if self_play else SubprocVecEnv
        train_env = make_vec_env(
            make_env(stage, domain_rand, reward_config=reward_config),
            n_envs=args.n_envs, seed=args.seed, vec_env_cls=vec_cls,
        )
        eval_env = make_vec_env(
            make_env(stage, domain_rand=False, reward_config=reward_config),
            n_envs=1, seed=args.seed + 1000, vec_env_cls=vec_cls,
        )

        stage_ent_coef = get_stage_config(stage).recommended_ent_coef

        def build_fresh_model():
            return PPO(
                policy="MlpPolicy", env=train_env, verbose=1, tensorboard_log=log_dir, seed=args.seed,
                learning_rate=hp("learning_rate", 3e-4), n_steps=hp("n_steps", 1024),
                batch_size=hp("batch_size", 512), n_epochs=hp("n_epochs", 10),
                gamma=hp("gamma", 0.99), gae_lambda=hp("gae_lambda", 0.95),
                clip_range=hp("clip_range", 0.2), ent_coef=hp("ent_coef", stage_ent_coef),
                vf_coef=hp("vf_coef", 0.5), max_grad_norm=hp("max_grad_norm", 0.5),
                policy_kwargs={"net_arch": list(net_arch)}, device=device,
            )

        if prev_model is None and args.resume_from:
            # Seed from a checkpoint: direct load when spaces match; expand the input layer when only
            # the obs width grew (35→39); fatal on any other mismatch.
            print(f"Resuming from {args.resume_from}")
            try:
                model = PPO.load(args.resume_from, env=train_env, tensorboard_log=log_dir, device=device)
            except ValueError as e:
                if "spaces do not match" not in str(e):
                    raise
                old_obs = PPO.load(args.resume_from, device=device).observation_space.shape[0]
                new_obs = train_env.observation_space.shape[0]
                if old_obs < new_obs:
                    print(f"  obs grew {old_obs}→{new_obs}: expanding input layer (curriculum transfer).")
                    from bucky.selfplay import transfer_weights_expand_obs
                    model = build_fresh_model()
                    transfer_weights_expand_obs(model, args.resume_from, device)
                else:
                    sys.exit(
                        f"\nCannot resume from {args.resume_from}: its observation/action space is "
                        f"incompatible with the current environment (not a simple obs expansion).\n  {e}\n"
                    )
        elif prev_model is None:
            model = build_fresh_model()
        elif prev_model.observation_space.shape[0] == train_env.observation_space.shape[0]:
            prev_model.set_env(train_env)   # same obs (e.g. APPROACH→PUSH): continue the in-memory model
            model = prev_model
        else:
            # Obs grew (PUSH→SELF_PLAY): build fresh and expand the input layer from the prev checkpoint.
            from bucky.selfplay import transfer_weights_expand_obs
            model = build_fresh_model()
            copied, padded = transfer_weights_expand_obs(model, prev_ckpt_path, device)
            model.num_timesteps = int(getattr(prev_model, "num_timesteps", 0) or 0)  # progress stays monotonic
            print(f"  phase transfer: expanded {padded} input layer(s), copied {copied} tensors.")

        state["model"] = model

        if args.compile:
            # EXPERIMENTAL: torch.compile prefixes state_dict keys with "_orig_mod." — verify SB3
            # save/load and ONNX export round-trip before using for real runs.
            mode = "reduce-overhead" if device.type == "cuda" else "default"
            model.policy = torch.compile(model.policy, mode=mode)
            print(f"Policy compiled with torch.compile (mode={mode}).")

        extra_callbacks = []
        if stream is not None:
            from bucky.callbacks import MetricsCallback
            extra_callbacks.append(MetricsCallback(stream))
            if args.viz:
                from bucky.callbacks import LiveVizCallback, SelfPlayVizCallback
                extra_callbacks.insert(0, SelfPlayVizCallback(stream, opponent_path=snapshot_npz)
                                       if self_play else
                                       LiveVizCallback(stream, stage=stage, domain_rand=False))

        # Self-play bootstrap: freeze the policy as the initial opponent, then refresh it periodically.
        if self_play:
            from bucky.callbacks import SelfPlaySnapshotCallback
            from bucky.selfplay import export_policy_npz
            pool_size = int(cfg.get("opponent_pool_size", 5))
            model.save(snapshot_base)
            export_policy_npz(model, snapshot_npz)
            seed_npz = f"{snapshot_base}_0.npz"
            export_policy_npz(model, seed_npz)
            train_env.env_method("set_opponent_pool", [seed_npz])
            eval_env.env_method("set_opponent_pool", [seed_npz])
            extra_callbacks.append(
                SelfPlaySnapshotCallback(snapshot_base, every=max(50_000 // args.n_envs, 1), pool_size=pool_size)
            )

        callbacks = [
            EvalCallback(eval_env, best_model_save_path=ckpt_dir, log_path=log_dir,
                         eval_freq=max(10_000 // args.n_envs, 1), n_eval_episodes=20, deterministic=True),
            *extra_callbacks,
        ]
        if cfg.get("save_step_checkpoints"):
            callbacks.append(CheckpointCallback(save_freq=max(50_000 // args.n_envs, 1),
                                                save_path=ckpt_dir, name_prefix="model"))
        if args.fed_server and args.fed_group:
            from bucky.fedavg import FedSyncCallback
            callbacks.append(FedSyncCallback(
                args.fed_server, args.fed_token or "", args.fed_group, run_name,
                every=args.fed_every, shards=args.fed_shards, weight=args.n_envs, verbose=1,
            ))
        if deadline is not None:
            callbacks.append(StopAtTime(deadline))

        model.learn(total_timesteps=total_steps, callback=callbacks, progress_bar=True,
                    reset_num_timesteps=reset_num_timesteps)

        phase_ckpt = None
        if not is_last:   # persist so the next phase can continue / expand from it
            phase_ckpt = f"{ckpt_dir}/{stage.lower()}_model"
            model.save(phase_ckpt)
        for env in (train_env, eval_env):
            try:
                env.close()
            except Exception:  # noqa: BLE001 — best-effort cleanup
                pass
        return model, phase_ckpt

    # ── Phase plan: FULL_TRAINING → APPROACH/PUSH/SELF_PLAY; any other stage → a single phase ──
    time_based = args.until is not None or args.duration is not None
    now = time.time()
    window = None
    if time_based:
        # --until wins over --duration; the step budget is a huge sentinel so time binds.
        end = float(args.until) if args.until is not None else now + float(args.duration)
        window = max(0.0, end - now)

    if args.stage == Stage.FULL_TRAINING.value:
        split = cfg.get("full_training_split") or [r for _, r in FULL_TRAINING_PHASES]
        ssum = float(sum(split)) or 1.0
        ratios = [s / ssum for s in split]
        stages = [st.value for st, _ in FULL_TRAINING_PHASES]
    else:
        ratios = [1.0]
        stages = [args.stage]

    plan = []   # (stage, total_steps, deadline, reset_num_timesteps, is_last)
    cum = 0.0
    remaining = args.timesteps
    for i, (stage, r) in enumerate(zip(stages, ratios)):
        is_last = i == len(stages) - 1
        if time_based:
            cum += r
            plan.append((stage, args.timesteps, now + cum * window, i == 0, is_last))
        else:
            steps = remaining if is_last else int(round(args.timesteps * r))
            remaining -= steps
            plan.append((stage, steps, None, i == 0, is_last))

    if stream:
        stream.send({"type": "trainer_status", "phase": "started", "run_name": run_name, "run_type": "train"})
    budget = f"~{window / 60:.1f} min (until time)" if time_based else f"{args.timesteps:,} steps"
    print(f"\nTraining {run_name} | stage={args.stage} | {budget} | {len(plan)} phase(s) | {args.n_envs} envs")

    try:
        prev_model, prev_ckpt = None, None
        for i, (stage, steps, deadline, reset, is_last) in enumerate(plan):
            if len(plan) > 1:
                if stream:
                    stream.send({"type": "trainer_status",
                                 "phase": f"phase {i + 1}/{len(plan)}: {stage}",
                                 "run_name": run_name, "run_type": "train"})
                print(f"\n=== phase {i + 1}/{len(plan)}: {stage} ===")
            prev_model, prev_ckpt = run_phase(
                stage, total_steps=steps, deadline=deadline, prev_model=prev_model,
                prev_ckpt_path=prev_ckpt, reset_num_timesteps=reset, is_last=is_last,
            )
    except (KeyboardInterrupt, EOFError, BrokenPipeError, ConnectionResetError):
        # Hub "Kill" sends SIGINT precisely so we can checkpoint the live phase before exiting.
        print("\nInterrupted — saving model…")
        save_path = f"{ckpt_dir}/interrupted_model"
        if stream:
            stream.send({"type": "trainer_status", "phase": "saving", "run_name": run_name, "run_type": "train"})
        try:
            if state["model"] is not None:
                state["model"].save(save_path)
                print(f"Saved to {save_path}.zip")
        except Exception as exc:  # noqa: BLE001
            print(f"Could not save on interrupt: {exc}")
        update_meta("interrupted")
        if stream:
            stream.send({"type": "trainer_status", "phase": "interrupted", "run_name": run_name, "run_type": "train"})
    else:
        save_path = f"{ckpt_dir}/final_model"
        state["model"].save(save_path)
        update_meta("done")
        if stream:
            stream.send({"type": "trainer_status", "phase": "done", "run_name": run_name, "run_type": "train"})
        print(f"\nDone. Model saved to {save_path}.zip")
        if stream:
            time.sleep(0.3)  # let the final status frame flush to the hub
            stream.stop()


if __name__ == "__main__":
    main()
