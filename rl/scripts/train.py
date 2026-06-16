#!/usr/bin/env python
"""Train Bucky RL agent with PPO.

Usage:
    uv run python scripts/train.py --stage APPROACH_STATIC_BALL --timesteps 200000
    uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --timesteps 1000000 --n-envs 8
"""
from __future__ import annotations
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from rl.curriculum import Stage
from rl.envs.bucky_single import BuckySingleEnv
from rl.viz_server import VizServer


def make_env(stage: str, domain_rand: bool, viz_server: VizServer | None = None):
    def _init():
        cb = viz_server.send_state if viz_server else None
        return BuckySingleEnv(stage=stage, domain_rand=domain_rand, viz_callback=cb)
    return _init


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default="APPROACH_STATIC_BALL",
                        choices=[s.value for s in Stage])
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--no-domain-rand", action="store_true")
    parser.add_argument("--viz-port", type=int, default=0,
                        help="WebSocket port for live visualization (0 = disabled)")
    args = parser.parse_args()

    run_name = args.run_name or f"{args.stage}_seed{args.seed}"
    log_dir = f"runs/{run_name}"
    ckpt_dir = f"checkpoints/{run_name}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(ckpt_dir, exist_ok=True)

    domain_rand = not args.no_domain_rand

    viz_server = None
    if args.viz_port > 0:
        viz_server = VizServer(port=args.viz_port)
        viz_server.start()
        print(f"Viz server → ws://localhost:{args.viz_port}  (open http://localhost:5173/viz)")

    # SubprocVecEnv pickles the env factory — VizServer contains threading locks that can't
    # be pickled, so we never pass viz_server to the train envs. Visualization is fed from
    # the eval env instead, which runs in-process via DummyVecEnv.
    train_env = make_vec_env(
        make_env(args.stage, domain_rand),
        n_envs=args.n_envs,
        seed=args.seed,
        vec_env_cls=SubprocVecEnv,
    )

    eval_env = make_vec_env(
        make_env(args.stage, domain_rand=False, viz_server=viz_server),
        n_envs=1,
        seed=args.seed + 1000,
    )

    # PPO hyperparameters tuned for low-dim continuous control (17-dim obs, 3-dim act).
    # net_arch=[64,64] chosen to fit Cortex-M4F after int8 quantization.
    # n_steps=512, batch_size=256 → 8 minibatches per update.
    model = PPO(
        policy="MlpPolicy",
        env=train_env,
        verbose=1,
        tensorboard_log=log_dir,
        seed=args.seed,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs={"net_arch": [64, 64]},
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
        CheckpointCallback(
            save_freq=max(50_000 // args.n_envs, 1),
            save_path=ckpt_dir,
            name_prefix="rl_model",
        ),
    ]

    print(f"\nTraining {run_name} | stage={args.stage} | {args.timesteps:,} steps | {args.n_envs} envs")
    model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    model.save(f"{ckpt_dir}/final_model")
    print(f"\nDone. Model saved to {ckpt_dir}/final_model.zip")

    if viz_server:
        viz_server.stop()


if __name__ == "__main__":
    main()
