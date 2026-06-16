#!/usr/bin/env python
"""Export trained policy actor to ONNX and verify against SB3 output.

Usage:
    uv run python scripts/export_onnx.py \
        --checkpoint checkpoints/APPROACH_STATIC_BALL_seed0/final_model \
        --output policy.onnx
"""
from __future__ import annotations
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import numpy as np
import torch
from stable_baselines3 import PPO
from rl.envs.bucky_single import BuckySingleEnv
from rl.obs import OBS_DIM


class OnnxExportableActor(torch.nn.Module):
    """Wraps SB3's MlpPolicy actor for ONNX export (actor-only, deterministic)."""

    def __init__(self, policy):
        super().__init__()
        self.mlp_extractor = policy.mlp_extractor
        self.action_net = policy.action_net

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        features = self.mlp_extractor.forward_actor(obs)
        return torch.tanh(self.action_net(features))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default="policy.onnx")
    args = parser.parse_args()

    env = BuckySingleEnv(stage="APPROACH_STATIC_BALL", domain_rand=False)
    model = PPO.load(args.checkpoint, env=env)
    policy = model.policy
    policy.eval()

    actor = OnnxExportableActor(policy).cpu()
    dummy_input = torch.zeros(1, OBS_DIM, dtype=torch.float32)

    torch.onnx.export(
        actor,
        dummy_input,
        args.output,
        opset_version=17,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
    )
    print(f"Exported to {args.output}")

    import onnxruntime as ort
    import onnx

    onnx.checker.check_model(args.output)
    sess = ort.InferenceSession(args.output)

    obs_np = np.random.default_rng(0).random((1, OBS_DIM)).astype(np.float32)
    obs_t = torch.from_numpy(obs_np)

    with torch.no_grad():
        sb3_out = actor(obs_t).numpy()

    ort_out = sess.run(["action"], {"observation": obs_np})[0]

    max_diff = float(np.max(np.abs(sb3_out - ort_out)))
    print(f"Max abs diff SB3 vs ONNX: {max_diff:.2e}")
    assert max_diff < 1e-4, f"ONNX mismatch: {max_diff}"
    print("ONNX verification PASSED ✓")
    env.close()


if __name__ == "__main__":
    main()
