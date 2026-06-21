"""Federated weight-averaging (FedAvg) for distributing one training run over devices.

A distributed run is split into N **shards**, each training its own environments. Every
``sync_every`` steps each shard pushes its full policy weights to the coordinator and
pulls back the elementwise average across all shards, then loads it and keeps training.
Over many rounds the shards stay in lock-step around a shared policy while collecting
N× the experience — so several modest CPU boxes train one model together.

Weights travel as ``.npz`` (named numpy arrays) so the **coordinator averages them with
numpy alone — no torch on the server**. Only the trainer side (which already has torch)
converts to/from the policy ``state_dict``.
"""
from __future__ import annotations

import io
import time

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

# Client-side sync barrier: how long a shard waits for the round's average before giving
# up and continuing solo (so one dead shard can't wedge the others forever).
PULL_TIMEOUT = 180.0
PULL_POLL = 2.0


# ── weight (de)serialisation ────────────────────────────────────────────────────
def policy_state_to_arrays(model) -> dict[str, np.ndarray]:
    """The policy's full ``state_dict`` as plain numpy arrays (keys preserved)."""
    return {k: v.detach().cpu().numpy() for k, v in model.policy.state_dict().items()}


def load_arrays_into_policy(model, arrays: dict[str, np.ndarray]) -> None:
    """Load averaged numpy arrays back into the policy (shapes/dtypes preserved)."""
    import torch

    sd = model.policy.state_dict()
    new_sd = {
        k: torch.as_tensor(arrays[k]).reshape(tuple(sd[k].shape)).to(sd[k].dtype)
        for k in sd
        if k in arrays
    }
    model.policy.load_state_dict(new_sd, strict=False)


def average_arrays(states: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    """Elementwise mean of several weight dicts (FedAvg). All dicts share their keys."""
    keys = states[0].keys()
    return {k: np.mean([s[k] for s in states], axis=0) for k in keys}


def to_npz_bytes(arrays: dict[str, np.ndarray]) -> bytes:
    buf = io.BytesIO()
    np.savez(buf, **arrays)
    return buf.getvalue()


def from_npz_bytes(data: bytes) -> dict[str, np.ndarray]:
    with np.load(io.BytesIO(data)) as d:
        return {k: d[k] for k in d.files}


# ── trainer-side sync callback ───────────────────────────────────────────────────
class FedSyncCallback(BaseCallback):
    """Push this shard's weights each round and load back the averaged weights.

    The pull is a deliberate barrier: training blocks until the coordinator has the
    round's average (or this shard times out and proceeds solo). A failed request never
    kills training — a missed round just means this shard keeps its own weights.
    """

    def __init__(self, api_base: str, token: str, group: str, shard: str,
                 every: int, shards: int, verbose: int = 0) -> None:
        super().__init__(verbose)
        self._api = api_base.rstrip("/")
        self._headers = {"X-Fed-Token": token}
        self._group = group
        self._shard = shard
        self._every = max(1, every)
        self._shards = max(1, shards)
        self._last_round = -1

    def _on_step(self) -> bool:
        rnd = self.num_timesteps // self._every
        if rnd <= self._last_round:
            return True
        self._last_round = rnd
        self._sync(rnd)
        return True

    def _sync(self, rnd: int) -> None:
        import requests

        try:
            payload = to_npz_bytes(policy_state_to_arrays(self.model))
            requests.post(
                f"{self._api}/dist/{self._group}/push",
                headers=self._headers,
                params={"round": rnd, "shard": self._shard, "shards": self._shards},
                files={"file": ("weights.npz", payload, "application/octet-stream")},
                timeout=60,
            ).raise_for_status()

            deadline = time.time() + PULL_TIMEOUT
            while time.time() < deadline:
                r = requests.get(
                    f"{self._api}/dist/{self._group}/pull",
                    headers=self._headers, params={"round": rnd}, timeout=30,
                )
                if r.status_code == 200:
                    load_arrays_into_policy(self.model, from_npz_bytes(r.content))
                    if self.verbose:
                        print(f"[fed] {self._shard} synced round {rnd}", flush=True)
                    return
                if r.status_code == 204:
                    time.sleep(PULL_POLL)
                    continue
                return  # unexpected status — skip this round's sync
        except requests.RequestException as exc:
            if self.verbose:
                print(f"[fed] sync skipped for round {rnd}: {exc}", flush=True)
