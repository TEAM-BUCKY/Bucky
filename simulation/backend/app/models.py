"""Per-model training config + on-disk metadata.

A *model* is a named, versioned training configuration. Today the trainer's
hyperparameters, network shape and reward weights were hardcoded in
``scripts/train.py``; this module makes them a first-class, serializable config so
the UI can create "different kinds of models" and the admin panel can show what
each checkpoint was trained with.

On disk, each run directory carries two sidecars next to its ``*.zip`` checkpoints:
  * ``config.json`` — the exact :class:`ModelConfig` the trainer was launched with.
  * ``meta.json``   — human/admin-facing metadata (name, version, status, best eval,
                       who trained it), updated at launch and on completion.

``list_models`` scans the checkpoints tree and enriches each run with its metadata,
inferring a minimal record for legacy directories that predate this system so old
(now-stale) checkpoints still appear in the browser.
"""
from __future__ import annotations

import json
import logging
import re
import time
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, Field

log = logging.getLogger(__name__)

META_FILE = "meta.json"
CONFIG_FILE = "config.json"


# ── config schema ─────────────────────────────────────────────────────────────
class Hyperparams(BaseModel):
    """PPO hyperparameters. Defaults mirror the previously-hardcoded values in
    ``scripts/train.py`` so a model created with no overrides trains identically."""

    learning_rate: float = 3e-4
    n_steps: int = 1024
    batch_size: int = 512
    n_epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5


class StopSpec(BaseModel):
    # steps: train for N timesteps; duration: N wall-clock seconds; until: epoch seconds.
    kind: str = "steps"
    value: float = 200_000


class ResumeRef(BaseModel):
    run: str
    checkpoint: str


class ModelConfig(BaseModel):
    """Everything needed to train one model version. Dumped to ``config.json`` and
    handed to ``scripts/train.py`` via ``--config``."""

    name: str = "model"
    version: str = "1"
    stage: str = "APPROACH_STATIC_BALL"
    n_envs: int = 16
    seed: int = 0
    domain_rand: bool = True
    stop: Optional[StopSpec] = None
    hyperparams: Hyperparams = Field(default_factory=Hyperparams)
    net_arch: list[int] = Field(default_factory=lambda: [64, 64])
    # Partial map of RewardConfig w_* weights; missing keys keep their tuned default.
    reward_weights: dict[str, float] = Field(default_factory=dict)
    # Keep the periodic `model_<N>_steps.zip` snapshots. Off by default — only the
    # best + final (and self-play opponent) checkpoints are kept, which is what you
    # almost always want; the step snapshots pile up disk fast.
    save_step_checkpoints: bool = False
    resume_from: Optional[ResumeRef] = None


# ── run-name derivation ───────────────────────────────────────────────────────
_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


def sanitize_segment(text: str) -> str:
    """Collapse anything unsafe in a single path segment to ``_`` (no separators,
    no parent refs). Mirrors the safety contract of ``JobManager._is_safe_name``."""
    cleaned = _SAFE.sub("_", (text or "").strip()).strip("._-")
    return cleaned or "model"


def run_name_for(name: str, version: str) -> str:
    """Filesystem run name for a (name, version) pair — a single safe segment.

    Kept flat (``<name>_v<version>``) so the existing resume/match path resolution
    in :mod:`app.jobs` (``checkpoints/<run>/<ckpt>``) needs no change; the structured
    name/version live in ``meta.json``.
    """
    return f"{sanitize_segment(name)}_v{sanitize_segment(version)}"


# ── metadata sidecars ─────────────────────────────────────────────────────────
def _meta_path(ckpt_root: Path, run: str) -> Path:
    return ckpt_root / run / META_FILE


def read_meta(ckpt_root: Path, run: str) -> dict | None:
    path = _meta_path(ckpt_root, run)
    try:
        if path.is_file():
            return json.loads(path.read_text())
    except Exception:  # noqa: BLE001 — a corrupt sidecar must not break listing
        log.warning("Could not read meta for run %s", run)
    return None


def write_meta(ckpt_root: Path, run: str, meta: dict) -> None:
    """Merge ``meta`` into the run's existing metadata and persist it."""
    run_dir = ckpt_root / run
    try:
        run_dir.mkdir(parents=True, exist_ok=True)
        current = read_meta(ckpt_root, run) or {}
        current.update(meta)
        (run_dir / META_FILE).write_text(json.dumps(current, indent=2))
    except Exception as exc:  # noqa: BLE001
        log.warning("Could not write meta for run %s: %s", run, exc)


def write_config(ckpt_root: Path, run: str, config: dict) -> Path:
    run_dir = ckpt_root / run
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / CONFIG_FILE
    path.write_text(json.dumps(config, indent=2))
    return path


# ── eval score lookup ─────────────────────────────────────────────────────────
def best_eval(runs_root: Path, run: str) -> float | None:
    """Best mean eval return from SB3's ``runs/<run>/evaluations.npz`` if present."""
    path = runs_root / run / "evaluations.npz"
    if not path.is_file():
        return None
    try:
        import numpy as np

        with np.load(path) as data:
            results = data["results"]  # shape (n_evals, n_episodes)
            if results.size == 0:
                return None
            return float(results.mean(axis=1).max())
    except Exception:  # noqa: BLE001 — optional, best-effort
        return None


# ── listing ───────────────────────────────────────────────────────────────────
def _checkpoint_entries(run_dir: Path) -> list[dict]:
    entries = []
    for f in sorted(run_dir.glob("*.zip")):
        try:
            st = f.stat()
            entries.append({"file": f.name, "size": st.st_size, "mtime": st.st_mtime})
        except OSError:
            continue
    return entries


def model_record(ckpt_root: Path, runs_root: Path, run: str) -> dict | None:
    """Build one enriched model record for a run dir, or ``None`` if it has no
    checkpoints. Legacy dirs (no ``meta.json``) get a minimal inferred record."""
    run_dir = ckpt_root / run
    checkpoints = _checkpoint_entries(run_dir)
    if not checkpoints:
        return None

    meta = read_meta(ckpt_root, run)
    try:
        created_at = run_dir.stat().st_mtime
    except OSError:
        created_at = None

    if meta is None:
        # Inferred record for a checkpoint that predates the metadata system.
        record = {
            "run": run,
            "name": run,
            "version": None,
            "stage": run.split("_seed")[0] if "_seed" in run else None,
            "status": "legacy",
            "created_at": created_at,
            "created_by": None,
            "has_meta": False,
        }
    else:
        record = {
            "run": run,
            "name": meta.get("name", run),
            "version": meta.get("version"),
            "stage": meta.get("config", {}).get("stage") or meta.get("stage"),
            "status": meta.get("status"),
            "created_at": meta.get("created_at", created_at),
            "created_by": meta.get("created_by"),
            "timesteps_trained": meta.get("timesteps_trained"),
            "parent": meta.get("parent"),
            "config": meta.get("config"),
            "has_meta": True,
        }
    # Prefer eval logs on disk; fall back to a value reported in meta (e.g. a remote
    # worker whose TensorBoard logs stayed on the guest device).
    score = best_eval(runs_root, run)
    if score is None and meta is not None:
        score = meta.get("best_eval")
    record["best_eval"] = score
    record["checkpoints"] = checkpoints
    return record


def list_models(ckpt_root: Path, runs_root: Path) -> list[dict]:
    """All runs with at least one checkpoint, newest first."""
    if not ckpt_root.is_dir():
        return []
    records = []
    for d in ckpt_root.iterdir():
        if not d.is_dir():
            continue
        rec = model_record(ckpt_root, runs_root, d.name)
        if rec:
            records.append(rec)
    records.sort(key=lambda r: r.get("created_at") or 0, reverse=True)
    return records


def launch_meta(config: ModelConfig, run: str, created_by: str) -> dict:
    """Initial metadata written when a run is launched."""
    return {
        "name": config.name,
        "version": config.version,
        "run": run,
        "config": config.model_dump(),
        "status": "training",
        "created_at": time.time(),
        "created_by": created_by,
        "parent": (
            f"{config.resume_from.run}/{config.resume_from.checkpoint}"
            if config.resume_from else None
        ),
    }
