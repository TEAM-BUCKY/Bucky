# Bucky backend

FastAPI service plus the training/simulation code (Python package `bucky`) for the
Bucky RoboCup Junior omni-drive robot. The soccer PPO trainer is the one built-in job;
the service is structured so more job types can be added later.

For deploying the whole stack (frontend + backend + HTTPS) on a VPS, see
[`../README.md`](../README.md). This file covers running the backend on its own.

## Install

Needs [uv](https://docs.astral.sh/uv/).

```bash
uv sync                # install runtime deps into .venv
uv sync --extra dev    # also install pytest + ruff
```

## Run the API server

```bash
echo "APP_PASSWORD=dev" > .env     # enable control actions locally (omit → view-only)
uv run uvicorn app.main:app --reload --port 8000
```

- Public (no auth): `GET /api/status`, `GET /api/runs`, `GET /api/models`, `GET /api/queue`,
  `GET /api/devices`, `GET /api/health`, `WS /api/stream`.
- Gated (HTTP Basic, `APP_USERNAME`/`APP_PASSWORD` from `.env`): `POST /api/jobs`,
  `POST /api/jobs/stop`, the `POST/DELETE /api/queue*` routes, `GET /api/models/{run}/{ckpt}/download`,
  `DELETE /api/models/...`, and `POST/DELETE /api/devices` (register/revoke guest devices).
- Device-token gated (guest workers): `POST /api/worker/{lease,heartbeat,complete}`,
  `POST /api/models/{run}/upload`.

Launching a job from `POST /api/jobs` spawns `scripts/train.py` (or `play.py`) as a
subprocess and streams its frames to all `/api/stream` viewers. You can also run the
scripts directly, as below.

## Models & the admin panel

Every training run carries a per-model **config** and **metadata**: create a named,
versioned model in the web admin panel (`/admin`) with its own PPO hyperparameters,
network architecture (`net_arch`) and reward-term weights. These are written as
`checkpoints/<run>/config.json` + `meta.json` and threaded into `scripts/train.py` via
`--config`. The admin panel browses, downloads and deletes models; running
`train.py --config <model.json>` reproduces a model exactly.

## Distributed training (guest devices)

The server holds a shared job **queue**. Besides the server's own in-process worker, any
trusted machine can register as a **guest device** and lease jobs from that queue — so you
can keep training locally while all checkpoints land centrally on the server.

```bash
# On the server: register a device in the admin panel (Devices tab) → copy its token.
# On the guest machine (from this backend/ directory):
uv run python scripts/worker.py --server https://your-bucky-host --token <device-token>
```

The worker polls for jobs, runs `scripts/train.py` locally, streams metrics/status back (the
browser's device selector lets you watch any device), and uploads the resulting checkpoints.
A lease that stops heart-beating is reclaimed and the job returns to the queue.

Set `ENABLE_LOCAL_WORKER=0` to make the server a pure coordinator/store that only guest
devices train for (default `1` — the server also trains, preserving single-machine use).

### Many runs at once, and stopping them

Each node runs several jobs concurrently: the server's in-process worker runs up to
`LOCAL_SLOTS` (default `1`), and a guest worker runs up to `--slots` (default `1`,
or `BUCKY_SLOTS`). The live runs are listed by `GET /api/active` and in the UI's "Active
runs" panel, each with a **Stop** button. `POST /api/jobs/<run>/stop` stops one run by name:
a local run is checkpointed and terminated; a run leased to a device is asked to stop via
its next heartbeat — the worker SIGINTs its trainer (which checkpoints), uploads, and frees
the slot.

### Distributing one run across devices (federated)

A single run can be split into **N shards** that train their own environments and average
their policy weights through the coordinator every `sync_every` steps (FedAvg). Enable it in
the launch form ("Distribute across devices") or via the `distributed` field:

```bash
curl -u admin:pw -X POST https://your-bucky-host/api/jobs -H 'content-type: application/json' \
  -d '{"mode":"train","stage":"SELF_PLAY_1V1","name":"team","version":"1",
       "distributed":{"shards":3,"sync_every":50000}}'
```

The shards (`team_shard0…2`) are leased across whatever workers are free (raise `LOCAL_SLOTS`
/ `--slots` to land several on one box). They stay in lock-step around a shared policy while
collecting N× the experience; any shard's final checkpoint is the federated model.

## Train

```bash
# Stage 1 — reach the ball (fast, good for iteration)
uv run python scripts/train.py --stage APPROACH_STATIC_BALL --timesteps 200000

# Stage 2 — score in an empty goal
uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --timesteps 1000000 --n-envs 8

# Resume from a checkpoint
uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --resume-from checkpoints/PUSH_TO_EMPTY_GOAL_seed0/final_model

# Stop by wall-clock instead of step count (saves a final model when the time is up)
uv run python scripts/train.py --stage APPROACH_STATIC_BALL --duration 3600          # train 1 hour
uv run python scripts/train.py --stage APPROACH_STATIC_BALL --until 1781990000        # train until an epoch time

# Watch the live field animate (off by default — headless trains faster)
uv run python scripts/train.py --stage SELF_PLAY_1V1 --stream-url ws://localhost:8000/api/ingest --viz

# TensorBoard
tensorboard --logdir runs/
```

Runs are **headless by default**: metrics/status still stream to the UI, but the animated
field (an extra in-process "shadow" rollout) only runs with `--viz` — in the web UI, the
launch form's **"Watch live"** toggle. The robot's omni-drive action also carries a 4th
**kick** channel, so a trained policy can shoot; the skilled-play reward weights (kick goal,
bank shot, steal, …) are editable in the reward-weights panel.

From the web UI you can pick a stop condition (steps / duration / until a time) per run and
**queue** several runs — each launches when the previous finishes, with an optional scheduled
start time, so training continues unattended. The queue is persisted under `state/queue.json`.

For **live visualization in the browser**, don't pass a stream URL by hand — start the API
server and the frontend (see `../README.md`) and launch the job from the website. The
backend wires `--stream-url` to its internal ingest endpoint for you. (`scripts/train.py`
does accept `--stream-url ws://host/api/ingest` directly if you want to stream a manual run.)

## Evaluate

```bash
uv run python scripts/eval.py --checkpoint checkpoints/PUSH_TO_EMPTY_GOAL_seed0/final_model
```

## Export ONNX

```bash
uv run python scripts/export_onnx.py \
  --checkpoint checkpoints/PUSH_TO_EMPTY_GOAL_seed0/final_model \
  --output policy.onnx
# Prints: "ONNX verification PASSED ✓"
```

## Tests

```bash
uv run pytest -v
```

## Observation design (17 dims, robot-egocentric, SI units)

| Idx   | Feature                          | Notes                        |
|-------|----------------------------------|------------------------------|
| 0–1   | ball bearing (sin, cos)          | avoids angle wraparound      |
| 2     | ball distance                    | normalized by field diagonal |
| 3–4   | ball velocity (vx, vy)           | robot frame, m/s             |
| 5–6   | own velocity (vx, vy)            | robot frame, m/s             |
| 7     | own angular velocity ω           | normalized by max ω          |
| 8–9   | heading→goal (sin, cos)          | BNO085-relative              |
| 10–12 | nearest edge (sin, cos, prox)    | from line-sensor ring        |
| 13    | over-goal-area flag              | black detected               |
| 14–16 | teammate (x, y, has_ball)        | zeroed in stage 1            |

**Action space:** `Box(-1, 1, shape=(3,))` = `[vx, vy, ω]`, scaled to 1 m/s / 6 rad/s.

**Reward terms:** `ball_to_goal` (primary, potential-based), `approach` (potential-based),
`possession`, `goal` (terminal +), `out_of_bounds` (terminal −), `spin`, `time_penalty`,
`action_magnitude`. All terms logged separately to TensorBoard.

## Adding the C++ backend

1. Create `bucky/physics/cpp_backend.py` (pybind11 wrapper).
2. Implement the `PhysicsBackend` ABC (`reset`, `step`, `dt`).
3. Pass to env: `BuckySingleEnv(physics=CppBackend())` — add a `physics` kwarg to `__init__`.
4. The observation and reward code is backend-agnostic; no other changes needed.

## Adding 2v2 self-play

1. Implement `BuckyTeamEnv` in `envs/bucky_team.py` as a `pettingzoo.ParallelEnv`.
2. Extend `PyPhysics` to handle N robots with inter-robot collisions.
3. Use `Stage.SELF_PLAY_2V2` config from `curriculum.py`.
4. Train with a multi-agent SB3 wrapper or MAPPO.
