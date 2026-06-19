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

- Public (no auth): `GET /api/status`, `GET /api/runs`, `GET /api/health`, `WS /api/stream`.
- Gated (HTTP Basic, `APP_USERNAME`/`APP_PASSWORD` from `.env`): `POST /api/jobs`, `POST /api/jobs/stop`.

Launching a job from `POST /api/jobs` spawns `scripts/train.py` (or `play.py`) as a
subprocess and streams its frames to all `/api/stream` viewers. You can also run the
scripts directly, as below.

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

# TensorBoard
tensorboard --logdir runs/
```

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
