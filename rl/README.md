# Bucky RL

PPO training stack for the Bucky RoboCup Junior omni-drive robot.

## Install

```bash
uv sync          # installs all deps into .venv
uv sync --extra dev  # also installs ruff
```

## Train

```bash
# Stage 1 — reach the ball (fast, good for iteration)
uv run python scripts/train.py --stage APPROACH_STATIC_BALL --timesteps 200000

# Stage 2 — score in an empty goal
uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --timesteps 1000000 --n-envs 8

# With live visualization (open http://localhost:5173/viz in the SvelteKit app)
uv run python scripts/train.py --stage PUSH_TO_EMPTY_GOAL --viz-port 8765

# TensorBoard
tensorboard --logdir runs/
```

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

1. Create `rl/physics/cpp_backend.py` (pybind11 wrapper).
2. Implement the `PhysicsBackend` ABC (`reset`, `step`, `dt`).
3. Pass to env: `BuckySingleEnv(physics=CppBackend())` — add a `physics` kwarg to `__init__`.
4. The observation and reward code is backend-agnostic; no other changes needed.

## Adding 2v2 self-play

1. Implement `BuckyTeamEnv` in `envs/bucky_team.py` as a `pettingzoo.ParallelEnv`.
2. Extend `PyPhysics` to handle N robots with inter-robot collisions.
3. Use `Stage.SELF_PLAY_2V2` config from `curriculum.py`.
4. Train with a multi-agent SB3 wrapper or MAPPO.
