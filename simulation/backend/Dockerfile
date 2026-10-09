# Backend: FastAPI + training stack + the firmware simulator, run with uv (installs the project
# editable so scripts/ and checkpoints/ resolve relative to /app at runtime).
#
# Build context is the REPOSITORY ROOT (see simulation/docker-compose.yml and /.dockerignore):
#   docker build -f simulation/backend/Dockerfile .

# ── Stage 1: compile the real robot firmware (robot/src) into the `bucky_fw` extension ───────
# Same base image as the runtime stage, so the extension matches its Python ABI. The compiler
# toolchain stays in this stage; the runtime image only receives the built module.
FROM python:3.12-slim AS firmware

RUN apt-get update && apt-get install -y --no-install-recommends g++ cmake ninja-build \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir "pybind11==3.1.0"

WORKDIR /src
COPY robot/src robot/src
COPY robot/sim robot/sim
# Only the stdlib-only build helper, not the whole backend (keeps this stage's cache stable).
COPY simulation/backend/bucky/__init__.py simulation/backend/bucky/__init__.py
COPY simulation/backend/bucky/firmware/__init__.py simulation/backend/bucky/firmware/build.py \
     simulation/backend/bucky/firmware/
ENV BUCKY_ROBOT_DIR=/src/robot
RUN cd simulation/backend && python -m bucky.firmware.build -v

# ── Stage 2: runtime ──────────────────────────────────────────────────────────────────────
FROM python:3.12-slim

# uv for fast, locked dependency installs.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app
ENV UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    PATH="/app/.venv/bin:$PATH"

# tcmalloc: faster allocator for the allocation-heavy multi-process rollout collection.
# The backend auto-detects and LD_PRELOADs it into trainer subprocesses (see app/jobs.py).
RUN apt-get update && apt-get install -y --no-install-recommends libtcmalloc-minimal4 \
    && rm -rf /var/lib/apt/lists/*

# Dependency layer — cached until pyproject.toml or uv.lock changes.
# --no-install-project installs only external deps, not the project itself.
# This keeps this heavy layer (torch, sb3, tensorboard) intact on source-only changes.
COPY simulation/backend/pyproject.toml simulation/backend/uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev --no-install-project

# Firmware for the Lab (bucky.firmware): the sources (program list, test names) plus the
# extension built in stage 1. BUCKY_FW_NO_BUILD: use that build, never compile at runtime
# (this image has no compiler).
COPY robot/src /robot/src
COPY --from=firmware /src/robot/sim/build/h562rg-RelWithDebInfo/bucky_fw*.so \
     /robot/sim/build/h562rg-RelWithDebInfo/
ENV BUCKY_ROBOT_DIR=/robot \
    BUCKY_FW_NO_BUILD=1

# Source layer — only re-runs the cheap project-install step on code changes.
COPY simulation/backend/bucky ./bucky
COPY simulation/backend/app ./app
COPY simulation/backend/scripts ./scripts
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev

EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
