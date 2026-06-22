# Backend: FastAPI + training stack, run with uv (installs the project editable so
# scripts/ and checkpoints/ resolve relative to /app at runtime).
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
COPY pyproject.toml uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev --no-install-project

# Source layer — only re-runs the cheap project-install step on code changes.
COPY bucky ./bucky
COPY app ./app
COPY scripts ./scripts
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev

EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
