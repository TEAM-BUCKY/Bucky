# Backend: FastAPI + training stack, run with uv (installs the project editable so
# scripts/ and checkpoints/ resolve relative to /app at runtime).
FROM python:3.12-slim

# uv for fast, locked dependency installs.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app
ENV UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    PATH="/app/.venv/bin:$PATH"

# Dependency layer (cached unless pyproject changes). The source is needed because
# the project installs itself; a stub import keeps the build self-contained.
COPY pyproject.toml ./
COPY bucky ./bucky
COPY app ./app
COPY scripts ./scripts
RUN uv sync --no-dev

EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
