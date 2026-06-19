# Bucky simulation & training platform

A self-contained stack for running and visualizing training jobs in the browser.
The website is public; control actions (launching/stopping jobs) sit behind a
username/password set in `.env`.

```
simulation/
├── frontend/            SvelteKit SPA (static build, served by nginx)
├── backend/             FastAPI app + the training/simulation code (package: bucky)
├── docker-compose.yml   Traefik (HTTPS) + backend + frontend
└── .env.example         Copy to .env and fill in
```

## Architecture

- **frontend** — static SPA. Reads the public live stream over WebSocket and calls
  the backend at `/api`. Control actions send HTTP Basic credentials.
- **backend** — FastAPI:
  - Public: `GET /api/status`, `GET /api/runs`, `GET /api/health`, `WS /api/stream`.
  - Gated (Basic auth): `POST /api/jobs`, `POST /api/jobs/stop`.
  - Internal: `WS /api/ingest` (the spawned trainer process streams frames here,
    token-gated).
  The backend spawns `scripts/train.py` / `scripts/play.py` as subprocesses and fans
  their frames out to all stream clients.
- **traefik** — terminates HTTPS (Let's Encrypt), serves the frontend at
  `https://${DOMAIN}` and routes `/api` to the backend.

## Deploy on a VPS

1. Point a DNS record for your domain at the server's IP.
2. Install Docker + the Compose plugin.
3. Clone and configure:
   ```bash
   git clone <repo> && cd <repo>/simulation
   cp .env.example .env
   $EDITOR .env            # set DOMAIN, ACME_EMAIL, APP_USERNAME, APP_PASSWORD, INGEST_TOKEN
   ```
4. Bring it up:
   ```bash
   docker compose up -d --build
   ```
   Traefik fetches a certificate automatically. The site is then live at
   `https://${DOMAIN}`; log in with `APP_USERNAME`/`APP_PASSWORD` to start jobs.

Checkpoints and run logs persist in `./data/` on the host. To update: `git pull`
then `docker compose up -d --build`.

> Note: training runs on the host CPU (no GPU assumed) — fine on a VPS, just slower
> than a GPU machine.

## Local development

Backend (from `backend/`, needs [uv](https://docs.astral.sh/uv/)):
```bash
uv sync --extra dev
echo "APP_PASSWORD=dev" > .env          # enable control locally
uv run uvicorn app.main:app --reload --port 8000
uv run pytest                            # tests
```

Frontend (from `frontend/`, needs pnpm):
```bash
pnpm install
VITE_API_BASE=http://localhost:8000/api pnpm dev    # http://localhost:5173
```
Open the app, log in with the credentials from the backend `.env`, and launch a job.
