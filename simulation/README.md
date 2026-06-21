# Bucky simulation & training platform

A self-contained stack for running and visualizing training jobs in the browser.
The website is public; control actions (launching/stopping jobs) sit behind a
username/password set in `.env`.

```
simulation/
├── frontend/            SvelteKit SPA (static build, served by nginx)
├── backend/             FastAPI app + the training/simulation code (package: bucky)
├── docker-compose.yml   backend + frontend, behind a shared external Traefik
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
- **Traefik** — NOT part of this stack. A shared Traefik instance (deployed
  separately, on the external `traefik_public` network) terminates HTTPS via the
  `leresolver` resolver, serves the frontend at `https://${DOMAIN}`, and routes
  `/api` to the backend. This stack only declares its routes via labels.

## Deploy on a VPS

1. Point a DNS record for your domain at the server's IP.
2. Install Docker + the Compose plugin, and make sure a shared Traefik is running
   with a `leresolver` cert resolver. Create its network once if it doesn't exist:
   ```bash
   docker network create traefik_public
   ```
3. Clone and configure:
   ```bash
   git clone <repo> && cd <repo>/simulation
   cp .env.example .env
   $EDITOR .env            # set DOMAIN, APP_USERNAME, APP_PASSWORD, INGEST_TOKEN
   ```
4. Bring it up:
   ```bash
   docker compose up -d --build
   ```
   The shared Traefik fetches a certificate automatically. The site is then live at
   `https://${DOMAIN}`; log in to start jobs (see auth below).

### Control authentication

Viewing is always public; launching/stopping jobs requires logging in. Two modes:

- **GitHub-org OAuth (recommended).** Set `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`,
  `GITHUB_ORG`, and `PUBLIC_URL=https://${DOMAIN}` in `.env`. Create an OAuth app at
  <https://github.com/settings/developers> with callback URL
  `https://${DOMAIN}/api/auth/callback`. Any member of `GITHUB_ORG` can then "Sign in
  with GitHub"; the shared password is ignored. Each action is recorded per-user in the
  audit log (`GET /api/activity`).
- **Shared password (fallback).** When OAuth is not configured, the legacy
  `APP_USERNAME`/`APP_PASSWORD` gate control — convenient for local dev.

Checkpoints, run logs, and server state persist in named Docker volumes
(`bucky_checkpoints`, `bucky_runs`, `bucky_state`) — managed by Docker and kept
outside the stack directory, so they survive restarts, rebuilds, and redeploys.
Devices, the job queue, users/sessions, and the audit log live in a SQLite database
(`state/bucky.db`, inside the `bucky_state` volume).
To update: `git pull` then `docker compose up -d --build`. They are only deleted by
an explicit `docker compose down -v`.

Inspect or back up a volume, e.g.:
```bash
docker run --rm -v bucky_checkpoints:/data -w /data alpine tar cz . > checkpoints-backup.tgz
```

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
