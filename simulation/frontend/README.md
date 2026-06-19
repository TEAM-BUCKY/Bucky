# Bucky frontend

SvelteKit single-page app (runes mode, built with `adapter-static`) that visualizes
training/simulation jobs. It's **publicly viewable** — anyone can watch the live stream
read-only — and control actions (launch/stop a job) require logging in with the backend
credentials.

For deploying the whole stack on a VPS, see [`../README.md`](../README.md). This file
covers running the frontend on its own for development.

## Develop

Needs [pnpm](https://pnpm.io/).

```bash
pnpm install

# Point at a backend running on :8000 (see ../backend/README.md), then start the dev server
VITE_API_BASE=http://localhost:8000/api pnpm dev      # http://localhost:5173
```

Open the app, log in with the credentials from the backend `.env`
(`APP_USERNAME`/`APP_PASSWORD`), and launch a job.

`VITE_API_BASE` is the base URL the app calls. It defaults to `/api` (same-origin, used in
production behind Traefik); set it to your backend's address during local dev.

## Build

```bash
pnpm build       # static output in build/
pnpm preview     # preview the production build locally
```

In production the static `build/` is served by nginx and the build is done inside
`Dockerfile` with `VITE_API_BASE=/api` — see `../docker-compose.yml`.
