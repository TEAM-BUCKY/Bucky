# SQLite + GitHub-org OAuth for the Bucky trainer

Date: 2026-06-21
Status: Approved → implementing

## Problem

The backend persists all mutable state as flat JSON files in `state/`
(`devices.json`, `queue.json`). This has two problems:

1. **No concurrent-write safety.** `_save()` rewrites the whole file; the local
   worker and guest-device check-ins can clobber each other.
2. **Auth is a single shared username/password** (`APP_USERNAME`/`APP_PASSWORD`)
   sent as HTTP Basic on every control request. No per-user identity, no audit.

We want: SQLite-backed durable state with real concurrency, GitHub-org OAuth so any
member of the Bucky org can log in (replacing password auth when OAuth is enabled),
and richer data (audit/history of who did what).

## Decisions (from brainstorming)

- **Session model:** DB-backed session cookie (httpOnly + Secure + SameSite=Lax).
  Revocable, supports real logout.
- **Authorization:** all verified org members get full control. Identity is recorded
  per action for audit. No roles yet (YAGNI).
- **DB scope:** state (devices, queue) + users + sessions + a **jobs audit/history**
  table. Disk stays the source of truth for `checkpoints/*.zip` and `runs/` tensorboard.
- **Tooling:** stdlib `sqlite3` (WAL mode), no ORM. Migrations via `PRAGMA user_version`.
- **OAuth gating:** OAuth is "on" iff `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`,
  `GITHUB_ORG` are all set. When on, password auth is **removed**. When off, Basic
  password auth remains (keeps local dev working without an OAuth app).

## Components

### `app/db.py`
Owns a single SQLite file at `state/bucky.db` (lands in the `bucky_state` volume).
Opened with WAL + `busy_timeout`, thread-safe via a connection lock. A migration
runner keyed on `PRAGMA user_version` creates tables at startup.

Tables:
- `users(id, login, name, avatar_url, created_at, last_login)`
- `sessions(token_hash, user_id, created_at, expires_at)`
- `devices(id, name, token_hash, created_at, last_seen, current_jobs)`
- `queue(id, position, seq, status, target, data)` — full item JSON in `data`,
  ordered by `position`; rewritten atomically on each save.
- `jobs(id, ts, action, run, actor, source, detail)` — audit/history.

### One-time JSON → DB migration
On startup, if `devices.json`/`queue.json` exist and the DB tables are empty, import
them once. The JSON files stay in place as a backup.

### `app/users.py`
DB-backed user + session store: `upsert_user`, `create_session` (returns plaintext
token, stores only its hash), `user_for_session`, `delete_session`, `purge_expired`.

### `app/oauth.py`
GitHub OAuth helpers: `authorize_url(state)`, `exchange_code(code) -> token`,
`fetch_user(token)`, `is_org_member(token, login) -> bool`. Blocking `requests`
calls run in a thread executor so the event loop isn't blocked.

### `app/auth.py` (rewrite)
`require_control` resolves the acting identity:
- **OAuth on:** session cookie → user login. Basic is ignored entirely.
- **OAuth off:** today's Basic password (identity = `"local-password"`).
Returns the actor string, which routes stamp into the `jobs` audit table.
Device-token and ingest-token auth are untouched (machine-to-machine).

### Routes (`app/routes.py`)
New: `GET /api/auth/login` (→ GitHub), `GET /api/auth/callback`, `GET /api/auth/me`,
`POST /api/auth/logout`, `GET /api/activity` (recent audit rows). Existing control
routes thread the actor into manager methods for audit.

CSRF: cookie is SameSite=Lax; control POSTs are same-origin behind Traefik. The OAuth
flow itself is protected by a random `state` carried in a short-lived signed cookie.

### Frontend (`simulation.svelte.ts`, `LoginControl.svelte`)
`/api/auth/me` reports `{ oauth: bool, user }`. When OAuth on: show "Sign in with
GitHub" (→ `/api/auth/login`) + the logged-in user + logout; control requests use
`credentials: 'include'` instead of a Basic header. When off: today's password form.

## Out of scope (YAGNI)
Roles/teams, moving model metadata into the DB, a `model_index` cache table (disk
scan already serves listing — revisit only if listing gets slow), refresh tokens.

## Testing
- DB: migration idempotency, concurrent writes, JSON import.
- Devices: CRUD persists across reopen.
- Auth: OAuth-on ignores Basic; OAuth-off honors Basic; session expiry/revoke.
- OAuth: callback member → session; non-member → 403; bad `state` → rejected (mocked).
- Audit: a launch/stop/delete writes a `jobs` row with the right actor.
