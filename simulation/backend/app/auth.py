"""Control-endpoint authentication.

Two mutually-exclusive modes, selected by configuration:

* **OAuth on** (``settings.oauth_enabled``): control requires a valid session cookie,
  issued after a GitHub-org login. The shared password is ignored entirely — exactly
  the "remove password auth when OAuth is enabled" requirement. The acting identity is
  the user's GitHub login.
* **OAuth off**: the legacy HTTP Basic ``APP_USERNAME``/``APP_PASSWORD`` gate, so local
  development works without standing up an OAuth app. The acting identity is the
  username (recorded as such in the audit log).

Read-only endpoints (the live stream, run lists, status) stay public; only mutating
actions depend on :func:`require_control`. The returned string is the *actor* — routes
stamp it into the ``jobs`` audit table.

We deliberately do not send a ``WWW-Authenticate: Basic`` challenge on 401s; that pops
the browser's native credential dialog and collides with the in-UI login.
"""
from __future__ import annotations

import secrets
from typing import Optional

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPBasic, HTTPBasicCredentials

from .config import settings
from .users import UserStore

# Name of the httpOnly session cookie set after a successful GitHub login.
SESSION_COOKIE = "bucky_session"
# Short-lived cookie holding the OAuth ``state`` between /login and /callback.
OAUTH_STATE_COOKIE = "bucky_oauth_state"

_security = HTTPBasic(auto_error=False)


def _user_store(request: Request) -> UserStore:
    store = getattr(request.app.state, "user_store", None)
    if store is None:  # pragma: no cover — wiring guarantees this in app startup
        raise HTTPException(status_code=500, detail="User store not initialised.")
    return store


def current_user(request: Request) -> Optional[dict]:
    """Resolve the session cookie to a user dict, or ``None`` (never raises).

    Used by ``/api/auth/me`` to render login state. Only meaningful when OAuth is on.
    """
    if not settings.oauth_enabled:
        return None
    token = request.cookies.get(SESSION_COOKIE, "")
    return _user_store(request).user_for_session(token)


def require_control(
    request: Request,
    credentials: Optional[HTTPBasicCredentials] = Depends(_security),
) -> str:
    """Authorise a control action; return the acting identity or raise 401/503."""
    if not settings.control_enabled:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=(
                "Control is disabled: configure GitHub OAuth "
                "(GITHUB_CLIENT_ID/SECRET/ORG) or set APP_PASSWORD."
            ),
        )

    if settings.oauth_enabled:
        user = current_user(request)
        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Sign in with GitHub to control training.",
            )
        return user["login"]

    # Password fallback (OAuth not configured) — legacy single shared credential.
    if credentials is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
        )
    user_ok = secrets.compare_digest(credentials.username, settings.username)
    pass_ok = secrets.compare_digest(credentials.password, settings.password)
    if not (user_ok and pass_ok):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
        )
    return credentials.username
