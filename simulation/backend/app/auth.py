"""HTTP Basic auth dependency for control endpoints.

Read-only endpoints (the live stream, run lists, status) stay public; only mutating
actions depend on ``require_control``.

Note: we deliberately do **not** send a ``WWW-Authenticate: Basic`` challenge header
on 401s. That header makes the browser pop its own native credential dialog (even for
``fetch``), which collides with the app's in-UI login. ``HTTPBasic(auto_error=False)``
returns ``None`` instead of raising-with-challenge when credentials are missing, so the
SPA handles every 401 itself.
"""
from __future__ import annotations

import secrets
from typing import Optional

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBasic, HTTPBasicCredentials

from .config import settings

_security = HTTPBasic(auto_error=False)


def require_control(
    credentials: Optional[HTTPBasicCredentials] = Depends(_security),
) -> str:
    """Verify Basic credentials against the configured username/password."""
    if not settings.control_enabled:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Control is disabled: set APP_PASSWORD in the environment.",
        )
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
