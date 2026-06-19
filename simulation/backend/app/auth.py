"""HTTP Basic auth dependency for control endpoints.

Read-only endpoints (the live stream, run lists, status) stay public; only mutating
actions depend on ``require_control``.
"""
from __future__ import annotations

import secrets

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBasic, HTTPBasicCredentials

from .config import settings

_security = HTTPBasic(auto_error=True)


def require_control(
    credentials: HTTPBasicCredentials = Depends(_security),
) -> str:
    """Verify Basic credentials against the configured username/password."""
    if not settings.control_enabled:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Control is disabled: set APP_PASSWORD in the environment.",
        )
    user_ok = secrets.compare_digest(credentials.username, settings.username)
    pass_ok = secrets.compare_digest(credentials.password, settings.password)
    if not (user_ok and pass_ok):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
            headers={"WWW-Authenticate": "Basic"},
        )
    return credentials.username
