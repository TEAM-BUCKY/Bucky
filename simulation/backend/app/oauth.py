"""GitHub OAuth + org-membership checks.

Used only when OAuth is configured (see :class:`~app.config.Settings.oauth_enabled`).
The flow:

1. ``authorize_url`` sends the browser to GitHub with ``scope=read:org`` and a random
   ``state`` we later verify, defeating login-CSRF.
2. ``exchange_code`` swaps the returned ``code`` for a user access token.
3. ``fetch_user`` reads the authenticated user's profile.
4. ``is_org_member`` confirms the user is an *active* member of the configured org —
   the actual authorization gate.

All calls are blocking ``requests`` calls; the async routes invoke them via
``asyncio.to_thread`` so the event loop is never blocked. A ``transport`` (anything
with ``requests``'s ``get``/``post``) can be injected for tests.
"""
from __future__ import annotations

import logging
from urllib.parse import urlencode

import requests

log = logging.getLogger(__name__)

_AUTHORIZE = "https://github.com/login/oauth/authorize"
_TOKEN = "https://github.com/login/oauth/access_token"
_API = "https://api.github.com"
_TIMEOUT = 10.0


class OAuthError(Exception):
    """Raised when GitHub returns an error or an unexpected response."""


class GitHubOAuth:
    def __init__(
        self,
        client_id: str,
        client_secret: str,
        org: str,
        transport=requests,
    ) -> None:
        self._client_id = client_id
        self._client_secret = client_secret
        self._org = org
        self._http = transport

    def authorize_url(self, redirect_uri: str, state: str) -> str:
        """The GitHub URL to send the browser to begin login."""
        q = urlencode({
            "client_id": self._client_id,
            "redirect_uri": redirect_uri,
            "scope": "read:org",
            "state": state,
            "allow_signup": "false",
        })
        return f"{_AUTHORIZE}?{q}"

    def exchange_code(self, code: str, redirect_uri: str) -> str:
        """Exchange an authorization code for a user access token."""
        resp = self._http.post(
            _TOKEN,
            data={
                "client_id": self._client_id,
                "client_secret": self._client_secret,
                "code": code,
                "redirect_uri": redirect_uri,
            },
            headers={"Accept": "application/json"},
            timeout=_TIMEOUT,
        )
        if resp.status_code != 200:
            raise OAuthError(f"token exchange failed ({resp.status_code})")
        body = resp.json()
        token = body.get("access_token")
        if not token:
            raise OAuthError(body.get("error_description") or "no access_token returned")
        return token

    def fetch_user(self, token: str) -> dict:
        """Read the authenticated user's profile (id, login, name, avatar_url)."""
        resp = self._http.get(
            f"{_API}/user",
            headers={"Authorization": f"Bearer {token}", "Accept": "application/vnd.github+json"},
            timeout=_TIMEOUT,
        )
        if resp.status_code != 200:
            raise OAuthError(f"could not read user ({resp.status_code})")
        u = resp.json()
        return {
            "id": u["id"],
            "login": u["login"],
            "name": u.get("name"),
            "avatar_url": u.get("avatar_url"),
        }

    def is_org_member(self, token: str) -> bool:
        """True iff the authenticated user is an *active* member of the configured org.

        Uses ``/user/memberships/orgs/{org}``, which checks the caller's own membership
        (so it works with the user token + ``read:org`` scope and reports pending vs
        active). 404 means not a member."""
        resp = self._http.get(
            f"{_API}/user/memberships/orgs/{self._org}",
            headers={"Authorization": f"Bearer {token}", "Accept": "application/vnd.github+json"},
            timeout=_TIMEOUT,
        )
        if resp.status_code == 404:
            return False
        if resp.status_code != 200:
            raise OAuthError(f"membership check failed ({resp.status_code})")
        return resp.json().get("state") == "active"
