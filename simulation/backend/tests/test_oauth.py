"""Unit tests for the GitHub OAuth client, using a fake HTTP transport."""
import pytest

from app.oauth import GitHubOAuth, OAuthError


class FakeResp:
    def __init__(self, status_code, payload=None):
        self.status_code = status_code
        self._payload = payload or {}

    def json(self):
        return self._payload


class FakeTransport:
    """Records calls and returns queued responses keyed by URL substring."""

    def __init__(self, responses):
        self._responses = responses
        self.calls = []

    def _match(self, url):
        for key, resp in self._responses.items():
            if key in url:
                return resp
        raise AssertionError(f"no fake response for {url}")

    def post(self, url, **kwargs):
        self.calls.append(("POST", url, kwargs))
        return self._match(url)

    def get(self, url, **kwargs):
        self.calls.append(("GET", url, kwargs))
        return self._match(url)


def make(responses):
    return GitHubOAuth("cid", "secret", "bucky", transport=FakeTransport(responses))


def test_authorize_url_carries_state_and_scope():
    oauth = make({})
    url = oauth.authorize_url("https://x/api/auth/callback", "st4te")
    assert url.startswith("https://github.com/login/oauth/authorize?")
    assert "client_id=cid" in url
    assert "scope=read%3Aorg" in url
    assert "state=st4te" in url


def test_exchange_code_returns_token():
    oauth = make({"login/oauth/access_token": FakeResp(200, {"access_token": "tok"})})
    assert oauth.exchange_code("code", "https://x/cb") == "tok"


def test_exchange_code_errors_without_token():
    oauth = make({"access_token": FakeResp(200, {"error_description": "bad code"})})
    with pytest.raises(OAuthError):
        oauth.exchange_code("code", "https://x/cb")


def test_fetch_user_maps_fields():
    profile = {"id": 7, "login": "koen", "name": "K", "avatar_url": "a"}
    oauth = make({"/user": FakeResp(200, profile)})
    assert oauth.fetch_user("tok") == profile


def test_is_org_member_true_when_active():
    oauth = make({"/user/memberships/orgs/bucky": FakeResp(200, {"state": "active"})})
    assert oauth.is_org_member("tok") is True


def test_is_org_member_false_when_pending():
    oauth = make({"/user/memberships/orgs/bucky": FakeResp(200, {"state": "pending"})})
    assert oauth.is_org_member("tok") is False


def test_is_org_member_false_on_404():
    oauth = make({"/user/memberships/orgs/bucky": FakeResp(404)})
    assert oauth.is_org_member("tok") is False
