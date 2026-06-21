"""Unit tests for ControlClient's per-side latest()/decay logic (no network needed)."""
import time

from bucky.stream_client import ControlClient


def test_defaults_to_ai_with_zero_action_per_side():
    c = ControlClient("ws://localhost:0/none")
    latest = c.latest()
    for side in ("a", "b"):
        # Unclaimed sides default to AI so the policy plays until a human takes over.
        assert latest[side]["mode"] == "ai"
        assert latest[side]["action"] == [0.0, 0.0, 0.0, 0.0]


def test_default_modes_override():
    # Legacy "manual red" starts side B under human control.
    c = ControlClient("ws://localhost:0/none", default_modes={"a": "ai", "b": "human"})
    latest = c.latest()
    assert latest["b"]["mode"] == "human"
    assert latest["a"]["mode"] == "ai"


def test_returns_fresh_action_per_side():
    c = ControlClient("ws://localhost:0/none")
    with c._lock:
        c._sides["a"].update(action=[1.0, -0.5, 0.25, 1.0], mode="human",
                             last_recv=time.monotonic())
    assert c.latest()["a"]["action"] == [1.0, -0.5, 0.25, 1.0]
    assert c.latest()["a"]["mode"] == "human"
    # The other side is untouched.
    assert c.latest()["b"]["action"] == [0.0, 0.0, 0.0, 0.0]


def test_stale_action_decays_to_zero_but_keeps_mode():
    c = ControlClient("ws://localhost:0/none", stale_after=0.05, ai_fallback_after=100.0)
    with c._lock:
        c._sides["b"].update(action=[1.0, 1.0, 1.0, 1.0], mode="human",
                             last_recv=time.monotonic() - 1.0)
    latest = c.latest()
    assert latest["b"]["action"] == [0.0, 0.0, 0.0, 0.0]   # safety: stop driving
    assert latest["b"]["mode"] == "human"                   # mode preserved (still within AI grace)


def test_reverts_to_ai_after_long_silence():
    # A disconnected player's side should hand back to the policy so the match continues.
    c = ControlClient("ws://localhost:0/none", stale_after=0.05, ai_fallback_after=0.1)
    with c._lock:
        c._sides["a"].update(action=[1.0, 1.0, 1.0, 1.0], mode="human",
                             last_recv=time.monotonic() - 1.0)
    latest = c.latest()
    assert latest["a"]["mode"] == "ai"
    assert latest["a"]["action"] == [0.0, 0.0, 0.0, 0.0]
