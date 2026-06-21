"""Unit tests for ControlClient's latest()/decay logic (no network needed)."""
import time

from bucky.stream_client import ControlClient


def test_defaults_to_human_with_zero_action():
    c = ControlClient("ws://localhost:0/none")
    latest = c.latest()
    assert latest["red_mode"] == "human"
    assert latest["action"] == [0.0, 0.0, 0.0, 0.0]


def test_returns_fresh_action():
    c = ControlClient("ws://localhost:0/none")
    with c._lock:
        c._action = [1.0, -0.5, 0.25, 1.0]
        c._red_mode = "human"
        c._last_recv = time.monotonic()
    assert c.latest()["action"] == [1.0, -0.5, 0.25, 1.0]


def test_stale_action_decays_to_zero_but_keeps_mode():
    c = ControlClient("ws://localhost:0/none", stale_after=0.05)
    with c._lock:
        c._action = [1.0, 1.0, 1.0, 1.0]
        c._red_mode = "ai"
        c._last_recv = time.monotonic() - 1.0   # well past the staleness window
    latest = c.latest()
    assert latest["action"] == [0.0, 0.0, 0.0, 0.0]   # safety: stop driving
    assert latest["red_mode"] == "ai"                  # mode is preserved
