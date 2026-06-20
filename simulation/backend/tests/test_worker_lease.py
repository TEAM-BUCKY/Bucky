"""Regression tests for the distributed-worker lease/online tracking.

A guest worker polls ``/worker/lease`` every few seconds even when the queue is
empty. Each poll is a check-in and must keep the device marked *online* — otherwise
the admin panel reports a happily-polling worker as "never online".
"""
import asyncio

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager


@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def test_idle_lease_marks_device_online(manager):
    """Polling an empty queue must register a check-in (last_seen → online)."""
    _record, token = manager.devices.register("test-pc")
    device_id = manager.devices.verify(token)
    assert manager.devices.public(device_id)["online"] is False  # never polled yet

    job = asyncio.run(manager.lease_for_worker(device_id))

    assert job is None  # empty queue, nothing leased
    assert manager.devices.public(device_id)["last_seen"] is not None
    assert manager.devices.public(device_id)["online"] is True
