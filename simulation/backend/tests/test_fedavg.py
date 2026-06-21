"""Federated weight-averaging: numpy averaging, npz round-trip, and the coordinator."""
import asyncio

import numpy as np
import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager
from bucky import fedavg


def test_average_arrays_is_elementwise_mean():
    a = {"w": np.array([0.0, 2.0]), "b": np.array([[1.0, 1.0]])}
    b = {"w": np.array([2.0, 4.0]), "b": np.array([[3.0, 5.0]])}
    avg = fedavg.average_arrays([a, b])
    assert np.allclose(avg["w"], [1.0, 3.0])
    assert np.allclose(avg["b"], [[2.0, 3.0]])


def test_npz_bytes_round_trip():
    arrays = {"x": np.arange(6, dtype=np.float32).reshape(2, 3), "y": np.float32(1.5)}
    out = fedavg.from_npz_bytes(fedavg.to_npz_bytes(arrays))
    assert np.allclose(out["x"], arrays["x"]) and np.allclose(out["y"], arrays["y"])


@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def _weights(scale: float) -> bytes:
    return fedavg.to_npz_bytes({"layer": np.full((2, 2), scale, dtype=np.float32)})


def test_coordinator_averages_once_quorum_reached(manager):
    group, rnd = "demo_dist", 0
    # First of two shards pushes — round not ready yet.
    asyncio.run(manager.dist_push(group, rnd, "shardA", 2, _weights(0.0)))
    assert asyncio.run(manager.dist_pull(group, rnd)) is None

    # Second shard completes the quorum — pull now returns the average.
    asyncio.run(manager.dist_push(group, rnd, "shardB", 2, _weights(4.0)))
    out = asyncio.run(manager.dist_pull(group, rnd))
    assert out is not None
    assert np.allclose(fedavg.from_npz_bytes(out)["layer"], 2.0)


def test_distributed_enqueue_fans_out_into_shards(manager):
    result = asyncio.run(manager.enqueue(
        {"mode": "train", "stage": "APPROACH_STATIC_BALL", "seed": 0,
         "distributed": {"shards": 3, "sync_every": 10000}},
        None,
    ))
    assert result["ok"] and result["shards"] == 3
    group = result["group"]
    items = [q for q in manager._queue if q["config"].get("dist_group") == group]
    assert len(items) == 3
    # Each shard carries the group + a distinct seed, and the group is registered.
    seeds = sorted(q["config"]["seed"] for q in items)
    assert seeds == [0, 1, 2]
    assert all(q["config"]["dist_shards"] == 3 for q in items)
    assert group in manager._dist
