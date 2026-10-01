"""Sensor models: ground truth → what the robot would actually read.

Modules never see the physics state directly, only sensor readings, so the same code can later
run on the robot. To add a sensor (sonar, line sensor, IR ring, …):

1. subclass :class:`Sensor`, set a unique ``name`` and declare noise/geometry as ``params``;
2. implement :meth:`Sensor.read` (``truth`` holds the full physics state, ``rng`` is seeded per
   episode so sweeps are reproducible);
3. decorate it with :func:`register_sensor`.

A module lists the sensors it needs in ``sensors = ("ball", "compass", ...)`` and reads them as
``ctx.ball``, ``ctx.compass``, …
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, ClassVar

import numpy as np

from bucky.lab.params import Configurable, describe_params
from bucky.physics.backend import PhysicsState


@dataclass
class TruthState:
    """Everything the simulator knows this step. Sensors derive their readings from this."""

    state: PhysicsState          # robot A + ball, sim frame (m, rad)
    t: float                     # seconds since episode start
    other_robot_pos: np.ndarray | None = None   # robot B, when it is on the field


class Sensor(Configurable):
    name: ClassVar[str] = ""

    def reset(self, rng: np.random.Generator) -> None:
        """Called at the start of every episode (e.g. to draw a per-episode compass bias)."""

    def read(self, truth: TruthState, rng: np.random.Generator) -> Any:
        raise NotImplementedError


_SENSORS: dict[str, type[Sensor]] = {}


def register_sensor(cls: type[Sensor]) -> type[Sensor]:
    if not cls.name:
        raise ValueError(f"{cls.__name__} needs a non-empty `name`")
    _SENSORS[cls.name] = cls
    return cls


def get_sensor(name: str) -> type[Sensor]:
    try:
        return _SENSORS[name]
    except KeyError:
        raise KeyError(f"unknown sensor {name!r}; known: {sorted(_SENSORS)}") from None


def list_sensors() -> list[dict]:
    return [
        {"name": n, "doc": (c.__doc__ or "").strip(), "params": describe_params(c.params)}
        for n, c in sorted(_SENSORS.items())
    ]
