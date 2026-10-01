"""Plug-in base classes for lab modules.

A *module* is one piece of robot code under test: a drive formula today; a compass filter, a
line-avoidance layer, a sonar localiser, … later. Each step the executor runs every module in
order with ``step(ctx, cmd)``:

* ``ctx`` (:class:`RobotView`) holds the sensor readings the module asked for (``ctx.ball``,
  ``ctx.compass``, …), the time, and the module's resolved params (``self.p``). A module may also
  publish estimates for later modules (``ctx.heading = ...``) and draw debug points
  (``ctx.mark("A", point)``) that show up in the replay.
* ``cmd`` is the :class:`DriveCommand` built so far (``None`` for the first module); the module
  returns the command to pass on. A drive module creates one; a later safety layer (e.g. a line
  sensor module) could veto or bend it.

New module kinds subclass :class:`LabModule` with their own ``kind`` and pair with an experiment
(see :mod:`bucky.lab.experiments`) that knows how to score them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, ClassVar

from bucky.lab.frames import Vec2
from bucky.lab.params import Configurable, Param


@dataclass
class DriveCommand:
    """What to do this frame, user frame. ``direction`` only sets the heading of travel (any
    length); ``speed`` is 0..1 of max speed; ``rotation`` is normalised ω in [-1, 1] (CCW
    positive), or ``None`` to let the executor hold the robot facing the opponent goal."""

    direction: Vec2
    speed: float
    rotation: float | None = None
    kick: float = 0.0


@dataclass
class RobotView:
    """Per-step context handed to modules."""

    readings: dict[str, Any]
    t: float
    dt: float
    marks: dict[str, Vec2] = field(default_factory=dict)
    heading: float | None = None   # best heading estimate (defaults to the compass reading)

    def __getattr__(self, name: str) -> Any:
        readings = self.__dict__.get("readings", {})
        if name in readings:
            return readings[name]
        raise AttributeError(
            f"no sensor reading {name!r} — add it to the module's `sensors` tuple "
            f"(available: {sorted(readings)})"
        )

    def mark(self, name: str, point: Vec2 | None) -> None:
        """Record a user-frame debug point (e.g. the target A) for the replay viewer."""
        if point is not None and math.isfinite(point.x) and math.isfinite(point.y):
            self.marks[name] = point


class LabModule(Configurable):
    kind: ClassVar[str] = ""
    name: ClassVar[str] = ""
    sensors: ClassVar[tuple[str, ...]] = ()

    def reset(self) -> None:
        """Called at the start of every episode; clear any internal state here."""

    def step(self, ctx: RobotView, cmd: DriveCommand | None) -> DriveCommand | None:
        raise NotImplementedError


class DriveModule(LabModule):
    """Base for drive formulas. Override :meth:`target` to return the point A (user frame, cm,
    robot at the origin) to drive towards this frame. Override :meth:`command` instead for full
    control of direction, speed and rotation."""

    kind = "drive"
    sensors = ("ball", "compass")
    params = {
        "speed": Param(0.5, 0.05, 1.0, 0.05, "Top speed, fraction of max (5.2 m/s)"),
        "slow_radius_cm": Param(
            10.0, 0.0, 50.0, 1.0, "Speed ramps down linearly within this distance of A"),
        "heading_kp": Param(
            0.3, 0.0, 1.0, 0.05, "P-gain holding the robot facing the opponent goal"),
    }

    def target(self, ctx: RobotView) -> Vec2 | None:
        raise NotImplementedError

    def command(self, ctx: RobotView) -> DriveCommand | None:
        a = self.target(ctx)
        if a is None:
            return DriveCommand(Vec2(0.0, 0.0), 0.0)
        ctx.mark("A", a)
        d = a.norm()
        ramp = 1.0 if self.p.slow_radius_cm <= 0 else min(1.0, d / self.p.slow_radius_cm)
        return DriveCommand(a, self.p.speed * ramp)

    def step(self, ctx: RobotView, cmd: DriveCommand | None) -> DriveCommand | None:
        return self.command(ctx)
