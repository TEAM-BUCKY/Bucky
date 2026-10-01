"""Run a stack of lab modules inside the real training physics.

Each 50 Hz step: truth → sensors → :class:`RobotView` → modules (in order) → :class:`DriveCommand`
→ normalised body-frame action ``[vx, vy, ω, kick]`` → :class:`TwoRobotPhysics`. Robot B is
parked off-field; the module drives robot A, which attacks +x (= user-frame +y).

The world→body conversion uses the robot's *heading estimate* (``ctx.heading``, defaulting to the
compass reading), exactly like the real robot would — so compass noise or a compass-filter module
genuinely affects how well the drive code performs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from bucky.lab.frames import Vec2, user_to_world, world_to_body
from bucky.lab.modules import DriveCommand, LabModule, RobotView
from bucky.lab.sensors import Sensor, TruthState, get_sensor
from bucky.physics.python_backend import TwoRobotPhysics

_PARKED_B = np.zeros(4)


@dataclass
class StepResult:
    info: dict
    cmd: DriveCommand | None
    marks: dict[str, Vec2] = field(default_factory=dict)


class Executor:
    def __init__(
        self,
        modules: list[LabModule],
        sensor_params: dict[str, dict] | None = None,
        seed: int = 0,
        heading_kp: float | None = None,
    ) -> None:
        self.modules = modules
        self.rng = np.random.default_rng(seed)
        sensor_params = sensor_params or {}
        names: list[str] = []
        for m in modules:
            names += [s for s in m.sensors if s not in names]
        if "compass" not in names:
            names.append("compass")   # always needed for the world→body conversion
        self.sensors: dict[str, Sensor] = {
            n: get_sensor(n)(**sensor_params.get(n, {})) for n in names
        }
        # Heading hold gain: an explicit value wins, else the first module that declares one.
        if heading_kp is None:
            heading_kp = next((m.p.heading_kp for m in modules if hasattr(m.p, "heading_kp")), 0.3)
        self.heading_kp = float(heading_kp)
        self.physics = TwoRobotPhysics()
        self.t = 0.0

    def reset(self, robot_pos, robot_heading: float, ball_pos, ball_vel=(0.0, 0.0)) -> None:
        self.physics.set_removed("b", True)
        self.physics.place_ball(ball_pos, ball_vel)
        self.physics.place_robot("a", robot_pos, robot_heading)
        self.t = 0.0
        for s in self.sensors.values():
            s.reset(self.rng)
        for m in self.modules:
            m.reset()

    def state(self):
        return self.physics.state_a()

    def step(self) -> StepResult:
        truth = TruthState(state=self.physics.state_a(), t=self.t)
        readings = {n: s.read(truth, self.rng) for n, s in self.sensors.items()}
        ctx = RobotView(readings=readings, t=self.t, dt=self.physics.dt,
                        heading=readings["compass"])
        cmd: DriveCommand | None = None
        for m in self.modules:
            cmd = m.step(ctx, cmd)
        action = command_to_action(cmd, ctx.heading, self.heading_kp)
        info = self.physics.step(action, _PARKED_B)
        self.t += self.physics.dt
        return StepResult(info=info, cmd=cmd, marks=ctx.marks)


def command_to_action(cmd: DriveCommand | None, heading: float, heading_kp: float) -> np.ndarray:
    """User-frame command → normalised body-frame physics action."""
    if cmd is None:
        return np.zeros(4)
    if cmd.rotation is None:
        omega = -heading_kp * heading          # hold heading 0 = facing the opponent goal
    else:
        omega = cmd.rotation
    speed = float(np.clip(cmd.speed, 0.0, 1.0))
    d = user_to_world(cmd.direction)
    n = math.hypot(d[0], d[1])
    if n < 1e-9 or speed <= 0.0:
        vx = vy = 0.0
    else:
        vx, vy = world_to_body(d / n * speed, heading)
    return np.array([vx, vy, float(np.clip(omega, -1.0, 1.0)), float(cmd.kick)])
