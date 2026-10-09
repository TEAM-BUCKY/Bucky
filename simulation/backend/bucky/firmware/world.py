"""Closed loop: the real firmware driving a robot in the training physics.

Each step (``dt``, 50 Hz by default)::

    physics truth → SimHardware.apply_truth (raw readings into the simulated chips)
                  → firmware runs dt of virtual time (it is parked in between)
                  → mean motor PWM over that dt → firmware wheel geometry → body twist
                  → TwoRobotPhysics.step

Usage::

    with FirmwareWorld("testDriveForward", warmup_s=1.0) as w:
        w.place(robot_m=(-0.5, 0.0), heading=0.0, ball_m=(0.5, 0.0))
        w.boot()
        w.run(2.0)
        print(w.serial.text[-500:], w.physics.state_a().robot_pos)

Only one FirmwareWorld can be live per process (the firmware's globals are process-wide).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from bucky.firmware.build import DEFAULT_BOARD
from bucky.firmware.hardware import Actuators, RobotHardwareConfig, RobotTruth, SimHardware
from bucky.firmware.kinematics import body_twist_from_wheels, twist_to_physics_action
from bucky.firmware.loader import load_firmware
from bucky.physics.python_backend import DT, TwoRobotPhysics

_PARKED_B = np.zeros(4)


class FirmwareError(RuntimeError):
    """The firmware program raised (a C++ exception) or the simulator rejected an operation."""


class SerialTail:
    """Accumulates the firmware's USB output."""

    def __init__(self, fw) -> None:
        self._fw = fw
        self.text = ""
        self._line_pos = 0

    def poll(self) -> str:
        new = self._fw.sim.usb_read().decode("utf-8", errors="replace")
        self.text += new
        return new

    def new_lines(self) -> list[str]:
        """Complete lines printed since the last call."""
        self.poll()
        end = self.text.rfind("\n")
        if end < self._line_pos:
            return []
        chunk = self.text[self._line_pos:end]
        self._line_pos = end + 1
        return [ln.rstrip("\r") for ln in chunk.split("\n")]

    def lines(self) -> list[str]:
        self.poll()
        return [ln.rstrip("\r") for ln in self.text.splitlines()]

    def clear(self) -> None:
        self.poll()
        self.text, self._line_pos = "", 0


@dataclass
class StepInfo:
    info: dict                         # TwoRobotPhysics.step() info
    actuators: Actuators
    twist: tuple[float, float, float]  # commanded (vx_right, vy_front m/s, ω_cw rad/s)
    status: str                        # firmware program status after the step
    readings: dict = field(default_factory=dict)


class FirmwareWorld:
    def __init__(self, program: str = "main_loop", *, board: str = DEFAULT_BOARD,
                 config: RobotHardwareConfig | None = None, seed: int = 0, dt: float = DT,
                 poll_cost_us: float = 1.0, warmup_s: float = 0.0,
                 wall_timeout_s: float = 10.0, physics: TwoRobotPhysics | None = None) -> None:
        self.fw = load_firmware(board)
        self.program = program
        self.cfg = config or RobotHardwareConfig()
        self.rng = np.random.default_rng(seed)
        self.dt = dt
        self.poll_cost_us = poll_cost_us
        self.warmup_s = warmup_s
        self.wall_timeout_s = wall_timeout_s
        self.physics = physics or TwoRobotPhysics(dt)
        self.physics.set_removed("b", True)
        self.fw.sim.stop()
        self.fw.sim.reboot()
        self.hw = SimHardware(self.fw, self.cfg, self.rng)
        self.serial = SerialTail(self.fw)
        self.t = 0.0
        self.last: StepInfo | None = None
        self._prev_vel = np.zeros(2)
        self._accel = np.zeros(2)
        self.place()

    # ── setup ──────────────────────────────────────────────────────────────────────────────
    def place(self, robot_m=(-0.3, 0.0), heading: float = 0.0, ball_m=(0.3, 0.0),
              ball_vel=(0.0, 0.0)) -> None:
        self.physics.place_ball(np.asarray(ball_m, float), np.asarray(ball_vel, float))
        self.physics.place_robot("a", np.asarray(robot_m, float), heading)
        self._prev_vel = np.zeros(2)
        self._accel = np.zeros(2)

    def boot(self, program: str | None = None) -> None:
        """Power-on reset, then start ``program``. With ``warmup_s`` the firmware first runs
        that long with the robot held in place (e.g. through the compass start-up)."""
        if program is not None:
            self.program = program
        sim = self.fw.sim
        sim.stop()
        sim.reboot()
        sim.set_poll_cost_ns(int(self.poll_cost_us * 1000))
        self.serial.clear()
        self.t = 0.0
        self._apply_truth()
        sim.start(self.program)
        if self.warmup_s > 0:
            self._run_firmware(self.warmup_s)
            self.hw.read_actuators()   # discard warm-up PWM

    def close(self) -> None:
        self.fw.sim.stop()

    def __enter__(self) -> FirmwareWorld:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ── stepping ───────────────────────────────────────────────────────────────────────────
    @property
    def firmware(self):
        """main.cpp's globals (``world.firmware.compass().getHeading()``). Between steps, use
        only getters that do not read the clock."""
        return self.fw.globals

    def truth(self) -> RobotTruth:
        s = self.physics.state_a()
        return RobotTruth(pos_m=s.robot_pos, heading=float(s.robot_heading), vel_mps=s.robot_vel,
                          omega=float(s.robot_omega), accel_mps2=self._accel,
                          ball_m=s.ball_pos, ball_visible=True)

    def _apply_truth(self) -> dict:
        return self.hw.apply_truth(self.truth())

    def _run_firmware(self, seconds: float) -> str:
        sim = self.fw.sim
        target = sim.now_ns() + int(round(seconds * 1e9))
        status = sim.run_until_ns(target, self.wall_timeout_s)
        if status == "hung":
            raise self.fw.FirmwareHang(
                f"firmware program {self.program!r} spun for {self.wall_timeout_s} s wall time "
                "without reading the clock (a loop with no millis/micros/delay)")
        if status == "error":
            raise FirmwareError(f"firmware program {self.program!r} failed: {sim.error()}")
        return status

    def step(self) -> StepInfo:
        readings = self._apply_truth()
        status = self._run_firmware(self.dt)
        act = self.hw.read_actuators()
        rim = self.cfg.motors.rim_speeds(act.duty)
        twist = body_twist_from_wheels(rim)
        action = twist_to_physics_action(*twist, kick=act.kick)
        info = self.physics.step(action, _PARKED_B)
        vel = self.physics.state_a().robot_vel
        self._accel = (vel - self._prev_vel) / self.dt
        self._prev_vel = vel.copy()
        self.t += self.dt
        self.serial.poll()
        self.last = StepInfo(info=info, actuators=act, twist=twist, status=status,
                             readings=readings)
        return self.last

    def run(self, seconds: float) -> list[StepInfo]:
        return [self.step() for _ in range(max(0, math.ceil(seconds / self.dt - 1e-9)))]
