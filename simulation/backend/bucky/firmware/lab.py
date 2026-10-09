"""Bucky Lab integration: the firmware's programs as lab modules of kind ``"firmware"``.

Every program (``main_loop``, ``firmware``, ``testDriveForward``, …) is registered as a module,
so the lab's experiments, sweeps, replays and dashboard run the *real* firmware::

    uv run python scripts/lab.py sweep testDriveForward --kind firmware --grid ball_step_cm=60

A firmware module has no Python ``step``: :class:`FirmwareExecutor` replaces the lab's executor,
feeding the physics truth through the sensor models into the simulated board and the board's
motor PWM back into the physics. Each episode is a fresh boot.
"""
from __future__ import annotations

from dataclasses import fields
from typing import ClassVar

from bucky.firmware.build import firmware_dir
from bucky.firmware.hardware import RobotHardwareConfig
from bucky.firmware.programs import BUILTINS, list_programs
from bucky.lab.executor import StepResult
from bucky.lab.modules import LabModule
from bucky.lab.params import Param
from bucky.lab.registry import register


class FirmwareProgram(LabModule):
    """Boot the firmware and run ``program`` (set per registered subclass)."""

    kind = "firmware"
    program: ClassVar[str] = ""
    sensors = ()
    params = {
        "warmup_s": Param(0.7, 0.0, 5.0, 0.1,
                          "Boot time with the robot held still (compass start-up is ~0.52 s)"),
        "poll_cost_us": Param(1.0, 0.1, 20.0, 0.1, "Virtual time each millis()/micros() costs"),
        "compass_noise_lsb": Param(2.0, 0.0, 20.0, 0.5, "Magnetometer noise (σ, LSB)"),
        "sonar_noise_cm": Param(0.5, 0.0, 10.0, 0.5, "Sonar range noise (σ, cm)"),
        "ir_noise": Param(3.0, 0.0, 50.0, 1.0, "IR ADC noise (σ, counts)"),
        "line_noise": Param(6.0, 0.0, 50.0, 1.0, "Line ADC noise (σ, counts)"),
    }

    def step(self, ctx, cmd):   # the firmware runs on the simulated board, not here
        raise NotImplementedError("firmware modules run through FirmwareExecutor")

    def hardware_config(self) -> RobotHardwareConfig:
        base = RobotHardwareConfig()

        def with_(model, **kw):
            names = {f.name for f in fields(model)}
            return type(model)(**{**{n: getattr(model, n) for n in names}, **kw})

        return RobotHardwareConfig(
            compass=with_(base.compass, noise_lsb=self.p.compass_noise_lsb),
            accel=base.accel,
            sonar=with_(base.sonar, noise_cm=self.p.sonar_noise_cm),
            ir=with_(base.ir, noise=self.p.ir_noise),
            line=with_(base.line, noise=self.p.line_noise),
            encoders=base.encoders,
            motors=base.motors,
            line_led_order=base.line_led_order,
        )


def _register_programs() -> None:
    # Deployments that ship only simulation/backend (the Docker image) have no firmware tree.
    if not (firmware_dir() / "main.cpp").is_file():
        return
    for name in list_programs():
        doc = BUILTINS.get(name, f"robot/src/tests: {name}(ctx) after the real boot")
        cls = type(f"Firmware_{name}", (FirmwareProgram,),
                   {"name": name, "program": name, "__doc__": doc,
                    "__module__": __name__})
        register(cls)


_register_programs()


class FirmwareExecutor:
    """Drop-in for :class:`bucky.lab.executor.Executor` that runs a firmware program."""

    def __init__(self, module: FirmwareProgram, seed: int = 0) -> None:
        from bucky.firmware.world import FirmwareWorld

        self.module = module
        self.world = FirmwareWorld(module.program, config=module.hardware_config(), seed=seed,
                                   poll_cost_us=module.p.poll_cost_us,
                                   warmup_s=module.p.warmup_s)
        self.physics = self.world.physics

    @property
    def t(self) -> float:
        return self.world.t

    def reset(self, robot_pos, robot_heading: float, ball_pos, ball_vel=(0.0, 0.0)) -> None:
        self.world.place(robot_pos, robot_heading, ball_pos, ball_vel)
        self.world.boot()

    def state(self):
        return self.physics.state_a()

    def step(self) -> StepResult:
        st = self.world.step()
        return StepResult(info=st.info, cmd=None, marks={})

    def close(self) -> None:
        self.world.close()
