"""The real firmware closing the loop through the sensor models and the training physics."""
from __future__ import annotations

import math

import numpy as np
import pytest

from bucky.firmware import kinematics as kin
from bucky.firmware.hardware import RobotHardwareConfig
from bucky.firmware.world import FirmwareWorld

IDEAL = RobotHardwareConfig(motors=kin.MotorModel(polarity=(1.0, 1.0, 1.0)))


@pytest.fixture
def world_factory(sim):
    worlds = []

    def make(program, **kw):
        w = FirmwareWorld(program, **kw)
        worlds.append(w)
        return w

    yield make
    for w in worlds:
        w.close()


def test_drive_forward_with_ideal_motors(world_factory):
    w = world_factory("testDriveForward", config=IDEAL, warmup_s=1.0)
    w.place(robot_m=(-0.6, 0.0), heading=0.0, ball_m=(0.0, 0.5))
    w.boot()
    w.run(2.0)
    s = w.physics.state_a()
    assert s.robot_pos[0] > -0.4                        # forward = +x (towards the opponent goal)
    assert abs(s.robot_pos[1]) < 0.02 and abs(s.robot_heading) < 0.02
    assert "=== Drive Forward Test ===" in w.serial.text


@pytest.mark.xfail(strict=True, reason=(
    "Motors::driveRadians/driveDegrees do not compensate the reversed M1/M3 wiring that "
    "test_calibrate.cpp and test_hold_heading.cpp do: on the board, driveDegrees(0) drives "
    "backwards. Remove this marker once Motors handles motor polarity."))
def test_drive_forward_on_the_board(world_factory):
    w = world_factory("testDriveForward", warmup_s=1.0)
    w.place(robot_m=(0.0, 0.0), heading=0.0, ball_m=(0.0, 0.5))
    w.boot()
    w.run(2.0)
    assert w.physics.state_a().robot_pos[0] > 0.2


def test_single_wheel_gives_the_predicted_twist(world_factory):
    w = world_factory("main_loop", warmup_s=0.7)     # main loop: driveMotorsDirect(100, 0, 0)
    w.place(robot_m=(0.0, 0.0), heading=0.0, ball_m=(0.0, 0.5))
    w.boot()
    st = w.step()
    duty = kin.MotorModel().duty_for_percent(100)
    want = kin.body_twist_from_wheels(kin.MotorModel().rim_speeds([[duty, 0], [0, 0], [0, 0]]))
    assert st.twist == pytest.approx(want, rel=1e-3)
    assert st.actuators.kick == pytest.approx(1.0, abs=1e-3)   # main() also kicks at full power


def test_hold_heading_fights_back(world_factory):
    w = world_factory("testHoldHeading", warmup_s=1.0)
    w.place(robot_m=(0.0, 0.0), heading=0.0, ball_m=(0.0, 0.5))
    w.boot()
    w.run(0.3)
    # Knock the robot 40° counter-clockwise; it should turn back towards where it started.
    s = w.physics.state_a()
    w.physics.place_robot("a", s.robot_pos, math.radians(40))
    w.run(1.5)
    assert abs(math.degrees(w.physics.state_a().robot_heading)) < 15
    assert "Offset:" in w.serial.text


def test_world_reports_sensor_readings(world_factory):
    w = world_factory("main_loop", warmup_s=0.7)
    w.place(robot_m=(0.0, 0.0), heading=0.0, ball_m=(0.4, 0.0))
    w.boot()
    st = w.step()
    ir = np.asarray(st.readings["ir"][:12], float)
    assert int(np.argmax(ir)) == 0                      # ball dead ahead → sensor 0
    assert np.isfinite(st.readings["sonar_cm"]).all()


def test_firmware_lab_sweep_is_deterministic():
    from bucky.lab.runner import replay, run_sweep

    grid = {"ball_step_cm": 60.0, "ring_radii_cm": [60.0], "ring_angles": 4, "timeout_s": 0.5}
    a = run_sweep("testDriveForward", kind="firmware", grid=grid, workers=1)
    b = run_sweep("testDriveForward", kind="firmware", grid=grid, workers=2)
    ra = [r["metrics"] for r in a["variants"][0]["records"]]
    rb = [r["metrics"] for r in b["variants"][0]["records"]]
    assert ra == rb and len(ra) == a["n_scenarios"] > 0
    rec = a["variants"][0]["records"][0]
    rep = replay("testDriveForward", rec["scenario"], kind="firmware", grid=grid)
    assert rep["metrics"] == rec["metrics"] and rep["trace"]


def test_firmware_modules_are_listed():
    from bucky.lab import registry
    from bucky.lab.experiments import list_experiments

    registry.discover()
    names = {m["name"] for m in registry.list_modules("firmware")}
    assert {"main_loop", "testDriveForward", "testHoldHeading"} <= names
    assert any(e["name"] == "drive_approach" for e in list_experiments("firmware"))
