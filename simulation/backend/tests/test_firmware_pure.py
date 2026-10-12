"""Firmware logic with no hardware behind it, called directly."""
from __future__ import annotations

import math
import struct

import numpy as np
import pytest

from bucky.firmware import kinematics as kin


def test_translate_position_flips_first_eight_of_every_sweep(fw):
    for sweep in range(4):
        for ch in range(16):
            pos = sweep * 16 + ch
            want = sweep * 16 + (7 - ch if ch < 8 else ch)
            assert fw.translatePosition(pos) == want
            assert fw.translatePosition(fw.translatePosition(pos)) == pos


@pytest.mark.parametrize("a", [-3.0, -1.2, 0.0, 0.4, 2.9])
def test_cordic_fallback_matches_libm(fw, a):
    s, c = fw.cordic_sin_cos(a)
    assert s == pytest.approx(math.sin(a), abs=1e-6) and c == pytest.approx(math.cos(a), abs=1e-6)
    ang, mod = fw.cordic_atan2_mod(math.sin(a) * 3, math.cos(a) * 3)
    assert ang == pytest.approx(a, abs=1e-5) and mod == pytest.approx(3.0, abs=1e-5)
    assert fw.cordic_atan2(0.0, 0.0) == 0.0


@pytest.mark.parametrize("deg", [0, 30, 90, 135, 200, 315])
@pytest.mark.parametrize("rot", [0.0, 12.5])
def test_wheel_speeds_match_python_kinematics(fw, deg, rot):
    a = math.radians(deg)
    got = fw.Motors.wheelSpeeds(math.sin(a), math.cos(a), 60.0, rot)
    assert got == pytest.approx(kin.wheel_speeds(a, 60.0, rot), abs=1e-4)


def test_kinematics_round_trip_and_direction():
    assert kin.M_INV @ kin.M == pytest.approx(np.eye(3))
    # driveDegrees(a): translation along (sin a, cos a), no rotation.
    for deg in (0, 45, 90, 180, 270):
        a = math.radians(deg)
        vx, vy, w = kin.body_twist_from_wheels(kin.wheel_speeds(a, 100.0, 0.0) / 100.0)
        assert (vx, vy) == pytest.approx((math.sin(a), math.cos(a)), abs=1e-9)
        assert w == pytest.approx(0.0, abs=1e-9)
    # The shared rotation term is a pure (clockwise) rotation.
    vx, vy, w = kin.body_twist_from_wheels([1.0, 1.0, 1.0])
    assert (vx, vy) == pytest.approx((0.0, 0.0), abs=1e-12) and w > 0


def test_smooth_ramp(fw):
    f = fw.getSmoothFunction
    # duration = |begin - totalSpeed| * 30000 µs
    assert f(0.0, 50.0, 50.0, 0) == pytest.approx(0.0)
    assert f(0.0, 50.0, 50.0, 750_000) == pytest.approx(25.0)   # half-way: smoothstep(0.5) = 0.5
    assert f(0.0, 50.0, 50.0, 1_600_000) == pytest.approx(50.0)


def test_ekf_basics(fw):
    ekf = fw.EKF()
    ekf.init(10.0, 20.0, 0.0)
    st = ekf.getState()
    assert (st.robotX, st.robotY, st.robotTheta) == pytest.approx((10.0, 20.0, 0.0))
    for _ in range(20):
        ekf.updatePosition(30.0, 20.0)
    assert ekf.getState().robotX == pytest.approx(30.0, abs=1.0)
    ekf.updateHeading(0.5)
    assert 0.0 < ekf.getState().robotTheta <= 0.5
    # Ball straight ahead (bearing 0) at 50 cm, robot facing +y.
    ekf.updateHeading(0.0)
    for _ in range(10):
        ekf.updateHeading(0.0)
        ekf.updateBall(0.0, 50.0)
    st = ekf.getState()
    assert st.ballX == pytest.approx(st.robotX, abs=5.0)
    assert st.ballY - st.robotY == pytest.approx(50.0, abs=8.0)
    cov = ekf.getCovariance()
    assert cov == pytest.approx(cov.T, abs=1e-3)
    assert np.all(np.linalg.eigvalsh((cov + cov.T) / 2) > -1e-3)
    ekf.predict(0.0, 0.0, 0.02)


def test_math_helpers(fw):
    assert fw.wrapSignedDegrees(190.0) == pytest.approx(-170.0)
    assert fw.wrapDegrees(-10.0) == pytest.approx(350.0)
    assert fw.radiansToDegrees(math.pi) == pytest.approx(180.0, abs=1e-4)


# StoredCalibration (tests/calibration/calibration_storage.h), little-endian, ARM layout.
_CAL = struct.Struct("<I3f3f12f12f?3x3f3f?3x")


def test_eeprom_calibration_is_loaded_at_boot(sim, run_program):
    out = run_program("main_loop", 0.8)
    assert "No valid calibration in flash; using defaults." in out

    blob = _CAL.pack(0xCA1B0003, 1100.0, 1150.0, 1200.0, *[1.0] * 3, *[1.0] * 12, *[0.0] * 12,
                     True, 10.0, -20.0, 5.0, 1.0, 1.1, 1.0, True)
    sim.sim.stop()
    sim.sim.eeprom_write(blob)
    sim.sim.reboot()
    out = run_program("main_loop", 0.8)
    assert "Calibration loaded from flash." in out
    assert sim.sim.eeprom_read()[: _CAL.size] == blob   # flash survives reboot
