"""LIS2MDL compass and LSM303AGR accelerometer: the real drivers against simulated chips."""
from __future__ import annotations

import math

import pytest

from bucky.firmware import sensors


def boot_compass(fw, budget_ms: int = 2000):
    c = fw.Compass()
    c.begin(fw.make_sensor_bus())
    while not c.tick():
        fw.sim.advance_ns(1_000_000)
        assert fw.sim.now_ns() < budget_ms * 1_000_000, "compass never finished booting"
    return c


def test_compass_boot_sequence(sim, hw):
    c = boot_compass(sim)
    assert c.isReady()
    # 20 ms boot + 100 ms reset + 200 ms settle + 10 samples × 20 ms
    assert sim.sim.now_ns() / 1e6 == pytest.approx(520, abs=15)
    regs = sim.devices.compass.regs()
    assert regs[0x60] == 0x8C   # continuous mode, 100 Hz, temperature compensation
    assert sim.devices.compass.samples_latched >= 10


def test_compass_absent_fails_after_retries(sim):
    sim.devices.compass.present = False
    c = boot_compass(sim)
    assert c.isFailed() and not c.isReady()


@pytest.mark.parametrize("deg", [0, 37, 90, 181, 270, 355])
def test_heading_follows_rotation(sim, hw, deg):
    c = boot_compass(sim)
    hw.set_heading(-math.radians(deg))       # sim heading is CCW; firmware heading is CW
    sim.sim.advance_ns(20_000_000)           # let the chip latch a new sample
    assert c.update()
    want = sensors.compass_heading_ref(sim.devices.compass.field)
    assert c.getHeading() == pytest.approx(want, abs=1e-3)
    err = (c.getHeading() - deg + 180) % 360 - 180
    assert abs(err) < 1.5                     # 190 mG field, 1.5 mG/LSB → ~0.5° quantisation


def test_calibration_removes_hard_and_soft_iron(sim):
    from bucky.firmware.hardware import RobotHardwareConfig, SimHardware

    model = sensors.CompassModel(hard_iron_lsb=(40.0, -25.0, 0.0), soft_iron=(1.0, 1.3, 1.0),
                                 noise_lsb=0.0)
    hw = SimHardware(sim, RobotHardwareConfig(compass=model))
    c = boot_compass(sim)
    c.setCalibration(40.0, -25.0, 0.0, 1.0, 1.0 / 1.3, 1.0)
    for deg in (10, 100, 200, 300):
        hw.set_heading(-math.radians(deg))
        sim.sim.advance_ns(20_000_000)
        assert c.update()
        assert abs((c.getHeading() - deg + 180) % 360 - 180) < 1.5


def test_hold_heading_controller_sign(sim, hw):
    c = boot_compass(sim)
    c.update()
    c.reset()
    hw.set_heading(-math.radians(20))       # turned 20° clockwise
    sim.sim.advance_ns(20_000_000)
    c.update()
    assert c.getOffset() == pytest.approx(20, abs=1.5)
    sim.sim.advance_ns(10_000_000)
    assert c.computeRotation(0.0) < 0         # rotate back counter-clockwise


def test_accelerometer(sim, hw):
    a = sim.Accelerometer()
    a.begin(sim.make_sensor_bus())
    assert a.isOk()
    assert sim.devices.accel.regs()[0x20] == 0x57
    hw.set_accel((0.25, -0.5))
    ax, ay, az = a.read()
    assert (ax, ay, az) == pytest.approx((0.25, -0.5, 1.0), abs=0.003)


def test_accelerometer_absent(sim):
    sim.devices.accel.present = False
    a = sim.Accelerometer()
    a.begin(sim.make_sensor_bus())
    assert not a.isOk() and a.read() is None


def test_i2c_scan_program(run_program):
    out = run_program("testI2CScan", 1.0)
    assert "0x19" in out and "0x1E" in out and "2 device(s) found." in out
