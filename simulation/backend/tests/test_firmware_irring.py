"""IRRing (robot/src/hardware/sensors/IRRing.cpp) against the physical IR ball model."""
from __future__ import annotations

import math
import re

import numpy as np
import pytest

from bucky.firmware import glue
from bucky.firmware.ir_ball import IRRingModel, with_ball

MODEL = IRRingModel()


def ideal_codes(bearing_deg: float, centre_cm: float, model: IRRingModel = MODEL) -> np.ndarray:
    b = math.radians(bearing_deg)
    ball = (math.cos(b) * centre_cm / 100, -math.sin(b) * centre_cm / 100)
    return model.front_end.code_of(model.photocurrents((0, 0), 0.0, ball), model.ball.carrier_hz)


def angle_err_deg(a_rad: float, truth_deg: float) -> float:
    return abs((math.degrees(a_rad) - truth_deg + 180) % 360 - 180)


GRID = [(b, d) for b in np.arange(-180.0, 180.0, 3.7) for d in (30.0, 45.0, 70.0, 100.0, 150.0)]


def test_estimate_over_the_ring(fw):
    for b, d in GRID:
        r = fw.IRRing.estimate([int(v) for v in ideal_codes(b, d)])
        assert r.seen and not r.clipped
        # At 150 cm the peak is ~13 codes, so the integer neighbour codes limit the bearing.
        assert angle_err_deg(r.angle, b) < (2.0 if d <= 100 else 3.5), (b, d)
        assert r.distanceCm == pytest.approx(d, rel=0.05), (b, d)


def test_estimate_with_droop_offsets_and_noise(fw):
    rng = np.random.default_rng(3)
    base = MODEL.baseline
    errs_b, errs_r = [], []
    for b, d in GRID[::3]:
        clean = ideal_codes(b, d) - base
        for _ in range(20):
            droop = rng.uniform(0.92, 1.0)               # hold age 0–833 µs
            offs = rng.normal(base, 1.24, 16)            # 3 ± 1 mV
            frame = np.clip(np.round(clean * droop + offs + rng.normal(0, 1.0, 16)), 0, 4095)
            frame[12:] = 0
            r = fw.IRRing.estimate([int(v) for v in frame])
            assert r.seen
            errs_b.append(angle_err_deg(r.angle, b))
            errs_r.append(abs(r.distanceCm / d - 1))
    assert np.percentile(errs_b, 95) < 5.0
    assert np.percentile(errs_r, 95) < 0.10


def test_clipped_ball_is_flagged_and_bounded(fw):
    r = fw.IRRing.estimate([int(v) for v in ideal_codes(0.0, 18.0)])   # 7 cm from sensor 0
    assert r.seen and r.clipped
    assert r.distanceCm <= 27.0
    assert angle_err_deg(r.angle, 0.0) < 1.0


def test_no_ball(fw):
    r = fw.IRRing.estimate([4] * 12 + [0] * 4)
    assert not r.seen


def test_position_is_x_right_y_forward(fw):
    r = fw.IRRing.estimate([int(v) for v in ideal_codes(90.0, 60.0)])
    x, y = r.position
    assert x == pytest.approx(60.0, rel=0.05) and abs(y) < 3.0


def test_numpy_port_matches_the_firmware(fw):
    for b, d in GRID[::7]:
        frame = ideal_codes(b, d)
        r = fw.IRRing.estimate([int(v) for v in frame])
        p = glue.ir_estimate(frame)
        assert p["seen"] == r.seen
        assert p["angle"] == pytest.approx(r.angle, abs=1e-3)
        assert p["distance_cm"] == pytest.approx(r.distanceCm, abs=0.1)


def test_ring_reads_the_board_through_gport(sim, hw):
    hw.set_ir(math.radians(-40.0), 80.0)
    port = sim.GPort()
    assert port.begin(sim.board.G_PORT1, sim.GSensorKind.IR)
    ring = sim.IRRing(port)
    assert ring.ready()
    sim.sim.advance_ns(200_000_000)
    assert ring.update()
    r = ring.ball()
    assert angle_err_deg(r.angle, -40.0) < 3.0
    assert r.distanceCm == pytest.approx(80.0, rel=0.08)
    assert not ring.update()                              # nothing new yet


_BALL = re.compile(r"BALL\s+bearing=(-?[\d.]+) deg\s+~range=([\d.]+) cm")


def test_rolling_led_ball_closed_loop():
    """FirmwareWorld: a 42-LED ball rolls past the parked robot; testIRPositioning tracks it."""
    from bucky.firmware.build import toolchain_available
    if not toolchain_available():
        pytest.skip("needs the firmware simulator")
    from bucky.firmware import frames
    from bucky.firmware.hardware import RobotHardwareConfig
    from bucky.firmware.world import FirmwareWorld

    cfg = RobotHardwareConfig(ir=with_ball(IRRingModel(), mode="leds"))
    with FirmwareWorld("testIRPositioning", config=cfg, seed=5) as w:
        w.place(robot_m=(0.0, -0.1), heading=0.0, ball_m=(-0.45, 0.4), ball_vel=(0.4, 0.0))
        w.boot()
        errs_b, errs_r = [], []
        for _ in range(100):
            w.step()
            t = w.truth()
            if t.ball_m is None or np.hypot(*t.ball_vel_mps) < 0.05:
                continue
            bearing, dist = frames.fw_bearing(t.ball_m, t.pos_m, t.heading)
            for line in w.serial.new_lines():
                m = _BALL.search(line)
                if m:
                    errs_b.append(angle_err_deg(math.radians(float(m[1])), math.degrees(bearing)))
                    errs_r.append(float(m[2]) / dist - 1)
        assert len(errs_b) >= 10
        # Frame and print latency (≤ ~50 ms at 0.4 m/s ≈ 2 cm) plus the rolling-LED ripple.
        assert np.median(errs_b) < 4.0 and max(errs_b) < 8.0
        assert np.std(errs_r) < 0.06 and abs(np.median(errs_r)) < 0.08
