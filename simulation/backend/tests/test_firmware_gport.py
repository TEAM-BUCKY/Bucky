"""G ports: mux clock, ADC trigger, circular DMA and reset sync — the real GPort/LineSensor code
against simulated IR and line boards."""
from __future__ import annotations

import math
import re

import numpy as np
import pytest

from bucky.firmware import sensors


def begin(fw, which: int):
    b = fw.board
    hw_cfg, kind = (b.G_PORT1, b.G_PORT1_KIND) if which == 1 else (b.G_PORT2, b.G_PORT2_KIND)
    port = fw.GPort()
    assert port.begin(hw_cfg, fw.GSensorKind(kind))
    return port


def test_ir_frames_deliver_physical_sensor_order(sim, hw):
    vals = list(range(100, 116))              # physical sensor k reads 100 + k
    hw.set_ir_raw(vals)
    port = begin(sim, 1)
    sim.sim.advance_ns(1_000_000_000)
    assert port.readIR() == vals
    # 600 Hz mux clock, 16 channels per frame → 37.5 frames/s (first reset only syncs).
    assert port.frameSequence() == pytest.approx(37, abs=2)
    assert port.desyncCount() == 0
    assert sim.sim.pad_period_ns(sim.board.G_PORT1.clockPin) == pytest.approx(1e9 / 600, rel=1e-3)


def test_ir_missing_board_gives_no_frames(sim):
    sim.devices.gport1.present = False
    port = begin(sim, 1)
    sim.sim.advance_ns(200_000_000)
    assert port.readIR() is None and port.frameSequence() == 0


def test_glitch_is_counted_as_desync(sim, hw):
    port = begin(sim, 1)
    sim.sim.advance_ns(200_000_000)
    seq = port.frameSequence()
    sim.devices.gport1.skip_step()
    sim.sim.advance_ns(200_000_000)
    assert port.desyncCount() == 1
    assert port.frameSequence() > seq         # and it re-syncs on the next reset


def test_line_frames_and_led_order(sim, hw):
    frame = np.arange(64, dtype=np.uint16).reshape(4, 16) * 10 + 500
    hw.set_line_raw(frame)
    port = begin(sim, 2)
    sim.sim.advance_ns(500_000_000)
    got = port.readLine()
    assert got is not None and np.array_equal(got.value, frame)
    assert port.frameSequence() == pytest.approx(31, abs=2)   # 4000 Hz / 64 per frame

    # A board that lights G, R, B, Dark: wrong colours until the firmware is told.
    sim.devices.gport2.led_order = [1, 0, 2, 3]
    sim.sim.advance_ns(100_000_000)
    assert np.array_equal(port.readLine().value[0], frame[1])
    c = sim.LineColor
    port.setLineOrder([c.Green, c.Red, c.Blue, c.Dark])
    sim.sim.advance_ns(100_000_000)
    assert np.array_equal(port.readLine().value, frame)


def test_line_sensor_white_balance(sim, hw):
    model = sensors.LineModel(noise=0.0)
    port = begin(sim, 2)
    line = sim.LineSensor(port)
    white = np.array([[model.dark + v for v in model.white] + [model.dark]] * 16).T
    hw.set_line_raw(white)
    assert line.calibrateWhite(8)
    assert all(line.hasSensor(s) for s in range(16))
    hw.set_line_raw(sensors.line_adc((0.0, 0.0), 0.0, model))   # centre spot: green
    sim.sim.advance_ns(50_000_000)
    assert line.update()
    r, g, b = line.color(3)
    expect = [round(model.green[c] * 255 / model.white[c]) for c in range(3)]
    assert (r, g, b) == pytest.approx(expect, abs=1)


def test_line_sees_the_boundary(sim, hw):
    port = begin(sim, 2)
    line = sim.LineSensor(port)
    hw.set_line_raw(sensors.line_adc((0.825, 0.0), 0.0, sensors.LineModel(noise=0.0)))
    sim.sim.advance_ns(50_000_000)
    assert line.update()
    refl = [line.reflected(sim.LineColor.Green, s) for s in range(16)]
    assert refl[0] > 1000 and refl[8] < 1000    # front sensor on the white line, back on green


def test_show_mode_steps_the_mux_by_hand(sim):
    port = begin(sim, 2)
    line = sim.LineSensor(port)
    sim.sim.advance_ns(100_000_000)
    resets = port.resetCount()
    assert line.showColor(sim.LineRGB(255, 0, 0), 50)
    assert port.resetCount() > resets
    sim.sim.advance_ns(100_000_000)
    assert port.frameSequence() > 0 and line.update()      # frames resume after the show


def _parse_ball(out: str):
    m = re.findall(r"BALL\s+bearing=(-?[\d.]+) deg\s+~range=([\d.]+) cm", out)
    assert m, out[-500:]
    return float(m[-1][0]), float(m[-1][1])


@pytest.mark.parametrize("bearing_deg", [0.0, 45.0, 100.0, -60.0])
def test_ir_positioning_program(sim, hw, run_program, bearing_deg):
    model = sensors.IRModel(noise=0.0)
    hw.cfg = type(hw.cfg)(ir=model)
    hw.set_ir(math.radians(bearing_deg), 40.0)
    bearing, rng = _parse_ball(run_program("testIRPositioning", 1.5))
    assert (bearing - bearing_deg + 180) % 360 - 180 == pytest.approx(0.0, abs=4.0)
    assert rng == pytest.approx(40.0, rel=0.1)


def test_ir_program_prints_frames(hw, run_program):
    hw.set_ir_raw([100 + k for k in range(16)])
    out = run_program("testIR", 2.0)
    assert "desyncs=0" in out
    assert "\t".join(str(100 + k) for k in range(16)) in out
