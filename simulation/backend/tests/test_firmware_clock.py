"""The virtual clock: time passes only through the firmware's time hooks or Python."""
from __future__ import annotations

import pytest


def test_advance_and_counters(sim):
    s = sim.sim
    assert s.now_ns() == 0 and s.micros() == 0
    s.advance_ns(2_500_000)
    assert s.micros() == 2500 and s.millis() == 2


def test_wraparound_offset(sim):
    s = sim.sim
    s.set_start_offset_us(2**32 - 1000)
    assert s.micros() == 2**32 - 1000
    s.advance_ns(2_000_000)
    assert s.micros() == 1000          # micros() wraps like the 32-bit counter on the MCU


def test_budget(sim):
    s = sim.sim
    s.set_budget_end_ns(1_000_000)
    s.advance_ns(500_000)
    with pytest.raises(sim.SimBudgetExceeded):
        s.advance_ns(1_000_000)


def test_firmware_poll_costs_time(sim):
    bus = sim.make_sensor_bus()
    t0 = sim.sim.now_ns()
    sim.i2c_dma_probe(bus, 0x1E)          # one byte on the wire at 400 kHz
    assert sim.sim.now_ns() - t0 == pytest.approx(22_500, abs=1)
