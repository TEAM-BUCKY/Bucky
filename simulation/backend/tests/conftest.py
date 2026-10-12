"""Shared fixtures for the firmware simulator tests (tests/test_firmware_*.py).

``fw``   the loaded ``bucky_fw`` module (built on first use; tests skip without a toolchain)
``sim``  ``fw`` after a power-on reset, with every device input back at a neutral default
``run_program`` boot a program and run it for a while
``hw``   a noise-free :class:`bucky.firmware.hardware.SimHardware` on that board
"""
from __future__ import annotations

import math

import pytest


@pytest.fixture(scope="session")
def fw():
    from bucky.firmware.build import toolchain_available

    if not toolchain_available():
        pytest.skip("the firmware simulator needs cmake and a C++ compiler")
    from bucky.firmware.loader import load_firmware

    return load_firmware()


@pytest.fixture
def sim(fw):
    s = fw.sim
    s.stop()
    # Reboot first: it drops interrupt handlers that point at firmware objects Python created in
    # the previous test (a Button, a GPort), before any pin change below could fire them.
    s.reboot()
    d = fw.devices
    d.compass.present = True
    d.accel.present = True
    for board in (d.gport1, d.gport2):
        board.present = True
        board.values = [[0] * 16 for _ in range(4)]
        board.led_order = [0, 1, 2, 3]
    d.sonar.echo_us = [math.nan] * 4
    d.encoders.rate = [0.0, 0.0, 0.0]
    for p in (fw.board.BUTTON1, fw.board.BUTTON2):
        s.set_input(p, 0)
    s.set_poll_cost_ns(1000)
    s.set_start_offset_us(0)
    s.set_budget_end_ns(0)
    s.set_usb_connected(True)
    s.eeprom_erase()
    s.reboot()
    s.usb_read()
    yield fw
    s.stop()
    s.set_budget_end_ns(0)
    s.reboot()   # same reason: this test's Python-owned objects are about to be freed


@pytest.fixture
def hw(sim):
    from bucky.firmware.hardware import SimHardware

    return SimHardware(sim, rng=None)


@pytest.fixture
def run_program(sim):
    """``run_program(program, seconds)``: boot ``program`` on the reset board, run it for
    ``seconds`` of virtual time and return its serial output."""

    def run(program: str, seconds: float, timeout: float = 30.0) -> str:
        sim.sim.start(program)
        status = sim.sim.run_until_ns(sim.sim.now_ns() + int(seconds * 1e9), timeout)
        assert status in ("parked", "finished"), (status, sim.sim.error())
        return sim.sim.usb_read().decode()

    return run
