"""Whole programs on the firmware thread: lockstep, stop/restart, reboot, hang detection."""
from __future__ import annotations

import subprocess
import sys
import textwrap

import pytest


def test_main_loop_heartbeat(sim, run_program):
    out = run_program("main_loop", 3.6)
    assert "No valid calibration in flash" in out
    assert out.count("Main loop running...") == 3      # once per virtual second after boot


def test_lockstep_parks_at_the_deadline(sim):
    s = sim.sim
    s.start("main_loop")
    for k in range(1, 6):
        assert s.run_until_ns(k * 100_000_000) == "parked"
        assert s.now_ns() == k * 100_000_000
    s.stop()
    assert s.status() == "stopped"


def test_firmware_getters_between_steps(sim):
    s = sim.sim
    s.start("main_loop")
    s.run_until_ns(1_000_000_000)
    compass = sim.globals.compass()
    assert compass.isReady()
    assert 0.0 <= compass.getHeading() < 360.0
    with pytest.raises(sim.SimMisuse):
        compass.update()                     # reads the clock: only the firmware thread may


def test_restart_and_repeated_reboot(sim):
    s = sim.sim
    for _ in range(10):
        s.stop()
        s.reboot()
        s.start("main_loop")
        assert s.run_until_ns(1_600_000_000) == "parked"
        out = s.usb_read().decode()
        assert out.count("Accel WHO_AM_I: 0x33") == 1
        assert out.count("Main loop running...") == 1


def test_unknown_program(sim):
    with pytest.raises(ValueError):
        sim.sim.start("testNope")


def test_cannot_start_twice(sim):
    sim.sim.start("main_loop")
    with pytest.raises(sim.SimMisuse):
        sim.sim.start("main_loop")


def test_firmware_globals_list_is_complete(fw):
    """Every global the firmware objects define must be reset by firmware_reboot() (or be
    re-initialised by the firmware itself) — see robot/sim/src/firmware/fw_globals.txt."""
    from bucky.firmware.build import build_dir, sim_dir

    archive = build_dir() / "libbucky_firmware_objects.a"
    nm = subprocess.run(["nm", "-C", "--defined-only", str(archive)], capture_output=True,
                        text=True)
    if nm.returncode != 0:
        pytest.skip("nm not available")
    found = set()
    for line in nm.stdout.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[1] in "bBdDuV" and not parts[2].startswith(("guard variable",
                                                                                  "typeinfo",
                                                                                  "vtable")):
            found.add(parts[2])
    listed = {ln.split("#")[0].strip() for ln in
              (sim_dir() / "src/firmware/fw_globals.txt").read_text().splitlines()}
    listed.discard("")
    assert found <= listed, (f"new firmware globals, add them to reboot.cpp + fw_globals.txt: "
                             f"{sorted(found - listed)}")


def test_hang_is_detected_in_a_fresh_process(fw):
    code = textwrap.dedent("""
        from bucky.firmware.loader import load_firmware
        fw = load_firmware()
        fw.sim.start("__spin")
        print(fw.sim.run_until_ns(10_000_000, 1.0), flush=True)
        try:
            fw.sim.reboot()
        except fw.FirmwareHang:
            print("poisoned", flush=True)
        import os
        os._exit(0)   # the spinning thread cannot be joined
    """)
    res = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60)
    assert res.stdout.split() == ["hung", "poisoned"], res.stderr
