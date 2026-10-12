"""The simulator builds, imports, and knows the firmware's programs."""
from __future__ import annotations

import time

from bucky.firmware import programs
from bucky.firmware.build import ensure_built, module_path


def test_build_is_cached(fw):
    t0 = time.perf_counter()
    path = ensure_built()
    assert path == module_path() and path.exists()
    assert time.perf_counter() - t0 < 5.0   # nothing changed → hash check only


def test_program_table_matches_sources(fw):
    names = fw.sim.programs()
    assert names[:2] == ["firmware", "main_loop"]
    assert names == programs.list_programs()
    for expected in ("testIR", "testLine", "testSonar", "testDriveForward", "testHoldHeading",
                     "testIRPositioning", "testCompass", "testEncoder", "testI2CScan"):
        assert expected in names
    # Declared in tests.h but never defined: must not be offered.
    for missing in ("testIRBallSeek", "testMainLoopSensors", "testStuckDetector",
                    "testStrategyFSM"):
        assert missing not in names


def test_run_test_symbol(fw):
    assert fw.sim.run_test_symbol() == programs.run_test_symbol()
    assert fw.sim.run_test_symbol() in fw.sim.programs()


def test_board_constants(fw):
    from bucky.firmware.pins import pin, pin_name

    b = fw.board
    assert b.NAME == "H562RG"
    assert b.MOTOR1.inA == pin("PB_14_ALT2") and pin_name(b.MOTOR1.inA) == "PB_14_ALT2"
    assert b.KICKER == pin("PA_6_ALT1")
    assert b.HAS_KICKER and b.HAS_GPORTS and b.HAS_BUTTONS
    assert b.G_PORT1_KIND == int(fw.GSensorKind.IR)
    assert b.G_PORT2_KIND == int(fw.GSensorKind.Line)
