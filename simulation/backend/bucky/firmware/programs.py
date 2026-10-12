"""Which firmware programs exist — read from the sources, so listing never needs a build."""
from __future__ import annotations

import re

from bucky.firmware.build import firmware_dir

#: Built-in entries besides the test programs.
BUILTINS = {
    "firmware": "main() exactly as built (runs main.cpp's RUN_TEST, if it sets one)",
    "main_loop": "main() with the RUN_TEST call skipped: the real main loop",
}

_TEST_DEF = re.compile(r"void\s+(test\w*)\s*\(\s*const\s+TestContext\s*&[^)]*\)\s*\{")
_RUN_TEST = re.compile(r"^[ \t]*#[ \t]*define[ \t]+RUN_TEST[ \t]+([A-Za-z_]\w*)[ \t]*(?://.*)?$",
                       re.M)


#: What the lab can grade a program on:
#:   "drive"  — it moves the robot → driving experiments (drive_approach)
#:   "sensor" — it reports what a sensor sees → sensor_check
#:   None     — nothing to grade in the lab (needs a human, or only talks to the bus)
#: Programs not listed are assumed to drive.
PROGRAM_ROLE: dict[str, str | None] = {
    "main_loop": "drive",
    "testDriveForward": "drive",
    "testHoldHeading": "drive",
    "testEncoder": "drive",
    "testIR": "sensor",
    "testIRPositioning": "sensor",
    "testSonar": "sensor",
    "testCompass": "sensor",
    "testLine": "sensor",
    "testI2CScan": None,
    "testCalibrate": None,
    "testCalibrationDump": None,
    "testCompassCalibrate": None,
}


def program_role(name: str) -> str | None:
    """See :data:`PROGRAM_ROLE`. ``"firmware"`` takes the role of the test its RUN_TEST runs."""
    if name == "firmware":
        name = run_test_symbol() or "main_loop"
    return PROGRAM_ROLE.get(name, "drive")


def test_programs() -> list[str]:
    """Every ``void testXxx(const TestContext&)`` defined under robot/src/tests."""
    names: set[str] = set()
    for path in sorted((firmware_dir() / "tests").rglob("*.cpp")):
        names.update(_TEST_DEF.findall(path.read_text(errors="replace")))
    return sorted(names)


def run_test_symbol() -> str:
    """The test main.cpp's ``#define RUN_TEST`` names ('' when none)."""
    main = firmware_dir() / "main.cpp"
    if not main.is_file():
        return ""
    m = _RUN_TEST.search(main.read_text(errors="replace"))
    return m.group(1) if m else ""


def list_programs() -> list[str]:
    return [*BUILTINS, *test_programs()]
