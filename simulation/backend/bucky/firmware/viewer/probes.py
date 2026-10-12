"""Pick values the firmware prints out of its serial output, for the perception panel. Each
probe is a regex over one line; add one for any test program that reports what it perceives."""
from __future__ import annotations

import re
from dataclasses import dataclass, field

_FLOAT = r"(-?\d+(?:\.\d+)?)"
PROBES = {
    # testIRPositioning: "BALL  bearing=12.34 deg  ~range=31.30 cm ..."
    "ball": re.compile(rf"BALL\s+bearing={_FLOAT} deg\s+~range={_FLOAT} cm"),
    "no_ball": re.compile(r"^no ball"),
    # testSonar: "S0: 42.0 cm\tS1: TIMEOUT\t..."
    "sonar": re.compile(r"^S0: "),
    # testHoldHeading: "Offset: 12.3 | Rotation: -24.6"
    "hold": re.compile(rf"Offset: {_FLOAT} \| Rotation: {_FLOAT}"),
    # testCompass: "heading=123.4 offset=-5.0 ..."
    "compass": re.compile(rf"heading={_FLOAT} offset={_FLOAT}"),
}


@dataclass
class Perceived:
    ball: tuple[float, float] | None = None        # bearing (deg, CW), range (cm)
    sonar_cm: list[float | None] = field(default_factory=list)
    values: dict[str, str] = field(default_factory=dict)

    def feed(self, line: str) -> None:
        if m := PROBES["ball"].search(line):
            self.ball = (float(m.group(1)), float(m.group(2)))
        elif PROBES["no_ball"].search(line):
            self.ball = None
        elif PROBES["sonar"].search(line):
            self.sonar_cm = []
            for cell in line.split("\t"):
                if ":" not in cell:
                    continue
                v = cell.split(":", 1)[1].strip()
                self.sonar_cm.append(float(v.split()[0]) if v.endswith("cm") else None)
        elif m := PROBES["hold"].search(line):
            self.values["offset"], self.values["rotation"] = m.group(1), m.group(2)
        elif m := PROBES["compass"].search(line):
            self.values["heading"], self.values["offset"] = m.group(1), m.group(2)
