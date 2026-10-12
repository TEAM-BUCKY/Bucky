"""``python -m bucky.firmware.viewer [--program NAME]`` — live view of the simulated robot."""
from __future__ import annotations

import argparse

from bucky.firmware.programs import list_programs


def main() -> None:
    ap = argparse.ArgumentParser(description="Live pygame view of the firmware simulator")
    ap.add_argument("--program", default="main_loop", choices=list_programs(),
                    help="firmware program to boot (P cycles through them in the window)")
    ap.add_argument("--robot", type=float, nargs=2, default=(-0.4, 0.0), metavar=("X", "Y"),
                    help="robot start, sim frame metres (+x = attacked goal)")
    ap.add_argument("--heading", type=float, default=0.0, help="robot start heading, degrees CCW")
    ap.add_argument("--ball", type=float, nargs=2, default=(0.3, 0.1), metavar=("X", "Y"))
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    import math

    from bucky.firmware.viewer.app import run  # pygame is optional: import late

    run(args.program, robot=tuple(args.robot), heading=math.radians(args.heading),
        ball=tuple(args.ball), seed=args.seed)


if __name__ == "__main__":
    main()
