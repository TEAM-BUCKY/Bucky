#!/usr/bin/env python
"""Bucky Lab CLI: test hand-written modules (drive formulas, …) across every start position.

Modules live in ``bucky/lab/user/*.py`` (see ``bucky/lab/__init__.py`` for a template).

Usage:
    uv run python scripts/lab.py list
    uv run python scripts/lab.py sweep bisector --grid ball_step_cm=10 --param speed=0.4
    uv run python scripts/lab.py sweep bisector --variant fast:speed=0.8 --variant slow:speed=0.3
    uv run python scripts/lab.py replay bisector --ball 0 0 --robot 0 40

The real firmware (robot/src, see bucky/firmware) runs as modules of kind "firmware", one per
program (main_loop, firmware, testDriveForward, …):
    uv run python scripts/lab.py sweep testDriveForward --kind firmware --grid ball_step_cm=60
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bucky.lab import registry  # noqa: E402
from bucky.lab.experiments import get_experiment, list_experiments  # noqa: E402
from bucky.lab.frames import Vec2, user_to_world  # noqa: E402
from bucky.lab.runner import replay, run_sweep  # noqa: E402

_SUMMARY_COLS = [
    ("success", "success", "{:6.1%}"),
    ("time_s", "time s", "{:6.2f}"),
    ("wrong_touch", "wrong", "{:6.1%}"),
    ("ball_push_wrong_cm", "push cm", "{:7.2f}"),
    ("path_eff", "path eff", "{:8.2f}"),
    ("out_of_bounds", "out", "{:6.1%}"),
    ("score", "score", "{:7.1f}"),
]


def _kv(items: list[str]) -> dict:
    out = {}
    for item in items or []:
        k, _, v = item.partition("=")
        if not _:
            raise SystemExit(f"expected KEY=VALUE, got {item!r}")
        out[k.strip()] = v.strip()
    return out


def _variant(spec: str) -> dict:
    label, _, rest = spec.partition(":")
    return {"label": label, "params": _kv([s for s in rest.split(",") if s])}


def _fmt(row: dict) -> str:
    cells = []
    for key, _, fmt in _SUMMARY_COLS:
        v = row.get(key)
        cells.append(fmt.format(v) if v is not None else " " * len(fmt.format(0)))
    return "  ".join(cells)


def cmd_list(_args) -> None:
    registry.discover()
    for m in registry.list_modules():
        summary = m["doc"].splitlines()[0] if m["doc"] else ""
        print(f"[{m['kind']}] {m['name']:<16} {m['file']}  — {summary}")
        for name, p in m["params"].items():
            print(f"      {name:<16} = {p['default']!r:<10} {p['help']}")
    for f, err in registry.load_errors().items():
        print(f"\n!! {f} failed to load:\n{err}")
    print("\nexperiments:")
    for e in list_experiments():
        print(f"  {e['name']} (tests {e['module_kind']} modules)")
        for name, p in e["params"].items():
            print(f"      {name:<18} = {p['default']!r:<20} {p['help']}")


def cmd_sweep(args) -> None:
    def progress(done, total):
        print(f"\r  {done}/{total} episodes", end="", file=sys.stderr, flush=True)

    res = run_sweep(
        args.module, kind=args.kind, experiment=args.experiment, params=_kv(args.param),
        variants=[_variant(v) for v in args.variant] or None, grid=_kv(args.grid),
        workers=args.workers, seed=args.seed, progress=progress,
    )
    print(file=sys.stderr)
    print(f"{res['module']} × {res['experiment']}: {res['n_scenarios']} scenarios, "
          f"{res['elapsed_s']} s")
    header = "  ".join(h.rjust(len(f.format(0))) for _, h, f in _SUMMARY_COLS)
    print(f"{'variant':<12}{header}")
    for v in res["variants"]:
        print(f"{v['label']:<12}{_fmt(v['overall'])}")
    for v in res["variants"]:
        if "by_angle" in v:
            print(f"\n{v['label']} by start angle (0 = behind ball, 180 = in front):")
            for row in v["by_angle"]:
                print(f"  {row['angle_deg']:>6.1f}°     {_fmt(row)}")
        print(f"\n{v['label']} worst {args.worst}:")
        for r in v["worst"][: args.worst]:
            sc = r["scenario"]
            print(f"  #{sc['id']:<5} ball={sc['ball']} robot=[{sc['robot'][0]:.3f}, "
                  f"{sc['robot'][1]:.3f}]  {_fmt(r['metrics'])}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(res, f)
        print(f"\nwrote {args.json}")


def cmd_replay(args) -> None:
    ball = user_to_world(Vec2(*args.ball))
    robot = user_to_world(Vec2(*args.robot))
    sc = {"id": 0, "ball": ball.tolist(), "robot": robot.tolist(), "heading": args.heading}
    get_experiment(args.experiment)   # fail fast on a typo
    res = replay(args.module, sc, kind=args.kind, experiment=args.experiment,
                 params=_kv(args.param), grid=_kv(args.grid), seed=args.seed)
    for fr in res["trace"][:: args.every]:
        marks = "  ".join(f"{k}=({v[0]:+.3f},{v[1]:+.3f})" for k, v in fr["m"].items())
        print(f"t={fr['t']:5.2f}  robot=({fr['r'][0]:+.3f},{fr['r'][1]:+.3f}) "
              f"ball=({fr['b'][0]:+.3f},{fr['b'][1]:+.3f})  {marks}")
    print(json.dumps(res["metrics"], indent=1))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="List modules, their params and the experiments")

    def common(p):
        p.add_argument("module")
        p.add_argument("--kind", default="drive")
        p.add_argument("--experiment", default="drive_approach")
        p.add_argument("--param", action="append", default=[], help="Module param KEY=VALUE")
        p.add_argument("--grid", action="append", default=[], help="Experiment param KEY=VALUE")
        p.add_argument("--seed", type=int, default=0)

    sw = sub.add_parser("sweep", help="Run every scenario and print a summary")
    common(sw)
    sw.add_argument("--variant", action="append", default=[],
                    help="LABEL:KEY=VAL,KEY=VAL — compare param variants side by side")
    sw.add_argument("--workers", type=int, default=None)
    sw.add_argument("--worst", type=int, default=10)
    sw.add_argument("--json", help="Write the full result to this file")

    rp = sub.add_parser("replay", help="Trace one scenario (positions in user-frame field cm)")
    common(rp)
    rp.add_argument("--ball", type=float, nargs=2, required=True, metavar=("X", "Y"))
    rp.add_argument("--robot", type=float, nargs=2, required=True, metavar=("X", "Y"))
    rp.add_argument("--heading", type=float, default=0.0, help="radians")
    rp.add_argument("--every", type=int, default=5, help="Print every Nth step")

    args = ap.parse_args()
    {"list": cmd_list, "sweep": cmd_sweep, "replay": cmd_replay}[args.cmd](args)


if __name__ == "__main__":
    main()
