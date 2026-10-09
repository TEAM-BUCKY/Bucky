"""sensor_check: grade the firmware's sensor programs on what they report, against the truth.

A sensor program (testIR, testIRPositioning, testSonar, testCompass, testLine) prints what its
sensor sees. Each scenario boots the program with the robot somewhere on the field and the ball
somewhere around it, runs it for a while, parses every line it prints and compares the values
with the physical truth at that moment. One checker per program knows what to parse and how to
score it; the experiment is the same for all of them.

Scenarios: robot on a grid over the arena (``robot_step_cm``) × ``headings_deg``; the ball is
placed at a scenario-seeded bearing and distance (``ball_dist_cm``) from the robot, inside the
field. The robot is held still, except for testCompass, which turns it at ``turn_deg_s`` so the
reported offset has something to follow.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from bucky.firmware import frames, sensors
from bucky.firmware.glue import WALL_X_CM, WALL_Y_CM
from bucky.game.field import BALL_RADIUS, HALF_H, HALF_W, ROBOT_RADIUS
from bucky.lab.experiments.base import Experiment, register_experiment, summarize_metrics
from bucky.lab.params import Param

_ROBOT_LIM_X = WALL_Y_CM / 100.0 - ROBOT_RADIUS      # sim frame: x = goal axis
_ROBOT_LIM_Y = WALL_X_CM / 100.0 - ROBOT_RADIUS
_F = r"(-?\d+(?:\.\d+)?)"


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class Tally:
    """Per-episode accumulators; ``metrics()`` turns them into the episode's metrics."""

    values: dict[str, list[float]] = field(default_factory=dict)
    counts: dict[str, int] = field(default_factory=dict)

    def add(self, key: str, v: float) -> None:
        self.values.setdefault(key, []).append(float(v))

    def count(self, key: str, n: int = 1) -> None:
        self.counts[key] = self.counts.get(key, 0) + n

    def mean(self, key: str) -> float | None:
        v = self.values.get(key)
        return float(np.mean(v)) if v else None


class Checker:
    """Parses one program's output and scores it. ``truth`` holds, at the time of the line:
    robot pos (sim m), heading (sim rad), ball (sim m), start heading, the hardware config."""

    program = ""
    turn_deg_s = 0.0

    def feed(self, line: str, truth: dict, tally: Tally) -> dict[str, list[float]] | None:
        """Handle one serial line; may return marks (sim-frame points) for the replay."""
        raise NotImplementedError

    def metrics(self, tally: Tally) -> dict:
        raise NotImplementedError

    #: (metric, threshold, text) — the note names the first metric past its threshold.
    notes: tuple = ()

    def note(self, m: dict) -> str:
        for key, limit, text in self.notes:
            v = m.get(key)
            if v is None:
                continue
            bad = v < limit if key in ("detect_rate", "line_accuracy", "line_recall") else v > limit
            if bad:
                return text.format(v=v)
        return "ok"


def _ball_truth(truth) -> tuple[float, float]:
    """(bearing deg CW in the robot frame, distance cm) of the ball centre."""
    b, d = frames.fw_bearing(truth["ball"], truth["pos"], truth["heading"])
    return math.degrees(b), d


def _point_at(truth, bearing_deg: float, dist_cm: float) -> list[float]:
    th = frames.sim_to_fw_heading(truth["heading"]) + math.radians(bearing_deg)
    p_fw = frames.sim_to_fw_pos(truth["pos"]) + dist_cm * np.array([math.sin(th), math.cos(th)])
    return frames.fw_to_sim_pos(p_fw).tolist()


class IRPositioningChecker(Checker):
    program = "testIRPositioning"
    notes = (("detect_rate", 0.95, "ball missed ({v:.0%} seen)"),
             ("phantom_rate", 0.05, "ball reported out of range"),
             ("bearing_err_deg", 5.0, "bearing off {v:.1f}°"),
             ("range_err_pct", 0.15, "range off {v:.0%}"))
    BALL = re.compile(rf"BALL\s+bearing={_F} deg\s+~range={_F} cm")

    @staticmethod
    def in_range(truth) -> bool:
        """Whether the ball is close enough to clear testIRPositioning's MIN_PEAK (100) on the
        modelled IR ring: (range_k / d)² ≥ 100."""
        _, d = _ball_truth(truth)
        return d <= truth["config"].ir.range_k / 10.0

    def feed(self, line, truth, tally):
        no_ball = line.startswith("no ball")
        m = None if no_ball else self.BALL.search(line)
        if not no_ball and m is None:
            return None
        tally.count("frames")
        visible = self.in_range(truth)
        if visible:
            tally.count("in_range")
            tally.count("seen", int(m is not None))
        elif m is not None:
            tally.count("phantom")
        if m is None:
            return None
        bearing, rng = float(m.group(1)), float(m.group(2))
        tb, td = _ball_truth(truth)
        tally.add("bearing_err_deg", abs(_wrap_deg(bearing - tb)))
        tally.add("range_err_cm", abs(rng - td))
        tally.add("range_err_pct", abs(rng - td) / max(td, 1.0))
        return {"seen": _point_at(truth, bearing, rng)}

    def metrics(self, t):
        frames_n = t.counts.get("frames", 0)
        in_range = t.counts.get("in_range", 0)
        detect = t.counts.get("seen", 0) / in_range if in_range else None
        phantom = t.counts.get("phantom", 0) / frames_n if frames_n else 0.0
        be, rp = t.mean("bearing_err_deg"), t.mean("range_err_pct")
        if in_range:
            score = 100.0 * detect - 2.0 * (be if be is not None else 45.0) \
                - 50.0 * (rp if rp is not None else 1.0)
        else:                       # ball out of range all episode: only "no ball" is right
            score = 100.0 * (1.0 - phantom)
        return {"reports": frames_n, "detect_rate": detect, "phantom_rate": phantom,
                "bearing_err_deg": be, "range_err_cm": t.mean("range_err_cm"),
                "range_err_pct": rp, "score": score}


class IRRawChecker(Checker):
    """testIR prints raw frames: the brightest of the 12 ring sensors should face the ball."""

    program = "testIR"
    notes = (("desyncs", 0.0, "{v:.0f} desyncs"),
             ("peak_err_deg", 15.0, "brightest sensor {v:.0f}° off the ball"))
    SEQ = re.compile(r"seq=(\d+) desyncs=(\d+)")

    def __init__(self) -> None:
        self.expect_frame = False

    def feed(self, line, truth, tally):
        if m := self.SEQ.search(line):
            tally.values["desyncs"] = [float(m.group(2))]
            self.expect_frame = True
            return None
        if not self.expect_frame or "\t" not in line:
            return None
        self.expect_frame = False
        vals = [float(v) for v in line.split("\t") if v.strip()]
        if len(vals) < 12:
            return None
        tally.count("frames")
        tb, _ = _ball_truth(truth)
        peak = int(np.argmax(vals[:12]))
        tally.add("peak_err_deg", abs(_wrap_deg(30.0 * peak - tb)))
        return {"peak": _point_at(truth, 30.0 * peak, 20.0)}

    def metrics(self, t):
        pe = t.mean("peak_err_deg")
        desync = t.mean("desyncs") or 0.0
        score = 100.0 * (t.counts.get("frames", 0) > 0) - 2.0 * (pe if pe is not None else 45.0) \
            - 10.0 * desync
        return {"reports": t.counts.get("frames", 0), "peak_err_deg": pe, "desyncs": desync,
                "score": score}


class SonarChecker(Checker):
    program = "testSonar"
    notes = (("missed_rate", 0.0, "{v:.0%} echoes missed"),
             ("sonar_err_cm", 2.0, "distance off {v:.1f} cm"))
    CELL = re.compile(r"S(\d): (?:(-?[\d.]+) cm|TIMEOUT)")

    def feed(self, line, truth, tally):
        cells = self.CELL.findall(line)
        if len(cells) < 4:
            return None
        model = truth["config"].sonar
        exact = sensors.sonar_distances_cm(truth["pos"], truth["heading"],
                                           sensors.SonarModel(angles_cw_deg=model.angles_cw_deg,
                                                              noise_cm=0.0))
        # Echoes longer than Sonar.cpp's 20 ms timeout cannot be measured.
        max_cm = 20000.0 * model.cm_per_us - model.latency_us * model.cm_per_us
        marks = {}
        for idx, val in cells:
            i = int(idx)
            truth_cm = exact[i]
            expect = math.isfinite(truth_cm) and truth_cm < max_cm
            tally.count("readings")
            if val == "":
                tally.count("timeouts")
                if expect:
                    tally.count("missed")
                continue
            d = float(val)
            if not expect:
                tally.count("phantom")
                continue
            tally.add("sonar_err_cm", abs(d - truth_cm))
            marks[f"S{i}"] = _point_at(truth, model.angles_cw_deg[i], d + model.mount_radius_cm)
        return marks

    def metrics(self, t):
        n = t.counts.get("readings", 0)
        err = t.mean("sonar_err_cm")
        missed = t.counts.get("missed", 0) / n if n else 1.0
        score = 100.0 - 5.0 * (err if err is not None else 20.0) - 100.0 * missed \
            - 100.0 * (t.counts.get("phantom", 0) / n if n else 0.0)
        return {"reports": n // 4, "sonar_err_cm": err, "missed_rate": missed,
                "timeout_rate": t.counts.get("timeouts", 0) / n if n else 0.0, "score": score}


class CompassChecker(Checker):
    """testCompass: absolute heading (firmware convention) and offset since start, while the
    robot turns; plus the accelerometer, which should read 1 g straight down."""

    program = "testCompass"
    turn_deg_s = 45.0
    notes = (("offset_err_deg", 3.0, "offset off {v:.1f}°"),
             ("heading_err_deg", 3.0, "heading off {v:.1f}°"),
             ("accel_err_g", 0.02, "accelerometer off {v:.3f} g"))
    LINE = re.compile(rf"heading={_F} offset={_F}(?:.*?ax={_F} ay={_F} az={_F})?")

    def feed(self, line, truth, tally):
        m = self.LINE.search(line)
        if m is None:
            return None
        tally.count("readings")
        heading, offset = float(m.group(1)), float(m.group(2))
        theta = math.degrees(frames.sim_to_fw_heading(truth["heading"]))
        start = math.degrees(frames.sim_to_fw_heading(truth["start_heading"]))
        north = truth["config"].compass.north_deg + truth["config"].compass.yaw_offset_deg
        tally.add("heading_err_deg", abs(_wrap_deg(heading - (theta + north))))
        tally.add("offset_err_deg", abs(_wrap_deg(offset - (theta - start))))
        if m.group(3) is not None:
            a = np.array([float(m.group(i)) for i in (3, 4, 5)])
            tally.add("accel_err_g", float(np.linalg.norm(a - np.array([0.0, 0.0, 1.0]))))
        th = frames.sim_to_fw_heading(truth["start_heading"]) + math.radians(offset)  # perceived
        tip = frames.sim_to_fw_pos(truth["pos"]) + 25.0 * np.array([math.sin(th), math.cos(th)])
        return {"heading": frames.fw_to_sim_pos(tip).tolist()}

    def metrics(self, t):
        oe, he = t.mean("offset_err_deg"), t.mean("heading_err_deg")
        score = 100.0 * (t.counts.get("readings", 0) > 0) - 5.0 * (oe if oe is not None else 20.0) \
            - 20.0 * (t.mean("accel_err_g") or 0.0)
        return {"reports": t.counts.get("readings", 0), "offset_err_deg": oe,
                "heading_err_deg": he, "accel_err_g": t.mean("accel_err_g"), "score": score}


class LineChecker(Checker):
    """testLine (uncalibrated) prints the raw table every 500 ms. Each sensor is classified
    white/green on its green-minus-dark reflection, halfway between the line model's green and
    white levels, and compared with what is really under it."""

    program = "testLine"
    notes = (("line_accuracy", 0.99, "{v:.0%} sensors right"),
             ("line_recall", 0.99, "line found by {v:.0%}"))
    ROW = re.compile(r"^(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(-?\d+)\t(-?\d+)\t(-?\d+)$")

    def __init__(self) -> None:
        self.rows: dict[int, int] = {}

    def feed(self, line, truth, tally):
        if line.startswith("sensor\t"):
            self.rows = {}
            return None
        m = self.ROW.match(line.strip())
        if m is None:
            return None
        s, g_minus_d = int(m.group(1)), int(m.group(7))
        self.rows[s] = g_minus_d
        if len(self.rows) < 16:
            return None
        model = truth["config"].line
        threshold = (model.green[1] + model.white[1]) / 2.0
        hits = sensors.line_hits(truth["pos"], truth["heading"], model)
        pts = sensors.line_points(truth["pos"], truth["heading"], model)
        marks = {}
        tally.count("frames")
        for k in range(16):
            seen = self.rows[k] > threshold
            tally.count("sensors")
            tally.count("correct", int(seen == hits[k]))
            if hits[k]:
                tally.count("on_line")
                tally.count("line_found", int(seen))
            if seen:
                marks[f"L{k}"] = pts[k].tolist()
        self.rows = {}
        return marks

    def metrics(self, t):
        n = t.counts.get("sensors", 0)
        acc = t.counts.get("correct", 0) / n if n else 0.0
        on = t.counts.get("on_line", 0)
        return {"reports": t.counts.get("frames", 0), "line_accuracy": acc,
                "line_recall": t.counts.get("line_found", 0) / on if on else None,
                "on_line_rate": on / n if n else 0.0, "score": 100.0 * acc}


CHECKERS: dict[str, type[Checker]] = {
    c.program: c for c in (IRPositioningChecker, IRRawChecker, SonarChecker, CompassChecker,
                           LineChecker)
}


def checker_for(program: str) -> Checker:
    from bucky.firmware.programs import run_test_symbol

    name = run_test_symbol() if program == "firmware" else program
    if name not in CHECKERS:
        raise ValueError(f"sensor_check has no checker for {program!r}; "
                         f"known: {sorted(CHECKERS)}")
    return CHECKERS[name]()


def _grid(lim: float, step: float) -> np.ndarray:
    k = int(math.floor(lim / step + 1e-9))
    return np.arange(-k, k + 1) * step


@register_experiment
class SensorCheck(Experiment):
    """Run a firmware sensor program with the robot all over the field and grade what it prints
    against the truth (IR bearing/range, sonar distances, compass heading, line under the ring)."""

    name = "sensor_check"
    module_kind = "firmware_sensor"
    params = {
        "mode": Param("field", options=("field",), help="Robot positions over the arena"),
        "robot_step_cm": Param(30.0, 10.0, 80.0, 5.0, "Robot grid spacing"),
        "headings_deg": Param([0.0, 135.0], help="Robot headings (0 = facing the opponent goal)"),
        "ball_dist_cm": Param([25.0, 90.0], help="Ball distance range from the robot [min, max]"),
        "duration_s": Param(1.5, 0.5, 10.0, 0.5, "How long to run after the boot"),
        "turn_deg_s": Param(45.0, 0.0, 360.0, 5.0, "testCompass: how fast the robot is turned"),
    }
    metric_specs = [
        {"key": "score", "label": "Score", "higher_is_better": True, "unit": "", "summary": True},
        {"key": "reports", "label": "Reports parsed", "higher_is_better": True, "unit": "",
         "summary": True},
        {"key": "detect_rate", "label": "Ball detected", "higher_is_better": True, "unit": "pct",
         "domain": [0, 1], "summary": True},
        {"key": "phantom_rate", "label": "Ball seen out of range", "higher_is_better": False,
         "unit": "pct", "domain": [0, 1]},
        {"key": "bearing_err_deg", "label": "Bearing error", "higher_is_better": False,
         "unit": "deg", "summary": True},
        {"key": "range_err_pct", "label": "Range error", "higher_is_better": False, "unit": "pct",
         "summary": True},
        {"key": "peak_err_deg", "label": "Peak sensor error", "higher_is_better": False,
         "unit": "deg", "summary": True},
        {"key": "sonar_err_cm", "label": "Sonar error", "higher_is_better": False, "unit": "cm",
         "summary": True},
        {"key": "missed_rate", "label": "Sonar missed", "higher_is_better": False, "unit": "pct",
         "domain": [0, 1], "summary": True},
        {"key": "offset_err_deg", "label": "Compass offset error", "higher_is_better": False,
         "unit": "deg", "summary": True},
        {"key": "heading_err_deg", "label": "Compass heading error", "higher_is_better": False,
         "unit": "deg"},
        {"key": "accel_err_g", "label": "Accelerometer error", "higher_is_better": False,
         "unit": "g"},
        {"key": "line_accuracy", "label": "Line sensors right", "higher_is_better": True,
         "unit": "pct", "domain": [0, 1], "summary": True},
        {"key": "line_recall", "label": "Line found", "higher_is_better": True, "unit": "pct",
         "domain": [0, 1]},
    ]

    def scenarios(self) -> list[dict]:
        p = self.p
        step = p.robot_step_cm / 100.0
        dmin, dmax = (list(p.ball_dist_cm) + [25.0, 90.0])[:2]
        out: list[dict] = []
        for rx in _grid(_ROBOT_LIM_X, step):
            for ry in _grid(_ROBOT_LIM_Y, step):
                for h in p.headings_deg or [0.0]:
                    sid = len(out)
                    rng = np.random.default_rng(1000 + sid)
                    robot = np.array([rx, ry])
                    ball = None
                    for _ in range(50):   # a ball spot inside the field, clear of the robot
                        a = rng.uniform(-math.pi, math.pi)
                        d = rng.uniform(dmin, dmax) / 100.0
                        cand = robot + d * np.array([math.cos(a), math.sin(a)])
                        inside = abs(cand[0]) < HALF_W - BALL_RADIUS and \
                            abs(cand[1]) < HALF_H - BALL_RADIUS
                        if inside:
                            ball = cand
                            break
                    if ball is None:
                        ball = np.clip(robot, [-HALF_W + 0.05, -HALF_H + 0.05],
                                       [HALF_W - 0.05, HALF_H - 0.05]) * 0.5
                    out.append({"id": sid, "robot": robot.tolist(), "ball": ball.tolist(),
                                "heading": math.radians(-h)})   # headings_deg is clockwise
        return out

    def run_episode(self, ex, scenario: dict, record: bool = False) -> dict:
        world = ex.world
        checker = checker_for(world.program)
        turn = math.radians(self.p.turn_deg_s) if checker.turn_deg_s else 0.0
        robot0 = np.array(scenario["robot"])
        ball0 = np.array(scenario["ball"])
        h0 = float(scenario["heading"])
        ex.reset(robot0, h0, ball0)
        world.serial.new_lines()   # boot output is not graded
        tally = Tally()
        trace: list[dict] = []
        marks: dict = {}
        steps = int(round(self.p.duration_s / world.dt))
        for k in range(steps):
            h = h0 - turn * k * world.dt          # turning clockwise
            if turn:
                world.physics.place_robot("a", robot0, h)
            world.physics.place_ball(ball0)       # the ball stays put (nothing should move it)
            world.step()
            s = world.physics.state_a()
            truth = {"pos": s.robot_pos, "heading": float(s.robot_heading),
                     "ball": s.ball_pos, "start_heading": h0, "config": world.cfg}
            for line in world.serial.new_lines():
                new = checker.feed(line, truth, tally)
                if new is not None:
                    marks = new
            if record:
                trace.append({"t": round(world.t, 3),
                              "r": [float(s.robot_pos[0]), float(s.robot_pos[1]),
                                    float(s.robot_heading)],
                              "b": [float(s.ball_pos[0]), float(s.ball_pos[1])],
                              "m": dict(marks)})
        metrics = checker.metrics(tally)
        metrics["note"] = checker.note(metrics) if metrics.get("reports") else \
            "program printed nothing to grade"
        metrics["score"] = round(float(metrics["score"]), 3)
        out = {"metrics": metrics}
        if record:
            out["trace"] = trace
        return out

    def aggregate(self, records: list[dict], worst_n: int = 25) -> dict:
        agg = super().aggregate(records, worst_n)
        by_heading: dict[float, list] = {}
        for r in records:
            by_heading.setdefault(round(-math.degrees(r["scenario"]["heading"]), 1) % 360,
                                  []).append(r["metrics"])
        agg["by_heading"] = [{"heading_deg": k, **summarize_metrics(v)}
                             for k, v in sorted(by_heading.items())]
        return agg
