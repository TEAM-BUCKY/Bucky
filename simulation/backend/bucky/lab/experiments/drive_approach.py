"""Drive approach experiment: can the drive code get the robot *behind* the ball from anywhere?

Scenarios (all combinations, invalid starts skipped):
* ``rings`` mode: ball on a grid over the whole playfield × robot on rings around the ball
  (``ring_radii_cm`` × ``ring_angles`` angles) × ``headings_deg``.
* ``field`` mode: ball fixed at ``ball_cm`` × robot on a grid over the whole arena × headings.

Approach angle (``angle_deg``) is where the robot starts as seen from the ball, user frame:
0° = behind the ball (own-goal side), 90° = right, 180° = in front (opponent-goal side), 270° =
left.

Success ("behind"): the robot is within ``success_radius_cm`` of the ball, inside a
±``success_cone_deg`` cone on the own-goal side, facing the opponent goal within
``heading_tol_deg``, for ``hold_steps`` consecutive steps.
"""
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from bucky.game.field import (
    ARENA_HALF_X,
    ARENA_HALF_Y,
    BALL_RADIUS,
    COLLISION_DIST,
    HALF_H,
    HALF_W,
    ROBOT_RADIUS,
    robot_fully_out,
    robot_in_goal,
)
from bucky.lab.executor import Executor
from bucky.lab.experiments.base import Experiment, register_experiment, summarize_metrics
from bucky.lab.frames import CM_PER_M, Vec2, user_to_world
from bucky.lab.params import Param
from bucky.physics.python_backend import _resolve_robot_goal

_ROBOT_LIM_X = ARENA_HALF_X - ROBOT_RADIUS
_ROBOT_LIM_Y = ARENA_HALF_Y - ROBOT_RADIUS
_TOUCH_DIST = COLLISION_DIST + 0.002


def _grid(half: float, step: float) -> np.ndarray:
    k = int(math.floor(half / step + 1e-9))
    return np.arange(-k, k + 1) * step


def _valid_robot(pos: np.ndarray, ball: np.ndarray) -> bool:
    if abs(pos[0]) > _ROBOT_LIM_X + 1e-9 or abs(pos[1]) > _ROBOT_LIM_Y + 1e-9:
        return False
    if robot_in_goal(pos) or np.linalg.norm(_resolve_robot_goal(pos) - pos) > 1e-9:
        return False
    return float(np.linalg.norm(pos - ball)) >= COLLISION_DIST + 0.01


def _user_field_to_world(xy_cm) -> np.ndarray:
    """User-frame field coordinates (cm, centre origin, +y = opponent goal) → sim metres."""
    return user_to_world(Vec2(float(xy_cm[0]), float(xy_cm[1])))


@register_experiment
class DriveApproach(Experiment):
    """Sweep start positions; score how reliably and cleanly the robot gets behind the ball."""

    name = "drive_approach"
    module_kind = "drive"
    also_accepts = ("firmware",)   # firmware programs that drive (bucky.firmware.lab)
    params = {
        "mode": Param("rings", options=("rings", "field"), help="What to sweep"),
        "ball_step_cm": Param(20.0, 5.0, 60.0, 5.0, "rings: ball grid spacing"),
        "ring_radii_cm": Param([30.0, 60.0, 100.0], help="rings: robot distances from the ball"),
        "ring_angles": Param(16, 4, 72, 1, "rings: robot start angles around the ball"),
        "ball_cm": Param([0.0, 0.0], help="field: fixed ball position (user frame x, y)"),
        "robot_step_cm": Param(10.0, 5.0, 50.0, 5.0, "field: robot grid spacing"),
        "headings_deg": Param([0.0], help="Robot start headings (0 = facing opponent goal)"),
        "timeout_s": Param(4.0, 0.5, 20.0, 0.5, "Give up after this long"),
        "success_radius_cm": Param(25.0, 13.0, 60.0, 1.0, "Max robot–ball distance for 'behind'"),
        "success_cone_deg": Param(30.0, 5.0, 90.0, 1.0, "Half-angle of the 'behind' cone"),
        "heading_tol_deg": Param(30.0, 5.0, 180.0, 1.0, "Max heading error for 'behind'"),
        "hold_steps": Param(3, 1, 50, 1, "Steps the 'behind' condition must hold"),
    }

    # ── scenarios ────────────────────────────────────────────────────────────
    def scenarios(self) -> list[dict]:
        p = self.p
        headings = [math.radians(h) for h in p.headings_deg] or [0.0]
        out: list[dict] = []

        def add(ball, robot, heading, **tags):
            out.append({"id": len(out), "ball": [float(ball[0]), float(ball[1])],
                        "robot": [float(robot[0]), float(robot[1])], "heading": heading, **tags})

        if p.mode == "rings":
            margin = BALL_RADIUS + 0.01
            step = p.ball_step_cm / CM_PER_M
            for bx in _grid(HALF_W - margin, step):
                for by in _grid(HALF_H - margin, step):
                    ball = np.array([bx, by])
                    for r in p.ring_radii_cm:
                        for i in range(p.ring_angles):
                            ang = 360.0 * i / p.ring_angles
                            th = math.radians(ang)
                            off = Vec2(r * math.sin(th), -r * math.cos(th))
                            robot = ball + user_to_world(off)
                            if not _valid_robot(robot, ball):
                                continue
                            for h in headings:
                                add(ball, robot, h, radius_cm=float(r), angle_deg=ang)
        else:
            ball = _user_field_to_world(p.ball_cm)
            step = p.robot_step_cm / CM_PER_M
            for rx in _grid(_ROBOT_LIM_X, step):
                for ry in _grid(_ROBOT_LIM_Y, step):
                    robot = np.array([rx, ry])
                    if not _valid_robot(robot, ball):
                        continue
                    rel = robot - ball
                    # angle_deg in the user convention (0 = behind, 90 = right, CW on screen)
                    ang = (math.degrees(math.atan2(-rel[1], -rel[0])) + 360.0) % 360.0
                    for h in headings:
                        add(ball, robot, h, radius_cm=float(np.linalg.norm(rel) * CM_PER_M),
                            angle_deg=ang)
        return out

    # ── one episode ──────────────────────────────────────────────────────────
    def run_episode(self, ex: Executor, scenario: dict, record: bool = False) -> dict:
        p = self.p
        ball0 = np.array(scenario["ball"])
        robot0 = np.array(scenario["robot"])
        ex.reset(robot0, scenario["heading"], ball0)

        cos_cone = math.cos(math.radians(p.success_cone_deg))
        head_tol = math.radians(p.heading_tol_deg)
        succ_r = p.success_radius_cm / CM_PER_M
        max_steps = int(round(p.timeout_s / ex.physics.dt))

        success = wrong_touch = touched = ball_out = own_goal = goal = False
        time_s = None
        streak = 0
        streak_start = 0.0
        path = 0.0
        out_steps = wall_steps = 0
        push_wrong = 0.0
        prev = robot0.copy()
        trace: list[dict] = []
        steps = 0

        for steps in range(1, max_steps + 1):
            pre = ex.state()
            t0 = ex.t
            res = ex.step()
            s = ex.state()
            r, b = s.robot_pos, s.ball_pos
            if record:
                trace.append({
                    "t": round(t0, 3),
                    "r": [float(pre.robot_pos[0]), float(pre.robot_pos[1]),
                          float(pre.robot_heading)],
                    "b": [float(pre.ball_pos[0]), float(pre.ball_pos[1])],
                    "m": {k: (pre.robot_pos + user_to_world(v)).tolist()
                          for k, v in res.marks.items()},
                })

            path += float(np.linalg.norm(r - prev))
            prev = r.copy()
            if robot_fully_out(r):
                out_steps += 1
            if abs(r[0]) >= _ROBOT_LIM_X - 1e-6 or abs(r[1]) >= _ROBOT_LIM_Y - 1e-6:
                wall_steps += 1
            push_wrong = max(push_wrong, float(ball0[0] - b[0]))

            rel = r - b
            dist = float(np.linalg.norm(rel))
            if dist <= _TOUCH_DIST:
                touched = True
            behind = (dist <= succ_r and dist > 1e-9 and -rel[0] / dist >= cos_cone
                      and abs(s.robot_heading) <= head_tol)
            if not behind and dist <= _TOUCH_DIST:
                wrong_touch = True
            if behind:
                if streak == 0:
                    streak_start = ex.t
                streak += 1
                if streak >= p.hold_steps:
                    success, time_s = True, streak_start
            else:
                streak = 0

            info = res.info
            ball_out |= bool(info["ball_out"])
            own_goal |= bool(info["goal_b"])
            goal |= bool(info["goal_a"])
            if success or ball_out or own_goal or goal:
                break

        straight = float(np.linalg.norm(prev - robot0))
        metrics = {
            "success": success,
            "time_s": time_s if success else None,
            "wrong_touch": wrong_touch,
            "touched": touched,
            "ball_push_wrong_cm": push_wrong * CM_PER_M,
            "ball_moved_cm": float(np.linalg.norm(ex.state().ball_pos - ball0)) * CM_PER_M,
            "path_len_cm": path * CM_PER_M,
            "path_eff": straight / path if path > 1e-6 else 1.0,
            "out_of_bounds": out_steps > 0,
            "out_steps": out_steps,
            "wall_hit": wall_steps > 0,
            "ball_out": ball_out,
            "own_goal": own_goal,
            "steps": steps,
        }
        metrics["score"] = score(metrics, p.timeout_s)
        out = {"metrics": metrics}
        if record:
            out["trace"] = trace
        return out

    # ── aggregation ──────────────────────────────────────────────────────────
    def aggregate(self, records: list[dict], worst_n: int = 25) -> dict:
        agg = super().aggregate(records, worst_n)
        by_ball: dict[tuple, list] = defaultdict(list)
        by_angle: dict[float, list] = defaultdict(list)
        for rec in records:
            sc = rec["scenario"]
            by_ball[(round(sc["ball"][0], 4), round(sc["ball"][1], 4))].append(rec["metrics"])
            by_angle[round(sc.get("angle_deg", 0.0) / 22.5) * 22.5 % 360].append(rec["metrics"])
        agg["by_ball"] = [{"ball": list(k), **summarize_metrics(v)} for k, v in by_ball.items()]
        agg["by_angle"] = [{"angle_deg": k, **summarize_metrics(v)}
                           for k, v in sorted(by_angle.items())]
        return agg


def score(m: dict, timeout_s: float) -> float:
    """Higher is better. 100 for reaching 'behind', minus time and penalties."""
    s = 100.0 if m["success"] else 0.0
    s -= 10.0 * (m["time_s"] if m["success"] else timeout_s)
    s -= 40.0 * m["wrong_touch"]
    s -= m["ball_push_wrong_cm"]
    s -= 20.0 * m["out_of_bounds"]
    s -= 50.0 * m["own_goal"]
    s -= 30.0 * m["ball_out"]
    return round(s, 3)
