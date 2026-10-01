"""Bucky Lab: frames, registry discovery, the bisector drive formula, episodes and sweeps."""
import math

import numpy as np
import pytest

from bucky.lab import registry
from bucky.lab.executor import command_to_action
from bucky.lab.experiments import get_experiment
from bucky.lab.frames import Vec2, user_to_world, world_to_body, world_to_user
from bucky.lab.modules import DriveCommand, RobotView
from bucky.lab.runner import build_executor, replay, run_sweep
from bucky.lab.sensors import TruthState, get_sensor
from bucky.physics.backend import PhysicsState


def _bisector(**params):
    registry.discover()
    return registry.get_module("drive", "bisector")(**params)


def _ctx(ball):
    return RobotView(readings={"ball": ball, "compass": 0.0}, t=0.0, dt=0.02)


def test_frames_roundtrip_and_orientation():
    # +x sim (towards the opponent goal) is +y user; +y sim (left) is -x user.
    assert world_to_user([1.0, 0.0]) == Vec2(0.0, 100.0)
    assert world_to_user([0.0, 1.0]) == Vec2(-100.0, 0.0)
    v = Vec2(12.5, -40.0)
    assert world_to_user(user_to_world(v)) == pytest.approx(v)
    # Facing +y sim (heading 90°), a world +x vector is to the robot's right (body -y).
    assert world_to_body([1.0, 0.0], math.pi / 2) == pytest.approx([0.0, -1.0])


def test_command_drives_in_world_direction_for_any_heading():
    for heading in (0.0, 1.0, -2.5):
        a = command_to_action(DriveCommand(Vec2(0.0, 10.0), 1.0, rotation=0.0), heading, 0.3)
        c, s = math.cos(heading), math.sin(heading)
        world = np.array([c * a[0] - s * a[1], s * a[0] + c * a[1]])
        assert world == pytest.approx([1.0, 0.0], abs=1e-9)   # user +y → sim +x


def test_bisector_matches_geogebra():
    a = _bisector().target(_ctx(Vec2(-50.0, -22.4)))
    assert a.x == pytest.approx(-18.62, abs=0.05)
    assert a.y == pytest.approx(-42.73, abs=0.05)


@pytest.mark.parametrize("ball", [Vec2(0.0, -40.0), Vec2(0.0, 1e-9), Vec2(0.0, 0.0)])
def test_bisector_degenerate_cases_are_finite(ball):
    # ball straight ahead of the robot (robot exactly in front of it from the ball's view),
    # ball on top of the robot.
    a = _bisector().target(_ctx(ball))
    assert math.isfinite(a.x) and math.isfinite(a.y)


def test_bisector_settles_at_c_when_behind():
    m = _bisector(behind_dist=20.0)
    a = m.target(_ctx(Vec2(0.0, 20.0)))   # robot exactly at C
    assert a == pytest.approx(Vec2(0.0, 0.0), abs=1e-9)


def test_registry_reload_picks_up_new_file(tmp_path):
    src = (
        "from bucky.lab import DriveModule, register\n"
        "@register\n"
        "class T(DriveModule):\n"
        "    name = 'tmp_{n}'\n"
        "    def target(self, ctx):\n"
        "        return ctx.ball\n"
    )
    (tmp_path / "t.py").write_text(src.format(n=1))
    (tmp_path / "broken.py").write_text("raise RuntimeError('boom')\n")
    registry.discover(tmp_path)
    assert registry.get_module("drive", "tmp_1")
    assert "broken.py" in registry.load_errors()
    (tmp_path / "t.py").write_text(src.format(n=2))
    registry.discover(tmp_path)
    assert registry.get_module("drive", "tmp_2")
    with pytest.raises(KeyError):
        registry.get_module("drive", "tmp_1")
    registry.discover()


def test_unknown_param_rejected():
    with pytest.raises(ValueError):
        _bisector(not_a_param=1)


def _episode(robot_user_cm, ball_user_cm=(0.0, 0.0)):
    exp = get_experiment("drive_approach")()
    sc = {"id": 0, "heading": 0.0,
          "ball": user_to_world(Vec2(*ball_user_cm)).tolist(),
          "robot": user_to_world(Vec2(*robot_user_cm)).tolist()}
    ex = build_executor("drive", "bisector", {}, {}, seed=0)
    return exp.run_episode(ex, sc, record=True)


def test_episode_from_behind_is_quick_and_clean():
    res = _episode((0.0, -40.0))
    m = res["metrics"]
    assert m["success"] and not m["wrong_touch"]
    assert m["time_s"] < 0.5
    assert res["trace"] and "A" in res["trace"][0]["m"]


def test_episode_from_in_front_goes_around_without_touching():
    m = _episode((0.0, 40.0))["metrics"]
    assert m["success"] and not m["wrong_touch"]
    assert m["ball_push_wrong_cm"] == pytest.approx(0.0, abs=0.5)


def test_small_sweep_shape():
    grid = {"ball_step_cm": 60.0, "ring_radii_cm": [50.0], "ring_angles": 4}
    variants = [{"label": "a", "params": {}}, {"label": "b", "params": {"speed": 0.3}}]
    res = run_sweep("bisector", grid=grid, workers=1, variants=variants)
    n = res["n_scenarios"]
    assert n > 0 and len(res["variants"]) == 2
    for v in res["variants"]:
        assert len(v["records"]) == n
        assert v["overall"]["n"] == n
        assert {"success", "score", "wrong_touch", "path_eff"} <= set(v["overall"])
        assert v["by_ball"] and v["by_angle"] and v["worst"]
    # Replaying a scenario reproduces the sweep result exactly.
    rec = res["variants"][0]["records"][0]
    rep = replay("bisector", rec["scenario"], grid=grid)
    assert rep["metrics"] == rec["metrics"]


def test_sensor_stubs_and_noise():
    st = PhysicsState(np.zeros(2), np.zeros(2), 0.0, 0.0, np.array([0.5, 0.0]), np.zeros(2))
    truth = TruthState(state=st, t=0.0)
    rng = np.random.default_rng(0)
    assert get_sensor("ball")().read(truth, rng) == Vec2(0.0, 50.0)
    assert get_sensor("ball")(dropout=1.0).read(truth, rng) is None
    for name in ("sonar", "line"):
        with pytest.raises(NotImplementedError):
            get_sensor(name)().read(truth, rng)
