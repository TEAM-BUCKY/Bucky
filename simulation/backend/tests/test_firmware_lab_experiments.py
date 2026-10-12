"""The lab's firmware experiments: which program gets which experiment, the sensor checks, the
EKF tracking bench, and the glue they share."""
from __future__ import annotations

import math

import numpy as np
import pytest

from bucky.firmware import glue, sensors
from bucky.firmware.ekf_bench import EKF_NOISE_DEFAULTS
from bucky.firmware.programs import program_role, run_test_symbol
from bucky.lab import registry
from bucky.lab.experiments import get_experiment, list_experiments
from bucky.lab.runner import build_executor, replay, run_sweep

# ── which program is graded how ─────────────────────────────────────────────────────────────

def test_programs_are_registered_by_role():
    registry.discover()
    drive = {m["name"] for m in registry.list_modules("firmware")}
    sensor = {m["name"] for m in registry.list_modules("firmware_sensor")}
    assert {"main_loop", "testDriveForward", "testHoldHeading", "testEncoder"} <= drive
    assert {"testIR", "testIRPositioning", "testSonar", "testCompass", "testLine"} <= sensor
    assert not (drive & sensor)
    for hidden in ("testI2CScan", "testCalibrate", "testCalibrationDump", "testCompassCalibrate"):
        assert hidden not in drive | sensor
    # "firmware" (main.cpp as built) is graded like the test its RUN_TEST runs.
    assert program_role("firmware") == program_role(run_test_symbol() or "main_loop")
    assert {m["name"] for m in registry.list_modules("ekf")} == {"firmware_ekf"}


def test_experiments_match_kinds():
    exps = {e["name"]: e for e in list_experiments()}
    assert get_experiment("drive_approach").accepts("firmware")
    assert not get_experiment("drive_approach").accepts("firmware_sensor")
    assert exps["sensor_check"]["accepts_kinds"] == ["firmware_sensor"]
    assert exps["ekf_tracking"]["accepts_kinds"] == ["ekf"]
    for name in ("sensor_check", "ekf_tracking"):
        keys = [m["key"] for m in exps[name]["metrics"]]
        assert "score" in keys and len(keys) == len(set(keys))
    assert exps["drive_approach"]["metrics"] == []           # UI keeps its built-in metrics


def test_drive_experiment_refuses_sensor_programs():
    with pytest.raises(ValueError):
        run_sweep("testIR", kind="firmware_sensor", experiment="drive_approach")


# ── glue ────────────────────────────────────────────────────────────────────────────────────

def test_sonar_localize_at_any_heading():
    """From a guess some cm off, every answer is exact; an answer is only missing when no
    usable ray reaches one of the axes (all rays on side walls, or near corners)."""
    model = sensors.SonarModel(noise_cm=0.0)
    found = total = 0
    for theta_deg in range(-180, 180, 10):
        for pos_cm in ((0.0, 0.0), (-40.0, 60.0), (55.0, -80.0), (20.0, 30.0)):
            theta = math.radians(theta_deg)
            pos_sim = (pos_cm[1] / 100.0, -pos_cm[0] / 100.0)
            d = sensors.sonar_distances_cm(pos_sim, -theta, model)
            got = glue.sonar_localize(d, [True] * 4, theta, (pos_cm[0] + 8.0, pos_cm[1] - 6.0),
                                      model)
            total += 1
            if got is not None:
                found += 1
                assert got == pytest.approx(pos_cm, abs=0.05), (theta_deg, pos_cm)
    assert found / total > 0.8


def test_sonar_localize_needs_both_axes():
    assert glue.sonar_localize([50.0, math.nan, math.nan, math.nan], [True, False, False, False],
                               0.0, (0.0, 0.0)) is None


def test_ir_bearing_range_matches_the_model():
    m = sensors.IRModel(noise=0.0)
    for bearing in (0.0, 30.0, 47.0, -100.0):
        b, r = glue.ir_bearing_range(sensors.ir_adc(math.radians(bearing), 40.0, m))
        assert math.degrees(b) == pytest.approx(bearing, abs=4.0)
        assert r == pytest.approx(40.0, rel=0.05)
    assert glue.ir_bearing_range(sensors.ir_adc(0.0, 40.0, m, visible=False)) is None


def test_encoder_field_velocity_round_trip():
    from bucky.firmware import kinematics as kin

    enc = sensors.EncoderModel()
    for th in (0.0, 0.7, -2.0):
        v_field = np.array([25.0, -40.0])
        right = np.array([math.cos(th), -math.sin(th)])
        fwd = np.array([math.sin(th), math.cos(th)])
        v_body = np.array([v_field @ right, v_field @ fwd]) / 100.0
        rates = sensors.encoder_rates(kin.wheel_rim_speeds(v_body, 0.0), enc)
        assert glue.encoder_field_velocity(rates, th, enc) == pytest.approx(v_field, abs=1e-6)


# ── sensor_check ────────────────────────────────────────────────────────────────────────────

SMALL = {"robot_step_cm": 60.0, "headings_deg": [0.0, 135.0], "duration_s": 1.5}


@pytest.mark.parametrize("program,checks", [
    ("testIRPositioning", {"detect_rate": (0.9, None), "bearing_err_deg": (None, 5.0),
                           "range_err_pct": (None, 0.15)}),
    ("testIR", {"peak_err_deg": (None, 20.0), "desyncs": (None, 0.0)}),
    ("testSonar", {"sonar_err_cm": (None, 2.0), "missed_rate": (None, 0.0)}),
    ("testCompass", {"offset_err_deg": (None, 3.0), "heading_err_deg": (None, 3.0),
                     "accel_err_g": (None, 0.02)}),
    ("testLine", {"line_accuracy": (0.99, None)}),
])
def test_sensor_programs_report_the_truth(program, checks):
    res = run_sweep(program, kind="firmware_sensor", experiment="sensor_check", grid=SMALL,
                    workers=2)
    o = res["variants"][0]["overall"]
    assert o["n"] == res["n_scenarios"] > 0
    assert o["reports"] >= 1, f"{program} printed nothing to grade"
    for key, (lo, hi) in checks.items():
        assert key in o, (key, o)
        if lo is not None:
            assert o[key] >= lo, (key, o[key])
        if hi is not None:
            assert o[key] <= hi, (key, o[key])
    assert res["variants"][0]["by_heading"]


def test_sensor_check_replay_shows_what_the_program_saw():
    grid = {"robot_step_cm": 60.0, "headings_deg": [0.0]}
    exp = get_experiment("sensor_check")(**grid)
    sc = next(s for s in exp.scenarios()
              if math.dist(s["robot"], s["ball"]) < 0.6)           # ball within IR range
    rep = replay("testIRPositioning", sc, kind="firmware_sensor", experiment="sensor_check",
                 grid=grid)
    assert rep["trace"] and any("seen" in f["m"] for f in rep["trace"])
    seen = next(f["m"]["seen"] for f in reversed(rep["trace"]) if "seen" in f["m"])
    assert math.dist(seen, sc["ball"]) < 0.06                    # reported ball ≈ real ball


# ── ekf_tracking ────────────────────────────────────────────────────────────────────────────

def test_ekf_noise_defaults_match_the_firmware(fw):
    n = fw.EKFNoise()
    for k, v in EKF_NOISE_DEFAULTS.items():
        assert getattr(n, k) == pytest.approx(v), k


def _ekf_run(motion: str, **module_params) -> dict:
    registry.discover()
    exp = get_experiment("ekf_tracking")(motion=motion, robot_step_cm=100.0, duration_s=4.0)
    sc = exp.scenarios()[0]
    ex = build_executor("ekf", "firmware_ekf", module_params, None, seed=0)
    return exp.run_episode(ex, sc, record=True)


@pytest.mark.parametrize("motion", ["still", "line", "circle", "figure8"])
def test_ekf_tracks_while_driving(fw, motion):
    m = _ekf_run(motion)["metrics"]
    assert m["pos_rmse_cm"] < 3.0
    assert m["heading_rmse_deg"] < 3.0
    assert m["speed_rmse_cm_s"] < 5.0
    assert m["sonar_fix_hz"] > 10.0
    if m["ball_rmse_cm"] is not None:
        assert m["ball_rmse_cm"] < 15.0


@pytest.mark.xfail(strict=True, reason=(
    "Same EKF limitation as tests/test_firmware_ekf.py::test_heading_keeps_up_with_a_turning_"
    "robot: no angular-rate state, so a robot that turns continuously is tracked many degrees "
    "behind while the filter claims ~1 deg."))
@pytest.mark.parametrize("motion", ["turning", "spin"])
def test_ekf_tracks_a_turning_robot(fw, motion):
    m = _ekf_run(motion)["metrics"]
    assert m["heading_rmse_deg"] < 5.0 and m["consistency"] < 3.0


def test_ekf_without_sonar_dead_reckons(fw):
    with_sonar = _ekf_run("circle")["metrics"]
    without = _ekf_run("circle", use_sonar=False)["metrics"]
    assert without["sonar_fix_hz"] == 0.0
    assert without["pos_rmse_cm"] > with_sonar["pos_rmse_cm"]


def test_ekf_replay_marks(fw):
    out = _ekf_run("circle")
    frames = out["trace"]
    assert frames and all({"E", "EH"} <= set(f["m"]) for f in frames)
    last = frames[-1]
    assert math.dist(last["m"]["E"], last["r"][:2]) < 0.05       # estimate on top of the robot


def test_ekf_sweep_runs_through_the_runner():
    res = run_sweep("firmware_ekf", kind="ekf", experiment="ekf_tracking",
                    grid={"robot_step_cm": 100.0, "motion": "mixed", "duration_s": 3.0}, workers=2)
    v = res["variants"][0]
    assert res["n_scenarios"] == 6 and {r["motion"] for r in v["by_motion"]} == {
        "still", "line", "circle", "figure8", "turning", "spin"}
    assert all(isinstance(r["metrics"]["note"], str) for r in v["records"])
