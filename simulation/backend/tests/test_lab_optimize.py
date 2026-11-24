"""Bucky Lab best-fit search: params set to "*" are searched for the highest mean score."""
import pytest

from bucky.lab.optimize import Axis, parse_axis, run_optimize, starred
from bucky.lab.params import Param

GRID = {"ball_step_cm": 60.0, "ring_radii_cm": [50.0], "ring_angles": 4}


def test_parse_axis():
    p = Param(20.0, 5.0, 60.0, 1.0)
    a = parse_axis("d", "*", p)
    assert (a.lo, a.hi, a.start) == (5.0, 60.0, 20.0)
    a = parse_axis("d", " *10..30 ", p)
    assert (a.lo, a.hi) == (10.0, 30.0)
    assert parse_axis("b", "*", Param(True)).choices == [False, True]
    assert parse_axis("c", "*", Param("a", options=("a", "b"))).choices == ["a", "b"]
    with pytest.raises(ValueError, match="no min/max"):
        parse_axis("x", "*", Param(1.0))
    with pytest.raises(ValueError, match="expected"):
        parse_axis("d", "*10-30", p)
    with pytest.raises(ValueError, match="can't search"):
        parse_axis("l", "*", Param([1.0]))
    assert starred({"a": "*", "b": 1.0, "c": " *1..2"}) == ["a", "c"]


def test_axis_candidates_and_shrink():
    a = Axis("n", "int", 3, lo=0, hi=10, cur_lo=0, cur_hi=10)
    assert a.candidates(3, 5) == [0, 2, 3, 5, 8, 10]
    a.shrink(5, 5)
    assert (a.cur_lo, a.cur_hi) == (2.5, 7.5)
    # Snapped to the param's step; a bracket never narrows below one step.
    s = Axis("s", "float", 0.5, lo=0.05, hi=1.0, cur_lo=0.05, cur_hi=1.0, step=0.05)
    assert s.candidates(0.5, 4) == [0.05, 0.35, 0.5, 0.7, 1.0]
    s.cur_lo, s.cur_hi = 0.6, 0.62
    assert s.candidates(0.6, 5) == [0.6]
    s.shrink(0.6, 5)
    assert (s.cur_lo, s.cur_hi) == pytest.approx((0.55, 0.65))


def _fake_sweep(score_fn, calls):
    def fake(module, *, params, variants, **_):
        calls.append(len(variants))
        out = []
        for v in variants:
            p = {**params, **v["params"]}
            out.append({"label": v["label"], "params": p, "records": [],
                        "overall": {"n": 1, "score": score_fn(p)}, "worst": []})
        return {"grid": {}, "variants": out}
    return fake


def test_two_param_search_finds_optimum(monkeypatch):
    import bucky.lab.optimize as opt

    calls = []
    # bisector: behind_dist ∈ [5, 60], speed has min/max too; peak at (37, 0.55).
    score = lambda p: -((p["behind_dist"] - 37) / 55) ** 2 - ((p["speed"] - 0.55)) ** 2
    monkeypatch.setattr(opt, "run_sweep", _fake_sweep(score, calls))
    res = run_optimize("bisector", params={"behind_dist": "*", "speed": "*0.1..1"},
                       grid=GRID, rounds=6)
    best = res["optimize"]["best"]
    assert best["behind_dist"] == pytest.approx(37, abs=2)
    assert best["speed"] == pytest.approx(0.55, abs=0.05)
    assert [v["label"] for v in res["variants"]] == ["best", "start"]
    hist = res["optimize"]["history"]
    assert len({tuple(sorted(h["params"].items())) for h in hist}) == len(hist)   # no re-runs
    assert hist[0]["score"] == res["optimize"]["best_score"]


def test_variants_rejected():
    with pytest.raises(ValueError, match="variants"):
        run_optimize("bisector", params={"behind_dist": "*"}, variants=[{"params": {}}])


def test_real_search_beats_or_matches_start():
    res = run_optimize("bisector", params={"behind_dist": "*10..40"}, grid=GRID, workers=1,
                       points=3, rounds=1)
    opt = res["optimize"]
    start = next(h for h in opt["history"] if h["params"] == opt["start"])
    assert opt["best_score"] >= start["score"]
    best = res["variants"][0]
    assert best["params"]["behind_dist"] == opt["best"]["behind_dist"]
    assert len(best["records"]) == res["n_scenarios"]
    # Records point at their (renamed) variant, so a replay picks up its concrete params.
    for v in res["variants"]:
        assert {r["variant"] for r in v["records"] + v["worst"]} == {v["label"]}
