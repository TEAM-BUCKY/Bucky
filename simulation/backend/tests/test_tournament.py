"""Round-robin standings math: points, goal difference, tiebreaks, Elo, and streaming.

The full match (MatchEngine) is slow (~30 s headless), so these tests monkeypatch
``play_match`` with scripted scores and assert the league table the trainer/frontend rely on.
"""
import numpy as np

from bucky.eval import tournament as T


def _fake_scores(script):
    """Return a play_match stand-in driven by a {(side_a_label, side_b_label): (ga, gb)} script.

    run_round_robin passes the actual policy objects; we key the script by object identity via a
    label attached to each fake policy.
    """
    def _play(policy_a, policy_b, *, seed=0, first_kickoff="a", max_steps=0):
        ga, gb = script[(policy_a.label, policy_b.label)]
        return {"a": ga, "b": gb}, ga, gb   # kicks == goals, arbitrary but deterministic
    return _play


class _Fake:
    def __init__(self, label):
        self.label = label

    def predict(self, obs, deterministic=True):
        return np.zeros(4, np.float32), None


def test_league_table_orders_by_points_then_gd(monkeypatch):
    A, B, C = _Fake("A"), _Fake("B"), _Fake("C")
    # A beats everyone 2-0; B beats C 1-0; sides alternate so script both orientations.
    script = {
        ("A", "B"): (2, 0), ("B", "A"): (0, 2),
        ("A", "C"): (2, 0), ("C", "A"): (0, 2),
        ("B", "C"): (1, 0), ("C", "B"): (0, 1),
    }
    monkeypatch.setattr(T, "play_match", _fake_scores(script))
    res = T.run_round_robin([("A", A), ("B", B), ("C", C)], matches_per_pairing=2, seed=0)

    labels = [s.label for s in res.table]
    assert labels == ["A", "B", "C"]           # A: 4 wins, B: 2 wins, C: 0 wins
    top = res.table[0]
    assert top.wins == 4 and top.losses == 0 and top.points == 12
    assert top.goals_for == 8 and top.goals_against == 0 and top.goal_diff == 8
    assert res.table[0].elo > res.table[1].elo > res.table[2].elo
    assert res.complete and len(res.pairings) == 3


def test_all_draws_equal_points_and_elo(monkeypatch):
    A, B = _Fake("A"), _Fake("B")
    script = {("A", "B"): (1, 1), ("B", "A"): (1, 1)}
    monkeypatch.setattr(T, "play_match", _fake_scores(script))
    res = T.run_round_robin([("A", A), ("B", B)], matches_per_pairing=2, seed=0)
    for s in res.table:
        assert s.wins == 0 and s.draws == 2 and s.points == 2
    assert abs(res.table[0].elo - res.table[1].elo) < 1e-9   # equal records → Elo unchanged


def test_on_update_streams_each_match(monkeypatch):
    A, B = _Fake("A"), _Fake("B")
    script = {("A", "B"): (3, 0), ("B", "A"): (0, 3)}
    monkeypatch.setattr(T, "play_match", _fake_scores(script))
    seen = []
    T.run_round_robin([("A", A), ("B", B)], matches_per_pairing=4, seed=0,
                      on_update=lambda live, p: seen.append(p["match"]))
    assert seen == [1, 2, 3, 4]                # one callback per match, in order


def test_duplicate_labels_rejected():
    A, B = _Fake("A"), _Fake("A")
    try:
        T.run_round_robin([("A", A), ("A", B)])
    except ValueError as e:
        assert "unique" in str(e)
    else:
        raise AssertionError("expected ValueError on duplicate labels")
