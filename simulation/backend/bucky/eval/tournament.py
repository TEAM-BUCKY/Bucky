"""Headless round-robin tournament: play every pair of policies and roll out a league table.

This is the single source of truth for standings, shared by the CLI (``scripts/tournament.py``)
and the frontend competition bracket (the backend tournament service drives ``run_round_robin``
directly and streams the standings).

Each pairing plays ``matches_per_pairing`` full matches with alternating sides + kickoff (so
home/away is balanced). Results roll up into a football-style table: 3 points a win, 1 a draw,
ranked by points → goal difference → goals for. An Elo rating is tracked as a secondary signal.

Matches run headless with no real-time throttle (the same inline pattern as the old
``eval_h2h.py``) so a full round-robin completes in seconds-to-minutes rather than hours.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Callable, Sequence

# Any object satisfying ``predict(obs, deterministic=True) -> (action, _)``.
Policy = Any
# (label, policy) — the label names the entrant in the standings.
Entrant = tuple[str, Policy]


@dataclass
class Standing:
    label: str
    played: int = 0
    wins: int = 0
    draws: int = 0
    losses: int = 0
    goals_for: int = 0
    goals_against: int = 0
    kicks: int = 0
    elo: float = 1000.0

    @property
    def points(self) -> int:
        return 3 * self.wins + self.draws

    @property
    def goal_diff(self) -> int:
        return self.goals_for - self.goals_against

    def to_dict(self) -> dict:
        d = asdict(self)
        d.update(points=self.points, goal_diff=self.goal_diff,
                 elo=round(self.elo, 1),
                 kicks_per_match=round(self.kicks / self.played, 2) if self.played else 0.0)
        return d


@dataclass
class PairResult:
    """Aggregate of one pairing (``a`` vs ``b``) over all its matches."""
    a: str
    b: str
    goals_a: int = 0
    goals_b: int = 0
    wins_a: int = 0
    wins_b: int = 0
    draws: int = 0
    matches: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


def play_match(policy_a: Policy, policy_b: Policy, *, seed: int = 0,
               first_kickoff: str = "a", max_steps: int = 200_000) -> tuple[dict, int, int]:
    """Play one full headless match; return ``(score{'a','b'}, kicks_a, kicks_b)``.

    ``max_steps`` is a safety cap — the referee ends a match well before it (2×7 min of sim),
    but the cap guarantees termination if a policy ever stalls the clock.
    """
    from bucky.match import MatchEngine

    eng = MatchEngine(policy_a, policy_b, seed=seed, domain_rand=False,
                      match_mode=True, first_kickoff=first_kickoff)
    kicks_a = kicks_b = 0
    frame = None
    for _ in range(max_steps):
        frame = eng.tick()
        kicks_a += int(frame["kicked_a"])
        kicks_b += int(frame["kicked_b"])
        if frame["match_over"]:
            break
    score = frame["score"] if frame else {"a": 0, "b": 0}
    return {"a": int(score["a"]), "b": int(score["b"])}, kicks_a, kicks_b


def _elo_update(ra: float, rb: float, score_a: float, k: float = 32.0) -> tuple[float, float]:
    expected_a = 1.0 / (1.0 + 10.0 ** ((rb - ra) / 400.0))
    ra_new = ra + k * (score_a - expected_a)
    rb_new = rb + k * ((1.0 - score_a) - (1.0 - expected_a))
    return ra_new, rb_new


def _record(sa: Standing, sb: Standing, gf: int, ga: int, kicks_a: int, kicks_b: int) -> None:
    """Fold one match (``gf`` = sa's goals, ``ga`` = sb's goals) into both standings."""
    sa.played += 1
    sb.played += 1
    sa.goals_for += gf
    sa.goals_against += ga
    sb.goals_for += ga
    sb.goals_against += gf
    sa.kicks += kicks_a
    sb.kicks += kicks_b
    if gf > ga:
        sa.wins += 1
        sb.losses += 1
        result_a = 1.0
    elif gf < ga:
        sa.losses += 1
        sb.wins += 1
        result_a = 0.0
    else:
        sa.draws += 1
        sb.draws += 1
        result_a = 0.5
    sa.elo, sb.elo = _elo_update(sa.elo, sb.elo, result_a)


def _sorted_table(standings: dict[str, Standing]) -> list[Standing]:
    return sorted(standings.values(),
                  key=lambda s: (s.points, s.goal_diff, s.goals_for, s.elo), reverse=True)


@dataclass
class Tournament:
    table: list[Standing]
    pairings: list[PairResult]
    complete: bool = True

    def to_dict(self) -> dict:
        return {
            "standings": [s.to_dict() for s in self.table],
            "pairings": [p.to_dict() for p in self.pairings],
            "complete": self.complete,
        }


def run_round_robin(
    entrants: Sequence[Entrant],
    *,
    matches_per_pairing: int = 5,
    seed: int = 0,
    max_steps: int = 200_000,
    on_update: Callable[[Tournament, dict], None] | None = None,
    should_stop: Callable[[], bool] | None = None,
) -> Tournament:
    """Play a full round-robin and return the final :class:`Tournament`.

    ``max_steps`` caps each match's length; the default lets the referee run a full 2×7-min
    match, while a smaller value yields faster (noisier) matches decided on the score at the cap
    — the frontend's "quick tournament" knob.
    ``on_update`` (optional) is called after every match with the live tournament state and a
    small progress dict — used by the backend service to stream standings to the bracket page.
    ``should_stop`` (optional) is polled before each match; when it returns True the round-robin
    ends early and the returned tournament is marked ``complete=False``.
    """
    labels = [label for label, _ in entrants]
    if len(set(labels)) != len(labels):
        raise ValueError(f"entrant labels must be unique: {labels}")
    standings = {label: Standing(label) for label in labels}
    pairings: list[PairResult] = []

    total_pairs = len(entrants) * (len(entrants) - 1) // 2
    total_matches = total_pairs * matches_per_pairing
    match_no = 0

    for i in range(len(entrants)):
        for j in range(i + 1, len(entrants)):
            la, pa = entrants[i]
            lb, pb = entrants[j]
            if should_stop is not None and should_stop():
                return Tournament(_sorted_table(standings), pairings, complete=False)
            pair = PairResult(a=la, b=lb)
            for m in range(matches_per_pairing):
                cand_home = (m % 2 == 0)                 # alternate which entrant is side A
                kickoff = "a" if cand_home else "b"
                if cand_home:
                    score, ka, kb = play_match(pa, pb, seed=seed + match_no,
                                               first_kickoff=kickoff, max_steps=max_steps)
                    gf, ga, kf, kg = score["a"], score["b"], ka, kb   # gf = la's goals
                else:
                    score, ka, kb = play_match(pb, pa, seed=seed + match_no,
                                               first_kickoff=kickoff, max_steps=max_steps)
                    gf, ga, kf, kg = score["b"], score["a"], kb, ka
                match_no += 1
                _record(standings[la], standings[lb], gf, ga, kf, kg)
                pair.matches += 1
                pair.goals_a += gf
                pair.goals_b += ga
                if gf > ga:
                    pair.wins_a += 1
                elif gf < ga:
                    pair.wins_b += 1
                else:
                    pair.draws += 1
                if on_update is not None:
                    live = Tournament(_sorted_table(standings), pairings + [pair], complete=False)
                    on_update(live, {"match": match_no, "total": total_matches,
                                     "pairing": f"{la} vs {lb}"})
            pairings.append(pair)

    return Tournament(_sorted_table(standings), pairings, complete=True)


def format_table(t: Tournament) -> str:
    """Pretty-print the league table for the CLI."""
    hdr = f"{'#':>2}  {'model':<22} {'P':>3} {'W':>3} {'D':>3} {'L':>3} " \
          f"{'GF':>4} {'GA':>4} {'GD':>4} {'Pts':>4} {'Elo':>6} {'K/m':>6}"
    lines = [hdr, "-" * len(hdr)]
    for rank, s in enumerate(t.table, 1):
        d = s.to_dict()
        lines.append(
            f"{rank:>2}  {s.label[:22]:<22} {s.played:>3} {s.wins:>3} {s.draws:>3} {s.losses:>3} "
            f"{s.goals_for:>4} {s.goals_against:>4} {s.goal_diff:>4} {d['points']:>4} "
            f"{d['elo']:>6} {d['kicks_per_match']:>6}")
    return "\n".join(lines)
