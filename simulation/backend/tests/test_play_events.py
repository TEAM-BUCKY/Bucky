"""Tests for the opponent-aware play-event tracker (steal / block / risky / kick-lost / skilled goals)."""
import numpy as np

from bucky.physics.backend import PhysicsState
from bucky.play_events import PlayEventTracker, CONTACT_DIST


def _st(robot_pos, heading, ball_pos, ball_vel=(0.0, 0.0)):
    return PhysicsState(
        robot_pos=np.array(robot_pos, dtype=float), robot_vel=np.zeros(2),
        robot_heading=float(heading), robot_omega=0.0,
        ball_pos=np.array(ball_pos, dtype=float), ball_vel=np.array(ball_vel, dtype=float),
    )


def test_steal_fires_on_b_to_a_transition_with_gradient():
    t = PlayEventTracker()
    ball = [0.4, 0.0]
    # Step 1: B controls the ball (near + facing), A far away.
    out1 = t.update(_st([-0.5, 0.0], 0.0, ball), _st([0.3, 0.0], 0.0, ball), {})
    assert "stole_ball" not in out1
    # Step 2: A now controls it, B has left → steal.
    out2 = t.update(_st([0.3, 0.0], 0.0, ball), _st([0.9, 0.0], 0.0, ball), {})
    assert out2["stole_ball"] is True
    assert out2["steal_gradient"] == np.clip(0.5 + 0.5 * 0.4 / 0.915, 0.0, 1.0)


def test_kicked_goal_vs_dribble():
    t = PlayEventTracker()
    # A kicks (arms the window), ball free in front.
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), _st([0.8, 0.8], 0.0, [0.14, 0.0]),
             {"kicked_a": True})
    # Goal arrives with the ball free (A far from it) → kicked goal.
    out = t.update(_st([0.5, 0.0], 0.0, [0.95, 0.0]), _st([0.8, 0.8], 0.0, [0.95, 0.0]),
                   {"goal_a": True})
    assert out.get("kicked_goal") is True

    # A plain goal with no recent kick is not a kicked goal.
    t2 = PlayEventTracker()
    out2 = t2.update(_st([0.9, 0.0], 0.0, [0.95, 0.0]), _st([0.0, 0.0], 0.0, [0.95, 0.0]),
                     {"goal_a": True})
    assert out2.get("kicked_goal") is not True


def test_bank_shot_requires_bounce_after_kick():
    t = PlayEventTracker()
    far_b = _st([0.8, 0.8], 0.0, [0.14, 0.0])
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), far_b, {"kicked_a": True})
    t.update(_st([0.2, 0.0], 0.0, [0.5, 0.3]), _st([0.8, 0.8], 0.0, [0.5, 0.3]),
             {"ball_wall_bounce": True})
    out = t.update(_st([0.3, 0.0], 0.0, [0.95, 0.2]), _st([0.8, 0.8], 0.0, [0.95, 0.2]),
                   {"goal_a": True})
    assert out.get("bank_shot") is True


def test_kick_lost_when_enemy_captures_after_kick():
    t = PlayEventTracker()
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), _st([0.8, 0.8], 0.0, [0.14, 0.0]),
             {"kicked_a": True})
    # Enemy B gains control shortly after, no goal → kick lost.
    ball = [0.3, 0.0]
    out = t.update(_st([-0.5, 0.0], 0.0, ball), _st([0.2, 0.0], 0.0, ball), {})
    assert out.get("kick_lost") is True


def test_blocked_shot_on_own_goal():
    t = PlayEventTracker()
    # Step 1: a live incoming shot on A's (−x) goal, A not yet on it.
    out1 = t.update(_st([0.2, 0.0], 0.0, [-0.3, 0.0], ball_vel=[-1.0, 0.0]),
                    _st([0.8, 0.0], np.pi, [-0.3, 0.0], ball_vel=[-1.0, 0.0]), {})
    assert "blocked_shot" not in out1
    # Step 2: A is on the ball and the shot is no longer live (deflected away).
    out2 = t.update(_st([-0.35, 0.0], 0.0, [-0.35, 0.0], ball_vel=[0.5, 0.0]),
                    _st([0.8, 0.0], np.pi, [-0.35, 0.0], ball_vel=[0.5, 0.0]), {})
    assert out2.get("blocked_shot") is True


def test_risky_shot_threads_past_defender():
    t = PlayEventTracker()
    # A kicks along +x toward the goal mouth; B sits near the shot line but far enough that the
    # ball *clears* it (perp 0.2 m > BLOCK_RADIUS ≈ 0.131 m, < RISKY_RADIUS 0.30 m).
    out = t.update(_st([0.0, 0.0], 0.0, [0.1, 0.0]),
                   _st([0.4, 0.2], np.pi, [0.1, 0.0]), {"kicked_a": True})
    assert out.get("risky_shot") is True
    assert 0.0 < out["risky_factor"] <= 1.0
    assert "kick_at_opponent" not in out


def test_kick_at_opponent_when_fired_into_defender():
    t = PlayEventTracker()
    # B sits right on the shot line and close (perp 0.05 m < BLOCK_RADIUS) → the ball hits it.
    out = t.update(_st([0.0, 0.0], 0.0, [0.1, 0.0]),
                   _st([0.4, 0.05], np.pi, [0.1, 0.0]), {"kicked_a": True})
    assert out.get("kick_at_opponent") is True
    assert "risky_shot" not in out          # a hit is not a thread


def test_no_risky_shot_when_opponent_off_the_line():
    t = PlayEventTracker()
    out = t.update(_st([0.0, 0.0], 0.0, [0.1, 0.0]),
                   _st([0.4, 0.5], np.pi, [0.1, 0.0]), {"kicked_a": True})
    assert "risky_shot" not in out
    assert "kick_at_opponent" not in out


def test_shot_out_of_bounds_on_relocation_without_goal():
    t = PlayEventTracker()
    far_b = _st([0.8, 0.8], 0.0, [0.14, 0.0])
    # A kicks the ball.
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), far_b, {"kicked_a": True})
    # Ball travels well clear of A (departs), still in play — no penalty yet.
    mid = t.update(_st([0.0, 0.0], 0.0, [0.6, 0.4]), far_b, {})
    assert "shot_out_of_bounds" not in mid
    # Referee relocates the ball for going out of play (out_of_reach) with no goal → penalty.
    out = t.update(_st([0.0, 0.0], 0.0, [1.0, 0.7]), far_b, {"ball_oob_relocated": True})
    assert out.get("shot_out_of_bounds") is True


def test_shot_out_of_bounds_not_flagged_when_shot_scores():
    t = PlayEventTracker()
    far_b = _st([0.8, 0.8], 0.0, [0.14, 0.0])
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), far_b, {"kicked_a": True})
    t.update(_st([0.0, 0.0], 0.0, [0.6, 0.0]), far_b, {})        # departs
    # Bank/rebound scores → goal_a; even with a relocation flag present, never penalized.
    out = t.update(_st([0.0, 0.0], 0.0, [0.95, 0.0]), far_b,
                   {"goal_a": True, "ball_oob_relocated": True})
    assert out.get("shot_out_of_bounds") is not True
    # And a later out-of-bounds relocation is not retroactively blamed on the scored shot.
    out2 = t.update(_st([0.0, 0.0], 0.0, [1.0, 0.7]), far_b, {"ball_oob_relocated": True})
    assert out2.get("shot_out_of_bounds") is not True


def test_shot_out_of_bounds_not_flagged_after_a_recovers_ball():
    t = PlayEventTracker()
    far_b = _st([0.8, 0.8], 0.0, [0.14, 0.0])
    t.update(_st([0.0, 0.0], 0.0, [0.14, 0.0]), far_b, {"kicked_a": True})
    t.update(_st([0.0, 0.0], 0.0, [0.6, 0.0]), far_b, {})        # departs
    # A catches up to the ball (controls it again) → shot resolved, no penalty.
    t.update(_st([0.95, 0.0], 0.0, [1.0, 0.0]), far_b, {})
    out = t.update(_st([0.0, 0.0], 0.0, [1.0, 0.7]), far_b, {"ball_oob_relocated": True})
    assert out.get("shot_out_of_bounds") is not True


def test_shot_out_of_bounds_needs_a_kick():
    # A ball relocated out of bounds with no preceding A kick is not A's shot-out-of-bounds.
    t = PlayEventTracker()
    far_b = _st([0.8, 0.8], 0.0, [0.14, 0.0])
    out = t.update(_st([0.0, 0.0], 0.0, [1.0, 0.7]), far_b, {"ball_oob_relocated": True})
    assert "shot_out_of_bounds" not in out


def test_contact_dist_constant_is_sane():
    assert CONTACT_DIST > 0.0
