import numpy as np
import pytest
from bucky.physics.python_backend import PyPhysics, FIELD_W
from bucky.rewards import (  # noqa: F401
    RewardConfig, RewardTerms, compute_rewards, ALIGN_RADIUS, OPP_GOAL,
    CAPTURE_RADIUS, SHOT_SPEED_THRESHOLD,
)

@pytest.fixture
def config():
    return RewardConfig()

@pytest.fixture
def physics():
    p = PyPhysics()
    p.reset(seed=0)
    return p

def _state_pair(physics, vx=0.0):
    s0 = physics._make_state()
    physics.step(vx, 0.0, 0.0)
    s1 = physics._make_state()
    return s0, s1

def test_reward_terms_all_finite(physics, config):
    s0, s1 = _state_pair(physics)
    terms = compute_rewards(s0, s1, config, info={})
    for name, val in terms.as_dict().items():
        assert np.isfinite(val), f"{name} is not finite: {val}"

def test_approach_potential_based(physics, config):
    s0, s1 = _state_pair(physics, vx=1.0)
    terms = compute_rewards(s0, s1, config, info={})
    d0 = np.linalg.norm(s0.ball_pos - s0.robot_pos)
    d1 = np.linalg.norm(s1.ball_pos - s1.robot_pos)
    expected = config.w_approach * (d0 - d1)
    assert abs(terms.approach - expected) < 1e-6

def test_ball_to_goal_potential_based(physics, config):
    opp_goal = np.array([FIELD_W / 2, 0.0])
    s0, s1 = _state_pair(physics, vx=1.0)
    terms = compute_rewards(s0, s1, config, info={})
    d0 = np.linalg.norm(s0.ball_pos - opp_goal)
    d1 = np.linalg.norm(s1.ball_pos - opp_goal)
    expected = config.w_ball_to_goal * (d0 - d1)
    assert abs(terms.ball_to_goal - expected) < 1e-6

def test_goal_bonus_on_goal_scored(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"goal_scored": True})
    assert terms.goal > 0.0

def test_out_of_bounds_penalty(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"out_of_bounds": True})
    assert terms.out_of_bounds < 0.0

def _place(physics, robot_pos, heading, ball_pos):
    physics._robot_pos = np.asarray(robot_pos, dtype=float)
    physics._robot_heading = float(heading)
    physics._ball_pos = np.asarray(ball_pos, dtype=float)
    return physics._make_state()

def test_front_alignment_rewards_correct_setup(physics, config):
    # Robot just behind the ball (-x side), front facing the ball (+x) — which
    # is also toward the goal, so it can catch and drive to the goal.
    ball = np.array([0.0, 0.0])
    robot = ball - np.array([0.1, 0.0])  # within ALIGN_RADIUS, behind the ball
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment > 0.0

def test_front_alignment_penalizes_wrong_side_and_rotation(physics, config):
    # Robot on the goal (+x) side of the ball, front pointing away from the ball
    # (toward the goal): can't catch it AND wrong side to drive it.
    ball = np.array([0.0, 0.0])
    robot = ball + np.array([0.1, 0.0])
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment < 0.0

def test_front_alignment_gated_by_proximity(physics, config):
    # Robot beyond ALIGN_RADIUS from the ball -> term fades to zero.
    ball = np.array([0.0, 0.0])
    robot = ball - np.array([ALIGN_RADIUS + 0.1, 0.0])
    s = _place(physics, robot, 0.0, ball)
    terms = compute_rewards(s, s, config, info={})
    assert terms.front_alignment == 0.0

def test_front_alignment_finite_when_ball_on_goal(physics, config):
    # Degenerate: ball exactly at the goal -> term stays finite (0).
    s = _place(physics, np.array([1.1, 0.0]), 0.0, OPP_GOAL.copy())
    terms = compute_rewards(s, s, config, info={})
    assert np.isfinite(terms.front_alignment)

def test_total_is_weighted_sum(physics, config):
    s0, s1 = _state_pair(physics)
    terms = compute_rewards(s0, s1, config, info={})
    d = terms.as_dict()
    expected = sum(d.values())
    assert abs(terms.total - expected) < 1e-6


# ── skilled-play terms (driven by env-supplied info flags) ───────────────────
def test_steal_scales_with_gradient(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"stole_ball": True, "steal_gradient": 0.8})
    assert abs(terms.steal - config.w_steal * 0.8) < 1e-6


def test_steal_zero_without_flag(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={}).steal == 0.0


def test_blocked_shot_positive(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={"blocked_shot": True}).blocked_shot > 0.0


def test_kick_goal_bonus_only_on_goal(physics, config):
    s = physics._make_state()
    scored = compute_rewards(s, s, config, info={"goal_scored": True, "kicked_goal": True})
    assert abs(scored.kick_goal - config.w_kick_goal) < 1e-6
    # kicked flag without an actual goal → no bonus
    assert compute_rewards(s, s, config, info={"kicked_goal": True}).kick_goal == 0.0


def test_bank_shot_bonus_only_on_goal(physics, config):
    s = physics._make_state()
    scored = compute_rewards(s, s, config, info={"goal_scored": True, "bank_shot": True})
    assert abs(scored.bank_shot - config.w_bank_shot) < 1e-6
    assert compute_rewards(s, s, config, info={"bank_shot": True}).bank_shot == 0.0


def test_risky_shot_scales_with_factor(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"risky_shot": True, "risky_factor": 0.5})
    assert abs(terms.risky_shot - config.w_risky_shot * 0.5) < 1e-6


def test_kick_lost_is_extra_punishment(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={"kick_lost": True}).kick_lost < 0.0


def _place_kick(physics, ball, ball_vel):
    # Robot just behind the ball; the kick is judged by the ball's post-kick velocity.
    physics._robot_pos = np.asarray(ball - np.array([0.1, 0.0]), dtype=float)
    physics._robot_heading = 0.0
    physics._ball_pos = np.asarray(ball, dtype=float)
    physics._ball_vel = np.asarray(ball_vel, dtype=float)
    return physics._make_state()


def test_kick_rewards_on_target_shot(physics, config):
    # A struck ball whose trajectory enters the goal mouth → kick_power_to_goal pays out (cos≈1).
    s = _place_kick(physics, np.array([0.0, 0.0]), [3.0, 0.0])
    terms = compute_rewards(s, s, config, info={"kicked": True})
    assert abs(terms.kick_attempt - config.w_kick_attempt) < 1e-6     # flat bonus off by default
    assert terms.kick_power_to_goal > 0.0
    assert abs(terms.kick_power_to_goal - config.w_kick_power_to_goal) < 1e-3  # cos≈1


def test_kick_zero_for_off_target_shot(physics, config):
    # Goal-ward-ish but aimed so it sails out the side line before reaching the goal → no reward.
    s = _place_kick(physics, np.array([0.5, 0.4]), [1.0, 3.0])
    terms = compute_rewards(s, s, config, info={"kicked": True})
    assert terms.kick_power_to_goal == 0.0
    assert terms.kick_attempt == 0.0


def test_kick_zero_for_backward_kick(physics, config):
    # Ball fired toward our own half earns nothing.
    s = _place_kick(physics, np.array([0.0, 0.0]), [-3.0, 0.0])
    terms = compute_rewards(s, s, config, info={"kicked": True})
    assert terms.kick_attempt == 0.0
    assert terms.kick_power_to_goal == 0.0


def test_kick_attempt_zero_without_flag(physics, config):
    s = _place_kick(physics, np.array([0.0, 0.0]), [3.0, 0.0])
    terms = compute_rewards(s, s, config, info={})
    assert terms.kick_attempt == 0.0
    assert terms.kick_power_to_goal == 0.0


def test_kick_at_opponent_penalized_and_suppresses_bonus(physics, config):
    # A goal-ward kick that is flagged as fired into the opponent: it must be penalized AND
    # earn none of the positive kick shaping (no feeding the enemy for the attempt bonus).
    ball = np.array([0.0, 0.0])
    s = _place(physics, ball - np.array([0.1, 0.0]), 0.0, ball)
    terms = compute_rewards(s, s, config, info={"kicked": True, "kick_at_opponent": True})
    assert abs(terms.kick_at_opponent - config.w_kick_at_opponent) < 1e-6
    assert terms.kick_at_opponent < 0.0
    assert terms.kick_attempt == 0.0
    assert terms.kick_power_to_goal == 0.0


def test_kick_lost_penalty_is_substantial(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"kick_lost": True})
    assert abs(terms.kick_lost - config.w_kick_lost) < 1e-6
    assert terms.kick_lost <= -10.0   # giving the enemy the ball is a heavy penalty


def test_shot_out_of_bounds_is_heavily_penalized(physics, config):
    s = physics._make_state()
    terms = compute_rewards(s, s, config, info={"shot_out_of_bounds": True})
    assert abs(terms.shot_out_of_bounds - config.w_shot_out_of_bounds) < 1e-6
    assert terms.shot_out_of_bounds < 0.0
    assert terms.shot_out_of_bounds <= -20.0   # really punishing: a wasted shot out of play


def test_shot_out_of_bounds_zero_without_flag(physics, config):
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={}).shot_out_of_bounds == 0.0


def test_shot_out_of_bounds_waived_when_it_scores(physics, config):
    # A rebound/bank that goes out of bounds but banks into the goal must NOT be punished.
    s = physics._make_state()
    terms = compute_rewards(s, s, config,
                            info={"shot_out_of_bounds": True, "goal_scored": True})
    assert terms.shot_out_of_bounds == 0.0
    assert terms.goal > 0.0


# ── shot_on_goal: reward a fast ball struck toward goal (not a dribble) ───────
def _place_with_ball_vel(physics, robot_pos, ball_pos, ball_vel):
    physics._robot_pos = np.asarray(robot_pos, dtype=float)
    physics._robot_heading = 0.0
    physics._ball_pos = np.asarray(ball_pos, dtype=float)
    physics._ball_vel = np.asarray(ball_vel, dtype=float)
    return physics._make_state()


def test_shot_on_goal_rewards_fast_goalward_ball(physics, config):
    # Fast ball heading at the +x goal, robot far away (ball in flight, not dribbled).
    ball = np.array([0.0, 0.0])
    speed = SHOT_SPEED_THRESHOLD + 1.0
    s = _place_with_ball_vel(physics, [-2.0, 0.0], ball, [speed, 0.0])
    terms = compute_rewards(s, s, config, info={})
    goal_dir = OPP_GOAL - ball
    goal_dir = goal_dir / np.linalg.norm(goal_dir)
    expected = config.w_shot_on_goal * float(np.dot([speed, 0.0], goal_dir))
    assert terms.shot_on_goal > 0.0
    assert abs(terms.shot_on_goal - expected) < 1e-6


def test_shot_on_goal_zero_for_slow_ball(physics, config):
    # A slow (dribbled) ball toward the goal does not count as a shot.
    ball = np.array([0.0, 0.0])
    s = _place_with_ball_vel(physics, [-2.0, 0.0], ball, [SHOT_SPEED_THRESHOLD - 0.3, 0.0])
    assert compute_rewards(s, s, config, info={}).shot_on_goal == 0.0


def test_shot_on_goal_zero_for_ball_away_from_goal(physics, config):
    # Fast ball, but moving away from the opponent goal → clamped to zero.
    ball = np.array([0.0, 0.0])
    s = _place_with_ball_vel(physics, [-2.0, 0.0], ball, [-(SHOT_SPEED_THRESHOLD + 1.0), 0.0])
    assert compute_rewards(s, s, config, info={}).shot_on_goal == 0.0


def test_shot_on_goal_zero_when_ball_in_capture(physics, config):
    # Fast ball but still in the robot's capture radius → it's being dribbled, not a shot.
    ball = np.array([0.0, 0.0])
    robot = ball - np.array([CAPTURE_RADIUS * 0.5, 0.0])
    s = _place_with_ball_vel(physics, robot, ball, [SHOT_SPEED_THRESHOLD + 1.0, 0.0])
    assert compute_rewards(s, s, config, info={}).shot_on_goal == 0.0


def test_action_smoothness_ignores_kick_dim(physics, config):
    s = physics._make_state()
    prev = np.array([0.0, 0.0, 0.0, 0.0])
    action = np.array([0.5, 0.5, 0.0, 1.0])   # big kick dim must not be penalized
    terms = compute_rewards(s, s, config, info={}, action=action, prev_action=prev)
    expected = config.w_action_smooth * (0.5**2 + 0.5**2)
    assert abs(terms.action_smoothness - expected) < 1e-6


def test_action_smoothness_zero_without_prev_action(physics, config):
    # Without a previous action there's nothing to compare against → no penalty.
    s = physics._make_state()
    action = np.array([1.0, 1.0, 1.0, 0.0])
    terms = compute_rewards(s, s, config, info={}, action=action)
    assert terms.action_smoothness == 0.0


def test_action_smoothness_penalizes_reversal(physics, config):
    # Flipping the command back-and-forth (rocking) must cost more than holding it steady.
    s = physics._make_state()
    steady = compute_rewards(s, s, config, info={},
                             action=np.array([1.0, 0.0, 0.0, 0.0]),
                             prev_action=np.array([1.0, 0.0, 0.0, 0.0]))
    rocking = compute_rewards(s, s, config, info={},
                              action=np.array([-1.0, 0.0, 0.0, 0.0]),
                              prev_action=np.array([1.0, 0.0, 0.0, 0.0]))
    assert steady.action_smoothness == 0.0
    assert rocking.action_smoothness < steady.action_smoothness


# ── playing an out-of-bounds ball: penalize + suppress positive shaping ───────
def test_play_oob_ball_penalizes_while_ball_out(physics, config):
    # Ball past the white line: a per-step penalty applies, scaled by how far out it is.
    near = _place(physics, [0.0, 0.0], 0.0, [0.95, 0.0])      # just over the +x line
    far = _place(physics, [0.0, 0.0], 0.0, [1.15, 0.0])       # deep into the band
    t_near = compute_rewards(near, near, config, info={"ball_out_raw": True})
    t_far = compute_rewards(far, far, config, info={"ball_out_raw": True})
    assert t_near.play_oob_ball < 0.0
    assert t_far.play_oob_ball < t_near.play_oob_ball          # further out → harsher


def test_play_oob_ball_extra_penalty_for_kicking_it(physics, config):
    s = _place(physics, [0.0, 0.0], 0.0, [0.95, 0.0])
    idle = compute_rewards(s, s, config, info={"ball_out_raw": True})
    kicked = compute_rewards(s, s, config, info={"ball_out_raw": True, "kicked": True})
    assert kicked.play_oob_ball < idle.play_oob_ball


def test_ball_out_suppresses_positive_shaping(physics, config):
    # In contact with the ball and facing it would normally earn possession + front_alignment;
    # while the ball is out those positives are suppressed, and the kick term pays nothing.
    ball = np.array([1.0, 0.0])                  # out of play (past +x line, in mouth-x but x>HALF_W)
    s = _place_kick(physics, ball, [3.0, 0.0])
    out = compute_rewards(s, s, config, info={"ball_out_raw": True, "kicked": True})
    assert out.possession == 0.0
    assert out.front_alignment == 0.0
    assert out.kick_power_to_goal == 0.0


# ── anti-stuck ───────────────────────────────────────────────────────────────
def test_stuck_fires_when_idle_away_from_ball(physics, config):
    # Robot far from a still ball, both essentially motionless → a small penalty.
    s = _place_with_ball_vel(physics, [-0.5, 0.0], [0.5, 0.0], [0.0, 0.0])
    terms = compute_rewards(s, s, config, info={})
    assert abs(terms.stuck - config.w_stuck) < 1e-6
    assert terms.stuck < 0.0


def test_stuck_zero_when_moving(physics, config):
    # Same geometry but the robot is driving toward the ball → not stuck.
    physics._robot_pos = np.array([-0.5, 0.0]); physics._robot_heading = 0.0
    physics._ball_pos = np.array([0.5, 0.0]); physics._ball_vel = np.zeros(2)
    physics._robot_vel = np.array([1.0, 0.0])
    s = physics._make_state()
    assert compute_rewards(s, s, config, info={}).stuck == 0.0


def test_stuck_zero_in_possession(physics, config):
    # Ball in the capture radius (possession) → idle is not "stuck".
    s = _place(physics, [0.0, 0.0], 0.0, [CAPTURE_RADIUS * 0.5, 0.0])
    assert compute_rewards(s, s, config, info={}).stuck == 0.0


# ── on-target shot helper ────────────────────────────────────────────────────
def test_shot_enters_goal_mouth_helper():
    from bucky.rewards import _shot_enters_goal_mouth
    # straight at the goal centre → on target
    assert _shot_enters_goal_mouth([0.0, 0.0], [3.0, 0.0]) is True
    # angled so it crosses the side line first → off target
    assert _shot_enters_goal_mouth([0.5, 0.4], [1.0, 3.0]) is False
    # moving away from the goal → off target
    assert _shot_enters_goal_mouth([0.0, 0.0], [-3.0, 0.0]) is False
