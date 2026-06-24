"""Reward computation for Bucky.

All reward terms are individually weighted and individually logged to TensorBoard.
Potential-based shaping: R_shaping = Φ(s) - Φ(s') so moving toward goal gives +reward.
"""
from __future__ import annotations

from dataclasses import dataclass
from dataclasses import fields as dataclass_fields

import numpy as np

from bucky.game.field import GOAL_HALF_WIDTH, HALF_H, HALF_W, OPP_GOAL, WALL_BAND, robot_in_goal
from bucky.physics.backend import PhysicsState
from bucky.physics.python_backend import MAX_LINEAR

CAPTURE_RADIUS = 0.14
# Spin penalty only kicks in above this turn rate. Kept at ~50% of the drivetrain's real
# max turn rate (physics.MAX_OMEGA ≈ 58 rad/s) so ordinary aiming turns are free and only
# pathological spinning is punished — same fraction this used under the old 6 rad/s cap.
MAX_OMEGA_PENALTY = 29.0
ALIGN_RADIUS = 0.4  # proximity fade-in distance for front_alignment (m)
SHOT_SPEED_THRESHOLD = 1.2  # m/s; above this a free ball counts as a struck shot, not a dribble.
                            # NOTE: with the real MAX_LINEAR≈5.24 m/s a hard dribble can now exceed
                            # this, so the threshold no longer cleanly separates dribble vs kick —
                            # revisit if shot_on_goal starts firing on driven balls.

# Anti-stuck: below these speeds (m/s) the ball/robot count as "not moving".
STUCK_BALL_SPEED = 0.15
STUCK_ROBOT_SPEED = 0.15
# Extra multiplier on the play-out-of-bounds penalty when the robot actively kicks the out ball.
KICK_OOB_EXTRA = 3.0


def _shot_enters_goal_mouth(ball_pos, ball_vel) -> bool:
    """True if a free ball travelling straight from ``ball_pos`` along ``ball_vel`` would reach
    the opponent (+x) goal line *inside the mouth* before crossing any other white line — i.e.
    the shot is genuinely on target and stays in play until it scores.

    Used to gate the kick-shaping reward so that only on-target shots pay out: a goal-ward kick
    that would sail past a side/back line (out of bounds) earns nothing, removing the incentive
    to blast the ball out of play.
    """
    vx, vy = float(ball_vel[0]), float(ball_vel[1])
    if vx <= 1e-6:                                  # not heading toward the +x goal at all
        return False
    t_goal = (HALF_W - float(ball_pos[0])) / vx
    if t_goal <= 0.0:                               # already past the goal line
        return False
    y_at_goal = float(ball_pos[1]) + vy * t_goal
    if abs(y_at_goal) >= GOAL_HALF_WIDTH:           # crosses the +x line outside the mouth → out
        return False
    if vy > 1e-9:                                   # crosses a side line before the goal? → out
        t_side = (HALF_H - float(ball_pos[1])) / vy
        if 0.0 < t_side < t_goal:
            return False
    elif vy < -1e-9:
        t_side = (-HALF_H - float(ball_pos[1])) / vy
        if 0.0 < t_side < t_goal:
            return False
    return True


@dataclass
class RewardConfig:
    w_approach: float = 2.5
    w_ball_to_goal: float = 5.0

    w_possession: float = 4.0
    w_front_align: float = 8.0

    possession_decay_steps: float = 40.0

    w_goal: float = 55.0
    w_goal_against: float = -100.0
    w_in_goal: float = -4.0
    w_predicted_goal: float = 55.0
    predicted_goal_horizon_steps: int = 75

    w_out_of_bounds: float = -50.0
    w_lack_of_progress: float = -5.0
    w_defective: float = -25.0
    w_spin: float = -0.3

    w_play_oob_ball: float = -0.15
    w_stuck: float = -0.1

    w_steal: float = 10.0
    w_blocked_shot: float = 15
    w_kick_goal: float = 12.0
    w_bank_shot: float = 6.0
    w_risky_shot: float = 2.0
    w_kick_lost: float = -8.0
    w_kick_at_opponent: float = -5.0
    w_shot_out_of_bounds: float = -12.0

    w_kick_attempt: float = 0.5
    w_kick_power_to_goal: float = 4

    w_shot_on_goal: float = 2.5

    w_speed: float = 0.1

    w_time: float = -0.05
    w_action_smooth: float = -0.02

    @classmethod
    def from_dict(cls, data: dict | None) -> "RewardConfig":
        """Build from a (partial) mapping of ``w_*`` weights, ignoring unknown keys.

        Missing keys keep their default — so a model config may override only the
        weights it cares about and leave the rest at the tuned defaults above.
        """
        if not data:
            return cls()
        fields = {f.name for f in dataclass_fields(cls)}
        return cls(**{k: float(v) for k, v in data.items() if k in fields})


@dataclass
class RewardTerms:
    approach: float = 0.0
    speed: float = 0.0
    ball_to_goal: float = 0.0

    possession: float = 0.0
    front_alignment: float = 0.0

    goal: float = 0.0
    goal_against: float = 0.0
    predicted_goal: float = 0.0
    in_goal: float = 0.0

    out_of_bounds: float = 0.0
    lack_of_progress: float = 0.0
    defective: float = 0.0
    spin: float = 0.0
    play_oob_ball: float = 0.0
    stuck: float = 0.0

    steal: float = 0.0
    blocked_shot: float = 0.0
    kick_goal: float = 0.0
    bank_shot: float = 0.0
    risky_shot: float = 0.0
    kick_lost: float = 0.0
    kick_at_opponent: float = 0.0
    shot_out_of_bounds: float = 0.0
    kick_attempt: float = 0.0
    kick_power_to_goal: float = 0.0
    shot_on_goal: float = 0.0

    time_penalty: float = 0.0
    action_smoothness: float = 0.0

    @property
    def total(self) -> float:
        return (self.approach + self.speed + self.ball_to_goal + self.possession +
                self.front_alignment + self.goal + self.goal_against +
                self.predicted_goal + self.in_goal +
                self.out_of_bounds + self.lack_of_progress + self.defective +
                self.spin + self.play_oob_ball + self.stuck +
                self.steal + self.blocked_shot + self.kick_goal +
                self.bank_shot + self.risky_shot + self.kick_lost +
                self.kick_at_opponent + self.shot_out_of_bounds + self.kick_attempt +
                self.kick_power_to_goal + self.shot_on_goal + self.time_penalty +
                self.action_smoothness)

    def as_dict(self) -> dict[str, float]:
        return {
            "approach": self.approach,
            "speed": self.speed,
            "ball_to_goal": self.ball_to_goal,
            "possession": self.possession,
            "front_alignment": self.front_alignment,
            "goal": self.goal,
            "goal_against": self.goal_against,
            "predicted_goal": self.predicted_goal,
            "in_goal": self.in_goal,
            "out_of_bounds": self.out_of_bounds,
            "lack_of_progress": self.lack_of_progress,
            "defective": self.defective,
            "spin": self.spin,
            "play_oob_ball": self.play_oob_ball,
            "stuck": self.stuck,
            "steal": self.steal,
            "blocked_shot": self.blocked_shot,
            "kick_goal": self.kick_goal,
            "bank_shot": self.bank_shot,
            "risky_shot": self.risky_shot,
            "kick_lost": self.kick_lost,
            "kick_at_opponent": self.kick_at_opponent,
            "shot_out_of_bounds": self.shot_out_of_bounds,
            "kick_attempt": self.kick_attempt,
            "kick_power_to_goal": self.kick_power_to_goal,
            "shot_on_goal": self.shot_on_goal,
            "time_penalty": self.time_penalty,
            "action_smoothness": self.action_smoothness,
        }


def compute_rewards(
    s0: PhysicsState,
    s1: PhysicsState,
    config: RewardConfig,
    info: dict,
    action: np.ndarray | None = None,
    prev_action: np.ndarray | None = None,
) -> RewardTerms:
    terms = RewardTerms()

    # The ball has crossed the white line this step (in the relocation grace window). While this
    # holds, engaging the ball (chasing, dribbling, kicking it goal-ward) only drives it further
    # out, so the positive shaping below is suppressed and a dedicated penalty is applied instead.
    ball_out_raw = bool(info.get("ball_out_raw", False))

    # Anti-camping decay (shared by possession + front_alignment): fades to 0 the longer the robot
    # lingers near the ball, so lining-up/capturing pays as a setup burst but camping does not. The
    # env supplies ``dwell_steps`` = consecutive steps dwelling within ALIGN_RADIUS facing the ball.
    dwell = float(info.get("dwell_steps", 0))
    dwell_decay = max(0.0, 1.0 - dwell / max(1.0, config.possession_decay_steps))

    d_robot_ball_0 = float(np.linalg.norm(s0.ball_pos - s0.robot_pos))
    d_robot_ball_1 = float(np.linalg.norm(s1.ball_pos - s1.robot_pos))
    terms.approach = config.w_approach * (d_robot_ball_0 - d_robot_ball_1)

    # Speed reward: pay the robot's *actual* velocity component toward its current objective —
    # the ball while chasing, the opponent goal once it has the ball. Normalized by top speed so
    # it sits on the same scale as ``approach``; backward/sideways motion earns nothing (clamped).
    robot_speed = float(np.linalg.norm(s1.robot_vel))
    if robot_speed > 1e-6:
        obj = (OPP_GOAL - s1.robot_pos) if d_robot_ball_1 < CAPTURE_RADIUS \
            else (s1.ball_pos - s1.robot_pos)
        obj_norm = float(np.linalg.norm(obj))
        if obj_norm > 1e-6:
            v_to_obj = float(np.dot(s1.robot_vel, obj / obj_norm))
            terms.speed = config.w_speed * max(0.0, v_to_obj) / MAX_LINEAR

    d_ball_goal_0 = float(np.linalg.norm(s0.ball_pos - OPP_GOAL))
    d_ball_goal_1 = float(np.linalg.norm(s1.ball_pos - OPP_GOAL))
    ball_to_goal = config.w_ball_to_goal * (d_ball_goal_0 - d_ball_goal_1)
    terms.ball_to_goal = min(0.0, ball_to_goal) if ball_out_raw else ball_to_goal

    if d_robot_ball_1 < CAPTURE_RADIUS and not ball_out_raw:
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        if np.dot(heading_vec, s1.ball_pos - s1.robot_pos) > 0:
            terms.possession = config.w_possession * dwell_decay
        else:
            # Facing away with the ball in the capture zone is still a flat penalty (no decay).
            terms.possession = -config.w_possession

    to_goal = OPP_GOAL - s1.ball_pos
    to_goal_dist = float(np.linalg.norm(to_goal))
    if to_goal_dist > 1e-6 and d_robot_ball_1 > 1e-6 and not ball_out_raw:
        goal_dir = to_goal / to_goal_dist
        approach_dir = (s1.ball_pos - s1.robot_pos) / d_robot_ball_1
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        face_ball = float(np.dot(heading_vec, approach_dir))
        drive_pos = float(np.dot(approach_dir, goal_dir))

        # Require BOTH: front aimed at the ball AND positioned behind it, and SQUARE the
        # product so the reward falls off sharply with misalignment. A plain product is
        # ~linear in cos(angle) and only reaches 0 at 90deg, so a near-orthogonal heading
        # (e.g. 65-77deg off the goal, which shoots wide) still paid out heavily. Squaring
        # crushes those marginal line-ups toward 0 while barely denting a true lineup, so the
        # robot is only paid when it is genuinely set up to drive/kick the ball into the goal.
        align = (max(0.0, face_ball) * max(0.0, drive_pos)) ** 2
        prox = max(0.0, 1.0 - d_robot_ball_1 / ALIGN_RADIUS)
        terms.front_alignment = config.w_front_align * align * prox * dwell_decay

    if info.get("goal_scored", False):
        terms.goal = config.w_goal

    if info.get("goal_against", False):
        terms.goal_against = config.w_goal_against

    # Look-ahead "inevitable goal": the env rolled the kicked ball forward and it scores → pay
    # goal-level credit now, at the kick (see env step / predict_goal_by_rollout).
    if info.get("predicted_goal", False):
        terms.predicted_goal = config.w_predicted_goal

    # Heavy penalty for driving inside a goal box (either goal). Per-step, no termination.
    if robot_in_goal(s1.robot_pos):
        terms.in_goal = config.w_in_goal

    if info.get("out_of_bounds", False):
        terms.out_of_bounds = config.w_out_of_bounds

    if info.get("lack_of_progress", False):
        terms.lack_of_progress = config.w_lack_of_progress

    if info.get("defective", False):
        terms.defective = config.w_defective

    # Skilled-play terms. These are driven by flags the env attaches to ``info`` (the
    # opponent-aware ones come from bucky.play_events, since PhysicsState only carries one
    # robot). They default off, so single-agent stages simply leave them zero.
    if info.get("stole_ball", False):
        terms.steal = config.w_steal * float(info.get("steal_gradient", 0.0))
    if info.get("blocked_shot", False):
        terms.blocked_shot = config.w_blocked_shot
    if info.get("goal_scored", False) and info.get("kicked_goal", False):
        terms.kick_goal = config.w_kick_goal
    if info.get("goal_scored", False) and info.get("bank_shot", False):
        terms.bank_shot = config.w_bank_shot
    if info.get("risky_shot", False):
        terms.risky_shot = config.w_risky_shot * float(info.get("risky_factor", 1.0))
    if info.get("kick_lost", False):
        terms.kick_lost = config.w_kick_lost

    # Shot blasted out of play: the env flags this when a *kicked* ball had to be relocated
    # for going out of bounds. The goal exception is structural — a rebound that banks into
    # the goal scores before any relocation, so it never sets this flag — but guard on
    # goal_scored too so the two can never both fire on the same step.
    if info.get("shot_out_of_bounds", False) and not info.get("goal_scored", False):
        terms.shot_out_of_bounds = config.w_shot_out_of_bounds

    # Firing the ball straight into the opponent hands them possession — penalize it directly
    # (the immediate counterpart to kick_lost, which only fires once they actually capture it).
    kick_at_opponent = bool(info.get("kick_at_opponent", False))
    if kick_at_opponent:
        terms.kick_at_opponent = config.w_kick_at_opponent

    # Dense kicker shaping: a legal kick (``info["kicked"]``) is rewarded only when the struck
    # ball is genuinely *on target* — its post-kick trajectory enters the opponent goal mouth
    # before crossing any white line (``_shot_enters_goal_mouth``). A goal-ward kick that would
    # sail out of bounds, a kick into the opponent, or a kick of an already-out ball earns
    # nothing — removing the "blast it anywhere goal-ward" incentive behind random / out-of-play
    # kicks. The reward scales by how squarely the ball is aimed at the goal.
    if info.get("kicked", False) and not kick_at_opponent and not ball_out_raw \
            and _shot_enters_goal_mouth(s1.ball_pos, s1.ball_vel):
        speed = float(np.linalg.norm(s1.ball_vel))
        to_goal = OPP_GOAL - s1.ball_pos
        to_goal_norm = float(np.linalg.norm(to_goal))
        if speed > 1e-6 and to_goal_norm > 1e-6:
            cos_to_goal = float(np.dot(s1.ball_vel / speed, to_goal / to_goal_norm))
            terms.kick_attempt = config.w_kick_attempt
            terms.kick_power_to_goal = config.w_kick_power_to_goal * max(0.0, cos_to_goal)

    # Quick-shot shaping: reward a fast, *free* ball heading at the goal. Gated on the ball
    # being out of the robot's capture radius (so a fast dribble doesn't count) and above a
    # speed threshold (so only a struck shot counts). Only the goalward velocity component is
    # rewarded; a fast ball going the wrong way earns nothing (clamped at 0).
    ball_speed = float(np.linalg.norm(s1.ball_vel))
    if ball_speed > SHOT_SPEED_THRESHOLD and d_robot_ball_1 > CAPTURE_RADIUS and not ball_out_raw:
        to_goal = OPP_GOAL - s1.ball_pos
        to_goal_norm = float(np.linalg.norm(to_goal))
        if to_goal_norm > 1e-6:
            goal_dir = to_goal / to_goal_norm
            vel_to_goal = float(np.dot(s1.ball_vel, goal_dir))
            terms.shot_on_goal = config.w_shot_on_goal * max(0.0, vel_to_goal)

    # Playing an out-of-bounds ball: while it sits past the white line (relocation grace window),
    # chasing or re-kicking it only drives it further out. Penalize per step, scaled by how far
    # past the line the ball is, with an extra slap for actively kicking it out there.
    if ball_out_raw:
        over = max(abs(float(s1.ball_pos[0])) - HALF_W,
                   abs(float(s1.ball_pos[1])) - HALF_H, 0.0)
        pen = 1.0 + over / WALL_BAND
        if info.get("kicked", False):
            pen += KICK_OOB_EXTRA
        terms.play_oob_ball = config.w_play_oob_ball * pen

    # Anti-stuck: not in possession, the ball essentially stationary, and the robot barely moving
    # — i.e. idling instead of going to the ball (including pinned against a wall). A small per-step
    # nudge to keep it active; brief decelerations cost little.
    if (d_robot_ball_1 > CAPTURE_RADIUS and not ball_out_raw
            and ball_speed < STUCK_BALL_SPEED and robot_speed < STUCK_ROBOT_SPEED):
        terms.stuck = config.w_stuck

    excess_spin = max(0.0, abs(s1.robot_omega) - MAX_OMEGA_PENALTY)
    terms.spin = config.w_spin * excess_spin

    terms.time_penalty = config.w_time

    if action is not None and prev_action is not None:
        # Anti-rocking: penalize the change in the drive command between steps (kick dim excluded —
        # it has its own cooldown gating and dedicated rewards). Steady driving → ~0; flipping the
        # command back-and-forth each step → large penalty.
        da = np.asarray(action[:3]) - np.asarray(prev_action[:3])
        terms.action_smoothness = config.w_action_smooth * float(np.sum(da ** 2))

    return terms
