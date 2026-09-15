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
OWN_GOAL = -OPP_GOAL            # robot A defends the -x goal
DEFENSE_LANE_HALF = 0.35       # m; perpendicular half-width of the "between ball and own goal" lane
QUICK_SHOT_WINDOW = 30.0       # steps (~0.6 s); a scoring shot fired within this of engaging the
                               # ball counts as a fast grab-and-shoot (full bonus at dwell 0)
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
    # Lowered 8.5→5.0: front_alignment is a per-step lineup nudge and was the single dominant
    # reward (~+500/ep in self-play), so the policy farmed "line up behind the ball" instead of
    # optimising precise *scoring*. Cut so the accuracy-conditioned kick/goal rewards dominate.
    w_front_align: float = 5.0
    # Per-step penalty for loitering near the ball while NOT lined up to drive/kick it at the goal
    # ("when aiming is possible, you should be aimed"). Scales with misalignment and closeness.
    w_front_misalign: float = -2.0
    # Defensive positioning: reward for being goal-side of the ball (between it and our own goal,
    # on the blocking line) while the ball is in our defensive half — pulls the learner back to
    # defend instead of always chasing forward (self-play only; single drills have no defender).
    w_defense_position: float = 3.0
    # Anti-spray: penalty for firing the kicker in the attacking half when the shot is NOT goal-bound
    # (a wasted shot). Defensive-half clearances are exempt. -3.0 over-suppressed attacking and, with
    # extended training, helped drive a passive mutual-stalemate (run#7: 0.6 kicks/ep). Eased to -1.0
    # so the kicker stays worth firing while still discouraging pure spray.
    w_wasted_kick: float = -1.0

    possession_decay_steps: float = 40.0

    # Scoring raised 55→75 toward symmetry with the -100 conceding penalty: at 55 vs -100 the agent
    # played risk-averse (run#5: concedes 2.0 but only scores 0.67), so push offence without making
    # conceding cheap.
    w_goal: float = 75.0
    w_goal_against: float = -70.0   # -100→-70: at -100 vs +75 scoring the policy plays for the draw
                                    # (run#19: fewest losses but 13 draws / only 52 GF). Lighter concede
                                    # penalty + strong scoring → play to WIN (draws→wins = more points).
    w_in_goal: float = -4.0
    w_predicted_goal: float = 75.0
    predicted_goal_horizon_steps: int = 75

    w_out_of_bounds: float = -50.0
    w_lack_of_progress: float = -5.0
    w_defective: float = -25.0
    w_spin: float = -0.3

    w_play_oob_ball: float = -0.15
    w_stuck: float = -0.1

    w_steal: float = 10.0       # reverted 18→10: user reports conceding is rare, so defense isn't the
                                # gap; the steal boost risked a give-and-resteal kickoff exploit.
    w_blocked_shot: float = 15
    w_kick_goal: float = 18.0   # 12→18: reward real (non-predicted) kicked goals more, part of the
                                # run#6 offence push.
    w_bank_shot: float = 20.0   # 6→15→20: actively reward banking a shot off a wall into the goal
                                # (e.g. around a defender) on top of predicted_goal — user wants more banks.
    w_risky_shot: float = 2.0
    w_kick_lost: float = -8.0
    w_kick_at_opponent: float = -5.0
    w_shot_out_of_bounds: float = -12.0
    # Anti-corner-camp: per-step penalty for the ball sitting in the dead attacking corner (deep past
    # the goal mouth, against a side wall — no scoring angle). Stops the robot dribbling it there and
    # getting stuck instead of pulling back for a shot.
    w_corner_camp: float = -0.4

    w_kick_attempt: float = 2.0   # 0.5→2.0: reward TAKING an on-target shot (one-time, gated on the
                                  # shot entering the mouth) so the robot actually shoots — incl.
                                  # long-range open-goal shots — instead of dribbling the ball in.
    # Strengthened 4→8→14 and squared (see compute): the main *accuracy-conditioned*, one-time
    # (non-farmable) shot reward. Raised again because self-play (front_alignment stripped) finishes
    # open goals poorly — this is the aiming gradient that survives into match play.
    w_kick_power_to_goal: float = 14
    # Continuous aiming gradient: like kick_power_to_goal but NOT gated on the shot already being on
    # target, so a wide shot earns partial reward that grows toward the goal centre — the smooth
    # "aim closer" signal the gated rewards can't give (they pay a miss zero). One-time per kick.
    w_kick_aim: float = 4.0
    # Shot-ANGLE quality: bonus for kicking from a position with a WIDE view of the goal mouth
    # (central) vs a narrow corner angle. One-time per kick (non-farmable). Targets v23's #1 weakness
    # — funnelling attacks into the corner (2% conversion) and firing junk instead of carrying central
    # (60% conversion). Pushes it to reposition to a good angle before shooting.
    w_kick_angle: float = 8.0
    # Quick-release: bonus for a SCORING shot (direct or banked) fired fast after grabbing the ball
    # (low dwell). Rewards the snatch-and-immediately-bank finish that beats the defender, instead of
    # dribbling/holding first. Scales from full (instant) to 0 at QUICK_SHOT_WINDOW dwell.
    w_quick_shot: float = 8.0

    # Per-step on-target-shot shaping. Kept modest: it accumulates every step a struck ball is in
    # flight, so over a 1500-step self-play episode even the now-accuracy-gated term can dwarf the
    # terminal scoring reward (predicted_goal/kick_goal). Lowered 2.5→1.0 so scoring stays the
    # dominant objective and the policy optimises goals, not time-on-target.
    w_shot_on_goal: float = 1.0

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
    front_misalign: float = 0.0
    defense_position: float = 0.0
    wasted_kick: float = 0.0
    corner_camp: float = 0.0

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
    kick_aim: float = 0.0
    kick_angle: float = 0.0
    quick_shot: float = 0.0
    shot_on_goal: float = 0.0

    time_penalty: float = 0.0
    action_smoothness: float = 0.0

    @property
    def total(self) -> float:
        return (self.approach + self.speed + self.ball_to_goal + self.possession +
                self.front_alignment + self.front_misalign + self.defense_position +
                self.wasted_kick + self.corner_camp + self.goal + self.goal_against +
                self.predicted_goal + self.in_goal +
                self.out_of_bounds + self.lack_of_progress + self.defective +
                self.spin + self.play_oob_ball + self.stuck +
                self.steal + self.blocked_shot + self.kick_goal +
                self.bank_shot + self.risky_shot + self.kick_lost +
                self.kick_at_opponent + self.shot_out_of_bounds + self.kick_attempt +
                self.kick_power_to_goal + self.kick_aim + self.kick_angle + self.quick_shot +
                self.shot_on_goal + self.time_penalty +
                self.action_smoothness)

    def as_dict(self) -> dict[str, float]:
        return {
            "approach": self.approach,
            "speed": self.speed,
            "ball_to_goal": self.ball_to_goal,
            "possession": self.possession,
            "front_alignment": self.front_alignment,
            "front_misalign": self.front_misalign,
            "defense_position": self.defense_position,
            "wasted_kick": self.wasted_kick,
            "corner_camp": self.corner_camp,
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
            "kick_aim": self.kick_aim,
            "kick_angle": self.kick_angle,
            "quick_shot": self.quick_shot,
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

        # Require BOTH: front aimed at the ball AND positioned behind it, and CUBE the
        # product so the reward falls off very sharply with misalignment. A plain product is
        # ~linear in cos(angle) and only reaches 0 at 90deg, so a near-orthogonal heading
        # (e.g. 65-77deg off the goal, which shoots wide) still paid out heavily. Squaring
        # already crushed marginal line-ups; cubing crushes them harder still (e.g. a 0.8
        # cos-product: 0.64 squared -> 0.51 cubed) while barely denting a true lineup, so the
        # robot is only paid when it is genuinely set up to drive/kick the ball into the goal.
        # (w_front_align bumped to 8.5 to restore the peak magnitude cubing slightly lowers.)
        aligned = max(0.0, face_ball) * max(0.0, drive_pos)   # raw lineup quality, 1 = perfect
        align = aligned ** 3
        prox = max(0.0, 1.0 - d_robot_ball_1 / ALIGN_RADIUS)
        terms.front_alignment = config.w_front_align * align * prox * dwell_decay

        # Counterpart penalty: while the robot is near the ball (prox > 0) but NOT lined up to
        # drive/kick it at the goal, dock points proportional to the misalignment (1 - aligned)
        # and closeness. Uses the *raw* lineup quality (not cubed) so the penalty grows linearly
        # with the angle, and deliberately carries NO dwell decay — a robot loitering on the ball
        # while poorly aimed keeps paying, pushing it to either line up and commit (kick) or leave
        # rather than hover half-aimed and shoot wide.
        terms.front_misalign = config.w_front_misalign * (1.0 - aligned) * prox

    # Defensive positioning (self-play): while the ball is in our defensive half, reward being
    # goal-side of it — on the ball→own-goal line, between the ball and the goal (not in the net),
    # scaled by how deep the threat is. Pulls the learner back to defend instead of ball-chasing.
    if float(s1.ball_pos[0]) < 0.0 and not ball_out_raw:
        to_owngoal = OWN_GOAL - s1.ball_pos
        d_og = float(np.linalg.norm(to_owngoal))
        if d_og > 1e-6:
            gdir = to_owngoal / d_og
            rel = s1.robot_pos - s1.ball_pos
            along = float(np.dot(rel, gdir))                  # >0 → robot is goal-side of the ball
            if along > 0.0:
                lateral = float(np.linalg.norm(rel - along * gdir))
                on_line = max(0.0, 1.0 - lateral / DEFENSE_LANE_HALF)
                ahead = max(0.0, 1.0 - along / d_og)          # 0 once at/behind the goal line
                depth = min(1.0, -float(s1.ball_pos[0]) / HALF_W)  # deeper threat → more reward
                terms.defense_position = config.w_defense_position * on_line * ahead * depth

    if info.get("goal_scored", False):
        terms.goal = config.w_goal

    if info.get("goal_against", False):
        terms.goal_against = config.w_goal_against

    # Look-ahead "inevitable goal": the env rolled the kicked ball forward and it scores → pay
    # goal-level credit now, at the kick (see env step / predict_goal_by_rollout).
    if info.get("predicted_goal", False):
        terms.predicted_goal = config.w_predicted_goal
        # Quick-release bonus: the same scoring shot, paid extra when fired FAST after engaging the
        # ball (low dwell). predicted_goal already covers banks (rollout bounces) and avoids the
        # defender (interception check), so this rewards the snatch-and-immediately-bank finish.
        quickness = max(0.0, 1.0 - float(info.get("dwell_steps", 0.0)) / QUICK_SHOT_WINDOW)
        terms.quick_shot = config.w_quick_shot * quickness

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
            # Square the goal-aim cosine so reward concentrates on shots aimed squarely at the goal
            # centre — a shot clipping the mouth edge pays much less than a dead-centre strike.
            terms.kick_power_to_goal = config.w_kick_power_to_goal * max(0.0, cos_to_goal) ** 2

    # Continuous aiming gradient (one-time per kick → non-farmable): NOT gated on the shot already
    # entering the mouth, so a wide shot earns partial reward that grows (cos²) as the aim nears the
    # goal centre. This is the smooth "aim closer" signal the gated rewards above can't provide (they
    # pay a miss zero), so it pulls off-target shots toward on-target. Attacking half only; not a
    # shot straight into the opponent.
    if info.get("kicked", False) and not kick_at_opponent and not ball_out_raw \
            and float(s1.ball_pos[0]) > 0.0:
        speed_k = float(np.linalg.norm(s1.ball_vel))
        to_goal_k = OPP_GOAL - s1.ball_pos
        n_k = float(np.linalg.norm(to_goal_k))
        if speed_k > 1e-6 and n_k > 1e-6:
            cos_k = float(np.dot(s1.ball_vel / speed_k, to_goal_k / n_k))
            # cos³ (sharper than cos²): concentrates the gradient on well-aimed shots and pays a
            # wild, near-sideways kick almost nothing — trims the wasted OOB shots run#12 still took.
            terms.kick_aim = config.w_kick_aim * max(0.0, cos_k) ** 3
        # Shot-ANGLE quality (one-time, non-farmable): how WIDE the goal mouth looks from the ball —
        # large from a central position, tiny from the corner. Squared for a sharp central preference.
        # Teaches the robot to carry the ball central before shooting instead of funnelling wide.
        p1 = np.array([HALF_W, GOAL_HALF_WIDTH]); p2 = np.array([HALF_W, -GOAL_HALF_WIDTH])
        v1 = p1 - s1.ball_pos; v2 = p2 - s1.ball_pos
        nv1 = float(np.linalg.norm(v1)); nv2 = float(np.linalg.norm(v2))
        if nv1 > 1e-6 and nv2 > 1e-6:
            width = float(np.arccos(np.clip(np.dot(v1, v2) / (nv1 * nv2), -1.0, 1.0)))
            terms.kick_angle = config.w_kick_angle * min(1.0, width) ** 2

    # Anti-spray: a kick fired in the ATTACKING half whose shot is not goal-bound is a wasted shot —
    # penalize it. Gated to the attacking half so defensive-half clearances aren't punished. This is
    # the direct counter to the 16-kicks/ep, 7%-on-target self-play spray.
    if info.get("kicked", False) and not ball_out_raw and float(s1.ball_pos[0]) > 0.0 \
            and not _shot_enters_goal_mouth(s1.ball_pos, s1.ball_vel):
        terms.wasted_kick = config.w_wasted_kick

    # Anti-corner-camp: the ball deep in the attacking corner (past the mouth laterally, hard against
    # a side wall) has no scoring angle — a dead spot the robot gets stuck dribbling into. Penalize it
    # per step (only while the robot is right on the ball, i.e. it's the one keeping it there) so it
    # pulls the ball back to a shootable position instead.
    if not ball_out_raw and float(s1.ball_pos[0]) > 0.55 \
            and abs(float(s1.ball_pos[1])) > GOAL_HALF_WIDTH + 0.08 \
            and abs(float(s1.ball_pos[1])) > HALF_H - 0.18 \
            and d_robot_ball_1 < CAPTURE_RADIUS + 0.06:
        terms.corner_camp = config.w_corner_camp

    # Quick-shot shaping: reward a fast, *free* ball that is genuinely ON TARGET — its current
    # trajectory enters the goal mouth before crossing any white line (``_shot_enters_goal_mouth``,
    # the same accuracy gate kick_power_to_goal uses). Also gated on the ball being out of the
    # robot's capture radius (so a fast dribble doesn't count) and above a speed threshold (so only
    # a struck shot counts). WITHOUT the on-target gate this paid 2.5 x goalward-velocity every step
    # for ANY fast goal-ward ball, so the policy learned to blast hard roughly-goalward shots that
    # farm this term (it dominated the reward) yet sail wide/out at game range — the gate makes
    # accuracy a precondition, so only shots that will actually score earn it.
    ball_speed = float(np.linalg.norm(s1.ball_vel))
    if ball_speed > SHOT_SPEED_THRESHOLD and d_robot_ball_1 > CAPTURE_RADIUS and not ball_out_raw \
            and _shot_enters_goal_mouth(s1.ball_pos, s1.ball_vel):
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
