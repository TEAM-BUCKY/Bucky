"""Reward computation for Bucky.

All reward terms are individually weighted and individually logged to TensorBoard.
Potential-based shaping: R_shaping = Φ(s) - Φ(s') so moving toward goal gives +reward.
"""
from __future__ import annotations

from dataclasses import dataclass
from dataclasses import fields as dataclass_fields

import numpy as np

from bucky.field import OPP_GOAL
from bucky.physics.backend import PhysicsState

CAPTURE_RADIUS = 0.14
MAX_OMEGA_PENALTY = 3.0
ALIGN_RADIUS = 0.4  # proximity fade-in distance for front_alignment (m)
SHOT_SPEED_THRESHOLD = 1.2  # m/s; above this a free ball counts as a struck shot, not a dribble
                            # (dribbling tops out near MAX_LINEAR=1.0; kicks reach KICK_MAX_SPEED=3.55)


@dataclass
class RewardConfig:
    w_approach: float = 0.5
    w_ball_to_goal: float = 2.5

    w_possession: float = 1
    w_front_align: float = 0.3

    w_goal: float = 20.0
    w_goal_against: float = -20.0

    w_out_of_bounds: float = -10.0        # robot fully out → 30 s suspension (rules §4.9)
    w_lack_of_progress: float = -2.0      # ball stuck between robots (rules §4.6)
    w_defective: float = -10.0            # removed as defective (rules §4.7)
    w_spin: float = -0.2

    # Skilled-play terms (kicker + opponent-aware; see bucky.play_events).
    w_steal: float = 3.0                  # capture ball from enemy, × field-position gradient
    w_blocked_shot: float = 5.0           # block an enemy shot on our goal
    w_kick_goal: float = 6.0              # bonus: goal scored from a kick (vs dribbling it in)
    w_bank_shot: float = 4.0              # bonus: goal scored off a wall bounce
    w_risky_shot: float = 2.0             # kick threaded past the opponent toward goal, × risk factor
    w_kick_lost: float = -6.0             # extra punishment: our kicked ball captured by enemy

    # Dense kick-shaping: reward *firing the kicker* toward the goal, not only kicks that
    # happen to score. Without this the agent learns to dribble (which earns the same goal
    # reward without the risk) and never explores the kicker — see the unused-kicker analysis.
    w_kick_attempt: float = 0.5           # flat bonus for a legal kick aimed goal-ward
    w_kick_power_to_goal: float = 1.5     # × cos(kick heading, ball→goal): reward aiming kicks at goal

    # Dense "quick shot" shaping: reward a fast ball in flight heading at the goal. Unlike
    # ``ball_to_goal`` (which a slow dribble also earns) this only pays for a *struck* shot,
    # nudging the agent toward decisive shots-on-goal over passive ball-shepherding.
    w_shot_on_goal: float = 1.5           # × (ball velocity component toward goal), when ball is fast & free

    w_time: float = -0.003                # heavier than before so stalemates/dithering cost more
    w_action_mag: float = -0.005

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
    ball_to_goal: float = 0.0

    possession: float = 0.0
    front_alignment: float = 0.0

    goal: float = 0.0
    goal_against: float = 0.0

    out_of_bounds: float = 0.0
    lack_of_progress: float = 0.0
    defective: float = 0.0
    spin: float = 0.0

    steal: float = 0.0
    blocked_shot: float = 0.0
    kick_goal: float = 0.0
    bank_shot: float = 0.0
    risky_shot: float = 0.0
    kick_lost: float = 0.0
    kick_attempt: float = 0.0
    kick_power_to_goal: float = 0.0
    shot_on_goal: float = 0.0

    time_penalty: float = 0.0
    action_magnitude: float = 0.0

    @property
    def total(self) -> float:
        return (self.approach + self.ball_to_goal + self.possession +
                self.front_alignment + self.goal + self.goal_against +
                self.out_of_bounds + self.lack_of_progress + self.defective +
                self.spin + self.steal + self.blocked_shot + self.kick_goal +
                self.bank_shot + self.risky_shot + self.kick_lost +
                self.kick_attempt + self.kick_power_to_goal + self.shot_on_goal +
                self.time_penalty + self.action_magnitude)

    def as_dict(self) -> dict[str, float]:
        return {
            "approach": self.approach,
            "ball_to_goal": self.ball_to_goal,
            "possession": self.possession,
            "front_alignment": self.front_alignment,
            "goal": self.goal,
            "goal_against": self.goal_against,
            "out_of_bounds": self.out_of_bounds,
            "lack_of_progress": self.lack_of_progress,
            "defective": self.defective,
            "spin": self.spin,
            "steal": self.steal,
            "blocked_shot": self.blocked_shot,
            "kick_goal": self.kick_goal,
            "bank_shot": self.bank_shot,
            "risky_shot": self.risky_shot,
            "kick_lost": self.kick_lost,
            "kick_attempt": self.kick_attempt,
            "kick_power_to_goal": self.kick_power_to_goal,
            "shot_on_goal": self.shot_on_goal,
            "time_penalty": self.time_penalty,
            "action_magnitude": self.action_magnitude,
        }


def compute_rewards(
    s0: PhysicsState,
    s1: PhysicsState,
    config: RewardConfig,
    info: dict,
    action: np.ndarray | None = None,
) -> RewardTerms:
    terms = RewardTerms()

    d_robot_ball_0 = float(np.linalg.norm(s0.ball_pos - s0.robot_pos))
    d_robot_ball_1 = float(np.linalg.norm(s1.ball_pos - s1.robot_pos))
    terms.approach = config.w_approach * (d_robot_ball_0 - d_robot_ball_1)

    d_ball_goal_0 = float(np.linalg.norm(s0.ball_pos - OPP_GOAL))
    d_ball_goal_1 = float(np.linalg.norm(s1.ball_pos - OPP_GOAL))
    terms.ball_to_goal = config.w_ball_to_goal * (d_ball_goal_0 - d_ball_goal_1)

    if d_robot_ball_1 < CAPTURE_RADIUS:
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        if np.dot(heading_vec, s1.ball_pos - s1.robot_pos) > 0:
            terms.possession = config.w_possession
        else:
            terms.possession = -config.w_possession

    # Reward setting up to catch the ball and drive it at the enemy goal: the
    # robot's front (where the catch zone is) must face the ball so it can catch
    # it, AND the robot must be behind the ball (on the side away from the goal)
    # so that catching and driving forward sends the ball toward the goal.
    # Faded in by proximity.
    to_goal = OPP_GOAL - s1.ball_pos
    to_goal_dist = float(np.linalg.norm(to_goal))
    if to_goal_dist > 1e-6 and d_robot_ball_1 > 1e-6:
        goal_dir = to_goal / to_goal_dist
        approach_dir = (s1.ball_pos - s1.robot_pos) / d_robot_ball_1
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        face_ball = float(np.dot(heading_vec, approach_dir))  # front/catch zone toward ball
        drive_pos = float(np.dot(approach_dir, goal_dir))     # behind ball -> can drive to goal
        align = 0.5 * (face_ball + drive_pos)
        prox = max(0.0, 1.0 - d_robot_ball_1 / ALIGN_RADIUS)
        terms.front_alignment = config.w_front_align * align * prox

    if info.get("goal_scored", False):
        terms.goal = config.w_goal

    if info.get("goal_against", False):
        terms.goal_against = config.w_goal_against

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

    # Dense kicker shaping: a legal kick (``info["kicked"]``) earns a flat attempt bonus plus
    # a term scaled by how well the kick heading points at the opponent goal. Backward / sideways
    # kicks (cos ≤ 0) earn nothing, so this rewards *useful* kicks without rewarding flailing.
    if info.get("kicked", False):
        to_goal = OPP_GOAL - s1.ball_pos
        to_goal_norm = float(np.linalg.norm(to_goal))
        if to_goal_norm > 1e-6:
            goal_dir = to_goal / to_goal_norm
            heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
            cos_to_goal = float(np.dot(heading_vec, goal_dir))
            if cos_to_goal > 0.0:
                terms.kick_attempt = config.w_kick_attempt
                terms.kick_power_to_goal = config.w_kick_power_to_goal * cos_to_goal

    # Quick-shot shaping: reward a fast, *free* ball heading at the goal. Gated on the ball
    # being out of the robot's capture radius (so a fast dribble doesn't count) and above a
    # speed threshold (so only a struck shot counts). Only the goalward velocity component is
    # rewarded; a fast ball going the wrong way earns nothing (clamped at 0).
    ball_speed = float(np.linalg.norm(s1.ball_vel))
    if ball_speed > SHOT_SPEED_THRESHOLD and d_robot_ball_1 > CAPTURE_RADIUS:
        to_goal = OPP_GOAL - s1.ball_pos
        to_goal_norm = float(np.linalg.norm(to_goal))
        if to_goal_norm > 1e-6:
            goal_dir = to_goal / to_goal_norm
            vel_to_goal = float(np.dot(s1.ball_vel, goal_dir))
            terms.shot_on_goal = config.w_shot_on_goal * max(0.0, vel_to_goal)

    excess_spin = max(0.0, abs(s1.robot_omega) - MAX_OMEGA_PENALTY)
    terms.spin = config.w_spin * excess_spin

    terms.time_penalty = config.w_time

    if action is not None:
        # Drive dims only — the kick dim has its own cooldown gating and dedicated
        # rewards, so it shouldn't be doubly penalized by the action-magnitude term.
        terms.action_magnitude = config.w_action_mag * float(np.sum(np.asarray(action[:3])**2))

    return terms
