"""Domain randomization config and per-episode samplers.

Toggle with DomainRandomConfig(enabled=False) for a clean deterministic env.
Heading drift models the BNO085 Game Rotation Vector: near-perfect short-term,
slow yaw drift (0.1–0.5°/min), mag-free (no magnetic north reference).
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class DomainRandomConfig:
    enabled: bool = True
    bearing_noise_std_deg: float = 3.0
    dist_noise_frac_std: float = 0.15
    max_action_latency_steps: int = 2
    motor_sat_min: float = 0.85
    motor_sat_max: float = 1.0
    friction_min: float = 0.7
    friction_max: float = 1.3
    restitution_min: float = 0.5
    restitution_max: float = 0.9
    mass_min: float = 1.2
    mass_max: float = 1.8
    heading_drift_rate_max: float = 0.0001


@dataclass
class EpisodeRandomization:
    action_latency_steps: int = 0
    motor_saturation: float = 1.0
    ball_damping_scale: float = 1.0
    ball_restitution: float = 0.7
    robot_mass_scale: float = 1.0
    heading_drift_rate: float = 0.0
    bearing_noise_std: float = 0.0
    dist_noise_frac_std: float = 0.0


def sample_episode_randomization(
    config: DomainRandomConfig,
    rng: np.random.Generator,
) -> EpisodeRandomization:
    if not config.enabled:
        return EpisodeRandomization()
    return EpisodeRandomization(
        action_latency_steps=int(rng.integers(0, config.max_action_latency_steps + 1)),
        motor_saturation=float(rng.uniform(config.motor_sat_min, config.motor_sat_max)),
        ball_damping_scale=float(rng.uniform(config.friction_min, config.friction_max)),
        ball_restitution=float(rng.uniform(config.restitution_min, config.restitution_max)),
        robot_mass_scale=float(rng.uniform(config.mass_min, config.mass_max) / 1.5),
        heading_drift_rate=float(rng.uniform(0, config.heading_drift_rate_max)),
        bearing_noise_std=float(abs(rng.normal(0, config.bearing_noise_std_deg)) * np.pi / 180),
        dist_noise_frac_std=float(abs(rng.normal(0, config.dist_noise_frac_std))),
    )
