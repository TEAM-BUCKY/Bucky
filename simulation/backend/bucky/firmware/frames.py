"""Sim frame ⇄ firmware frame.

* **Sim frame** (physics, :mod:`bucky.lab.frames`): metres, origin at the centre spot, +x towards
  the goal robot A attacks, +y to its left; heading 0 = facing +x, counter-clockwise positive.
  Sim *body* frame: +x = robot front, +y = robot left.
* **Firmware frame** (``robot/README.md``, ``strategy/EKF.h``): centimetres, x = right,
  y = forward / towards the opponent goal; angles clockwise positive from +y, so a direction at
  angle a is (sin a, cos a). Firmware *body* frame: x = robot right, y = robot front.

    fw_field = (-y_sim, x_sim) * 100        theta_fw = -heading_sim
    fw_body  = (-y_body, x_body)            omega_cw = -omega_ccw
"""
from __future__ import annotations

import math

import numpy as np

CM_PER_M = 100.0


def sim_to_fw_pos(p_m) -> np.ndarray:
    """Sim position/vector (m) → firmware field frame (cm), field-centred."""
    return np.array([-float(p_m[1]), float(p_m[0])]) * CM_PER_M


def fw_to_sim_pos(p_cm) -> np.ndarray:
    return np.array([float(p_cm[1]), -float(p_cm[0])]) / CM_PER_M


def sim_to_fw_heading(h_sim: float) -> float:
    """Sim heading (rad, CCW from +x) → firmware heading (rad, CW from +y), wrapped to (-π, π]."""
    return -math.remainder(h_sim, math.tau)


def fw_to_sim_heading(theta_fw: float) -> float:
    return -math.remainder(theta_fw, math.tau)


def sim_body_to_fw_body(v) -> np.ndarray:
    """(forward, left) → (right, forward)."""
    return np.array([-float(v[1]), float(v[0])])


def fw_body_to_sim_body(v) -> np.ndarray:
    """(right, forward) → (forward, left)."""
    return np.array([float(v[1]), -float(v[0])])


def world_to_sim_body(v, heading: float) -> np.ndarray:
    """Rotate a sim-frame vector into the sim body frame."""
    c, s = math.cos(heading), math.sin(heading)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])


def sim_body_to_world(v, heading: float) -> np.ndarray:
    c, s = math.cos(heading), math.sin(heading)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def world_to_fw_body(v_world, heading: float) -> np.ndarray:
    """Sim-frame vector → firmware body frame (right, forward), same units."""
    return sim_body_to_fw_body(world_to_sim_body(v_world, heading))


def fw_bearing(target_m, robot_m, heading_sim: float) -> tuple[float, float]:
    """Bearing (rad, robot frame, 0 = front, clockwise positive) and distance (cm) of a point —
    the convention of the IR code and ``EKF::updateBall``."""
    rel = world_to_fw_body(np.asarray(target_m, float) - np.asarray(robot_m, float), heading_sim)
    return math.atan2(rel[0], rel[1]), float(math.hypot(rel[0], rel[1])) * CM_PER_M
