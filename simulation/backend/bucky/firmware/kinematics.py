"""The firmware's omni-wheel geometry, and motor PWM → robot motion.

``Motors::wheelSpeeds`` (robot/src/hardware/motor/Motors.cpp) drives wheel i at

    u_i = T_i · (sin a, cos a) · scale + rotation

with T_1 = (½, -√3/2), T_2 = (-1, 0), T_3 = (½, √3/2) in the firmware body frame (x right,
y front) — the drive directions of wheels M1/M2/M3 mounted at 60°/180°/300° clockwise. Read as a
rigid-body model, u_i is the rim speed of wheel i along T_i, and the shared rotation term turns
the robot (clockwise positive, the firmware's angle convention):

    u = M · (vx, vy, d·ω_cw),   M = [[T_ix, T_iy, 1]]

The motor model maps the H-bridge duty the firmware writes (inA forward, inB reverse;
``|speed| · scale + MIN_SPEED`` counts out of ``MAX_SPEED + 1``) back to a rim speed.
Assumptions that only the real robot can confirm are marked UNVERIFIED.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bucky.physics.python_backend import MAX_LINEAR, MAX_OMEGA

SIN60 = math.sqrt(3.0) / 2.0
#: Wheel drive directions, firmware body frame (x right, y front).
WHEEL_DIRS = np.array([[0.5, -SIN60], [-1.0, 0.0], [0.5, SIN60]])
#: Wheel mounting radius (m), as in bucky.physics.python_backend.
WHEEL_BASE_M = MAX_LINEAR / MAX_OMEGA
#: rim speeds = M @ (vx_right, vy_front, d·ω_cw)
M = np.column_stack([WHEEL_DIRS, np.ones(3)])
M_INV = np.linalg.inv(M)

MIN_SPEED = 1300   # Motors.h
MAX_SPEED = 3399
PWM_PERIOD = MAX_SPEED + 1


def wheel_speeds(heading_rad: float, scale: float, rotation: float) -> np.ndarray:
    """Python mirror of ``Motors::wheelSpeeds`` (percent, before clamping)."""
    d = np.array([math.sin(heading_rad), math.cos(heading_rad)])
    return WHEEL_DIRS @ d * scale + rotation


def wheel_rim_speeds(v_fw, omega_cw: float, wheel_base_m: float = WHEEL_BASE_M) -> np.ndarray:
    """Body twist (firmware body frame, m/s; rad/s clockwise) → rim speed of each wheel (m/s)."""
    return M @ np.array([float(v_fw[0]), float(v_fw[1]), wheel_base_m * omega_cw])


def body_twist_from_wheels(rim, wheel_base_m: float = WHEEL_BASE_M) -> tuple[float, float, float]:
    """Rim speeds (m/s) → (vx_right, vy_front, ω_cw) in the firmware body frame."""
    vx, vy, r = M_INV @ np.asarray(rim, dtype=float)
    return float(vx), float(vy), float(r / wheel_base_m)


#: M1 and M3 are wired reversed on the robot: test_calibrate.cpp ("M1 and M3 direction pins are
#: physically flipped on this board") negates their command, test_hold_heading.cpp drives
#: (-r, r, -r) for a pure rotation, and the calibration's encoder sign check expects M1 < 0 and
#: M3 > 0 for a +M1 / -M3 command.
BOARD_MOTOR_POLARITY = (-1.0, 1.0, -1.0)


@dataclass(frozen=True)
class MotorModel:
    """H-bridge duty → rim speed (kinematic sense, see module doc).

    ``polarity`` is the board's wiring (default :data:`BOARD_MOTOR_POLARITY`; use (1, 1, 1) for
    a robot whose motors match ``Motors::wheelSpeeds``). UNVERIFIED: the motor is assumed to start
    turning exactly at the firmware's MIN_SPEED duty and to reach ``max_rim_mps`` at full duty,
    linearly in between."""

    deadband_duty: float = MIN_SPEED / PWM_PERIOD
    max_rim_mps: float = MAX_LINEAR
    polarity: tuple[float, float, float] = BOARD_MOTOR_POLARITY

    def rim_speeds(self, duty) -> np.ndarray:
        """``duty`` is (3, 2): per motor (inA, inB) duty in 0..1."""
        duty = np.asarray(duty, dtype=float).reshape(3, 2)
        drive = duty[:, 0] - duty[:, 1]
        mag = np.clip((np.abs(drive) - self.deadband_duty) / (1.0 - self.deadband_duty), 0.0, 1.0)
        return np.sign(drive) * mag * self.max_rim_mps * np.asarray(self.polarity)

    def duty_for_percent(self, percent: float) -> float:
        """What the firmware writes for a wheel speed in percent (``Motors::setMotorSpeed``)."""
        if abs(percent) <= 0.05:
            return 0.0
        scale = (MAX_SPEED - MIN_SPEED) / 100.0
        return int(abs(percent) * scale + MIN_SPEED) / PWM_PERIOD


def twist_to_physics_action(vx_right: float, vy_front: float, omega_cw: float,
                            kick: float = 0.0) -> np.ndarray:
    """Firmware-body twist (m/s, rad/s CW) → normalised TwoRobotPhysics action
    ``[v_forward, v_left, ω_ccw, kick]``."""
    return np.array([vy_front / MAX_LINEAR, -vx_right / MAX_LINEAR,
                     float(np.clip(-omega_cw / MAX_OMEGA, -1.0, 1.0)), kick])
