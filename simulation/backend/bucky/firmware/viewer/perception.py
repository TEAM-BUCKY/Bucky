"""Left panel: the robot as the firmware sees it (robot-centred, front = up).

* IR ring: the 12 IR sensors, coloured by what GPort::readIR() returns right now;
* line ring: the 16 line sensors, by the white-balanced green-minus-dark reflection the firmware
  reads (GPort::readLine);
* sonar: the distances the simulated sensors are reporting;
* wheels: the PWM duty the firmware drives each motor with, and the resulting body motion;
* compass: Compass::getHeading()/getOffset();
* the ball where the running program says it is (serial probes, e.g. testIRPositioning).
"""
from __future__ import annotations

import math

import numpy as np
import pygame

from bucky.firmware.kinematics import WHEEL_DIRS
from bucky.firmware.viewer import colors as C
from bucky.firmware.viewer.layout import PERCEPT_CX, PERCEPT_CY, PERCEPT_SCALE, body_to_screen
from bucky.game.field import ROBOT_RADIUS

ROBOT_R_CM = ROBOT_RADIUS * 100


def _text(screen, font, s, pos, color=C.TEXT_COLOR) -> None:
    screen.blit(font.render(s, True, color), pos)


def _polar(angle_cw_rad: float, r_cm: float) -> tuple[float, float]:
    return body_to_screen(math.sin(angle_cw_rad) * r_cm, math.cos(angle_cw_rad) * r_cm)


def draw_perception(screen, font, world, perceived) -> None:
    fw = world.fw
    g = fw.globals
    pygame.draw.circle(screen, C.ROBOT_BODY, (round(PERCEPT_CX), round(PERCEPT_CY)),
                       round(ROBOT_R_CM * PERCEPT_SCALE), 2)
    pygame.draw.line(screen, C.ROBOT_HEADING, body_to_screen(0, 0),
                     body_to_screen(0, ROBOT_R_CM), 1)

    # IR ring (what the firmware's IR port holds right now).
    ir = g.irPort().readIR()
    if ir is not None:
        amp = np.clip((np.asarray(ir[:12], float) - world.cfg.ir.baseline) / 600.0, 0, 1)
        for k in range(12):
            x, y = _polar(math.radians(30 * k), ROBOT_R_CM + 4)
            pygame.draw.circle(screen, C.lerp(C.IR_OFF, C.IR_MAX, amp[k]), (round(x), round(y)), 6)

    # Line ring.
    frame = g.linePort().readLine()
    if frame is not None:
        v = np.asarray(frame.value, float)
        refl = v[1] - v[3]     # green minus ambient
        lo, hi = world.cfg.line.green[1], world.cfg.line.white[1]
        for k in range(16):
            t = (refl[k] - lo) / max(1.0, hi - lo)
            x, y = _polar(math.radians(22.5 * k), world.cfg.line.ring_radius_cm)
            pygame.draw.circle(screen, C.lerp(C.FIELD_GREEN, C.FIELD_LINE, t),
                               (round(x), round(y)), 4)

    if world.last is not None:
        # Sonar.
        for a_deg, d in zip(world.cfg.sonar.angles_cw_deg, world.last.readings.get("sonar_cm", [])):
            a = math.radians(a_deg)
            if math.isfinite(d):
                p0 = _polar(a, ROBOT_R_CM)
                p1 = _polar(a, ROBOT_R_CM + min(d, 25.0))     # length capped, value labelled
                pygame.draw.line(screen, C.SONAR_RAY, p0, p1, 1)
                _text(screen, font, f"{d:.0f}", p1, C.SONAR_RAY)

        # Wheels: the duty the firmware writes (inA - inB, label) and where that pushes the
        # wheel's rim given the board's motor wiring (arrow along the wheel's drive direction).
        duty = world.last.actuators.duty
        rim = world.cfg.motors.rim_speeds(duty) / world.cfg.motors.max_rim_mps
        for i, wheel_deg in enumerate((60, 180, 300)):
            cx, cy = _polar(math.radians(wheel_deg), ROBOT_R_CM * 0.75)
            tx, ty = WHEEL_DIRS[i]
            end = (cx + tx * rim[i] * 40, cy - ty * rim[i] * 40)
            drive = duty[i][0] - duty[i][1]
            pygame.draw.line(screen, C.MOTOR_FWD if rim[i] >= 0 else C.MOTOR_REV, (cx, cy), end, 4)
            _text(screen, font, f"M{i + 1} {drive * 100:+.0f}%", (cx + 8, cy + 6), C.TEXT_DIM)

        vx, vy, w = world.last.twist
        end = body_to_screen(vx * 20, vy * 20)
        pygame.draw.line(screen, C.TWIST_ARROW, body_to_screen(0, 0), end, 2)
        _text(screen, font, f"twist {vx:+.2f},{vy:+.2f} m/s  ω {math.degrees(w):+.0f}°/s CW",
              (12, 58), C.TWIST_ARROW)
        if world.last.actuators.kick > 0.05:
            _text(screen, font, f"KICK {world.last.actuators.kick:.0%}", (12, 76), C.BALL_COLOR)

    # Compass.
    c = g.compass()
    if c.isReady():
        _text(screen, font,
              f"compass heading {c.getHeading():6.1f}°  offset {c.getOffset():+6.1f}°",
              (12, 40), C.COMPASS_ARC)
        north = -math.radians(c.getOffset())
        pygame.draw.line(screen, C.COMPASS_ARC, body_to_screen(0, 0),
                         _polar(north, ROBOT_R_CM * 0.6), 1)
    elif c.isFailed():
        _text(screen, font, "compass FAILED", (12, 40), C.BANNER)

    # Ball as the program reports it.
    if perceived.ball is not None:
        bearing, rng = perceived.ball
        x, y = _polar(math.radians(bearing), min(rng, 30.0))
        pygame.draw.circle(screen, C.PERCEIVED_BALL, (round(x), round(y)), 7, 2)
        _text(screen, font, f"ball {bearing:+.0f}° {rng:.0f} cm", (x + 9, y - 6), C.PERCEIVED_BALL)
    y0 = 94
    for k, v in perceived.values.items():
        _text(screen, font, f"{k}: {v}", (12, y0), C.TEXT_DIM)
        y0 += 16
