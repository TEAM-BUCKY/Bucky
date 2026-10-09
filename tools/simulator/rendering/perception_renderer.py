"""Left panel: robot-centric perception view."""

import math
import pygame
from simulation.world_state import WorldState, ROBOT_RADIUS, SONAR_ANGLES, ir_sensor_angles
from simulation.sensor_model import SensorReadings
from simulation.firmware_model import PerceivedState
from config.robot_config import RobotConfig
from rendering.colors import (
    ROBOT_BODY, ROBOT_HEADING, ROBOT_WHEEL,
    PERCEIVED_BALL, SONAR_WALL, MOTOR_ARROW, MOTOR_FWD, MOTOR_REV,
    COMPASS_ARC, TEXT_COLOR, TEXT_DIM, ir_intensity_color,
)
from rendering.panel_layout import PERCEPT_CX, PERCEPT_CY, PERCEPT_SCALE, PANEL_W, WINDOW_H, HUD_H


def _px(cm: float) -> float:
    return cm * PERCEPT_SCALE


def draw_perception(screen: pygame.Surface, world: WorldState,
                    readings: SensorReadings, perceived: PerceivedState,
                    config: RobotConfig, font: pygame.font.Font):
    cx, cy = PERCEPT_CX, PERCEPT_CY
    robot_r = max(6, int(_px(ROBOT_RADIUS)))

    # Robot body (always centered, facing up)
    pygame.draw.circle(screen, ROBOT_BODY, (cx, cy), robot_r)

    # Heading indicator (always points up since robot-centric)
    pygame.draw.line(screen, ROBOT_HEADING, (cx, cy),
                     (cx, cy - int(robot_r * 1.3)), 2)

    # Wheel indicators
    for wheel_offset in [0, 120, 240]:
        w_rad = math.radians(wheel_offset)
        wx = cx + math.sin(w_rad) * robot_r * 0.85
        wy = cy - math.cos(w_rad) * robot_r * 0.85
        pygame.draw.circle(screen, ROBOT_WHEEL, (int(wx), int(wy)),
                           max(2, int(_px(1.5))))

    # IR sensor ring
    _draw_ir_ring(screen, readings, config, cx, cy, robot_r)

    # Perceived ball
    if perceived.ball_detected:
        _draw_perceived_ball(screen, perceived, cx, cy, robot_r, font)

    # Sonar wall indicators
    _draw_sonar_walls(screen, perceived, config, cx, cy, font)

    # Motor vector arrow
    _draw_motor_vector(screen, perceived, cx, cy, robot_r)

    # Motor speed bars
    _draw_motor_bars(screen, perceived, font)

    # Compass heading arc
    _draw_compass(screen, perceived, cx, cy, robot_r, font)

    # Text readouts
    _draw_text_overlay(screen, perceived, config, font)


def _draw_ir_ring(screen, readings, config, cx, cy, robot_r):
    total = config.ir.total_sensor_count
    if total == 0:
        return

    angles = ir_sensor_angles(total)
    ring_r = robot_r + 8

    for i, angle in enumerate(angles):
        a_rad = math.radians(angle)
        dx = cx + math.sin(a_rad) * ring_r
        dy = cy - math.cos(a_rad) * ring_r
        intensity = readings.ir_intensities[i] if i < len(readings.ir_intensities) else 0.0
        color = ir_intensity_color(intensity)
        radius = 3 if intensity > 0.1 else 2
        pygame.draw.circle(screen, color, (int(dx), int(dy)), radius)


def _draw_perceived_ball(screen, perceived, cx, cy, robot_r, font):
    a_rad = math.radians(perceived.ball_angle_deg)
    # Map distance to screen distance (closer = shorter line)
    screen_dist = _px(min(perceived.ball_distance, 120))

    bx = cx + math.sin(a_rad) * screen_dist
    by = cy - math.cos(a_rad) * screen_dist

    # Direction line from robot to ball
    pygame.draw.line(screen, PERCEIVED_BALL, (cx, cy), (int(bx), int(by)), 2)

    # Ball circle
    pygame.draw.circle(screen, PERCEIVED_BALL, (int(bx), int(by)), 8)
    pygame.draw.circle(screen, (255, 200, 100), (int(bx), int(by)), 8, 1)

    # Ball angle text near the ball
    txt = font.render(f"{perceived.ball_angle_deg:.0f}\u00b0", True, PERCEIVED_BALL)
    screen.blit(txt, (int(bx) + 12, int(by) - 8))


def _draw_sonar_walls(screen, perceived, config, cx, cy, font):
    for i, dist in enumerate(perceived.sonar_distances):
        if i >= len(SONAR_ANGLES):
            break
        angle = SONAR_ANGLES[i]
        a_rad = math.radians(angle)

        # Wall indicator: a short line segment perpendicular to the sonar direction
        wall_dist = _px(dist)
        wall_cx = cx + math.sin(a_rad) * wall_dist
        wall_cy = cy - math.cos(a_rad) * wall_dist

        # Perpendicular direction
        perp_rad = a_rad + math.pi / 2
        half_len = 15
        x1 = wall_cx + math.cos(perp_rad) * half_len
        y1 = wall_cy + math.sin(perp_rad) * half_len
        x2 = wall_cx - math.cos(perp_rad) * half_len
        y2 = wall_cy - math.sin(perp_rad) * half_len

        pygame.draw.line(screen, SONAR_WALL, (int(x1), int(y1)), (int(x2), int(y2)), 2)

        # Dotted line from robot to wall
        pygame.draw.line(screen, (*SONAR_WALL[:3],), (cx, cy),
                         (int(wall_cx), int(wall_cy)), 1)

        # Distance label
        label = font.render(f"{dist:.0f}", True, SONAR_WALL)
        screen.blit(label, (int(wall_cx) + 4, int(wall_cy) - 14))


def _draw_motor_vector(screen, perceived, cx, cy, robot_r):
    if perceived.drive_scale < 1.0:
        return

    a_rad = math.radians(perceived.drive_angle_deg)
    length = robot_r * 0.5 + perceived.drive_scale * 0.4
    ex = cx + math.sin(a_rad) * length
    ey = cy - math.cos(a_rad) * length

    pygame.draw.line(screen, MOTOR_ARROW, (cx, cy), (int(ex), int(ey)), 3)

    # Arrowhead
    head_len = 8
    head_angle = 0.4
    for sign in [-1, 1]:
        hx = ex - math.sin(a_rad + sign * head_angle) * head_len
        hy = ey + math.cos(a_rad + sign * head_angle) * head_len
        pygame.draw.line(screen, MOTOR_ARROW, (int(ex), int(ey)), (int(hx), int(hy)), 2)


def _draw_motor_bars(screen, perceived, font):
    bar_x = 20
    bar_y_start = PERCEPT_CY + 140
    bar_w = 100
    bar_h = 12
    labels = ["M1", "M2", "M3"]

    for i, (label, speed) in enumerate(zip(labels, perceived.motor_speeds)):
        y = bar_y_start + i * (bar_h + 8)

        # Label
        txt = font.render(f"{label}", True, TEXT_DIM)
        screen.blit(txt, (bar_x, y - 1))

        # Bar background
        bx = bar_x + 28
        pygame.draw.rect(screen, (50, 50, 50), (bx, y, bar_w, bar_h))

        # Center line
        pygame.draw.line(screen, TEXT_DIM, (bx + bar_w // 2, y),
                         (bx + bar_w // 2, y + bar_h), 1)

        # Fill bar from center
        fill_w = int(abs(speed) / 100.0 * (bar_w // 2))
        color = MOTOR_FWD if speed >= 0 else MOTOR_REV
        if speed >= 0:
            pygame.draw.rect(screen, color, (bx + bar_w // 2, y + 1, fill_w, bar_h - 2))
        else:
            pygame.draw.rect(screen, color, (bx + bar_w // 2 - fill_w, y + 1, fill_w, bar_h - 2))

        # Speed value
        val_txt = font.render(f"{speed:+.0f}%", True, TEXT_COLOR)
        screen.blit(val_txt, (bx + bar_w + 6, y - 1))


def _draw_compass(screen, perceived, cx, cy, robot_r, font):
    offset = perceived.heading_offset
    if abs(offset) < 0.5:
        return

    # Draw arc showing heading offset
    arc_r = robot_r + 22
    start_angle = math.pi / 2  # up (0 degrees heading)
    end_angle = start_angle - math.radians(offset)

    if abs(offset) > 1:
        rect = pygame.Rect(cx - arc_r, cy - arc_r, arc_r * 2, arc_r * 2)
        a1, a2 = min(start_angle, end_angle), max(start_angle, end_angle)
        pygame.draw.arc(screen, COMPASS_ARC, rect, a1, a2, 2)

    # Rotation command indicator
    if abs(perceived.rotation_command) > 0.5:
        txt = font.render(f"rot: {perceived.rotation_command:+.1f}\u00b0", True, COMPASS_ARC)
        screen.blit(txt, (cx + arc_r + 8, cy - 8))


def _draw_text_overlay(screen, perceived, config, font):
    x = 20
    y = 30
    line_h = 18

    lines = [
        f"Ball: {'DETECTED' if perceived.ball_detected else 'none'}",
    ]
    if perceived.ball_detected:
        lines.append(f"  Angle: {perceived.ball_angle_deg:.1f}\u00b0")
        lines.append(f"  Dist:  {perceived.ball_distance:.0f} cm")

    lines.append(f"Heading: {perceived.heading_offset:+.1f}\u00b0")
    lines.append(f"Rotation: {perceived.rotation_command:+.1f}\u00b0")
    lines.append(f"Drive: {perceived.drive_angle_deg:.0f}\u00b0 @ {perceived.drive_scale:.0f}%")

    lines.append("")
    lines.append("Sonar:")
    sonar_labels = ["Front", "Right", "Back", "Left"]
    for i, dist in enumerate(perceived.sonar_distances):
        label = sonar_labels[i] if i < len(sonar_labels) else f"S{i}"
        lines.append(f"  {label}: {dist:.0f} cm")

    for i, line in enumerate(lines):
        color = TEXT_COLOR if not line.startswith(" ") else TEXT_DIM
        txt = font.render(line, True, color)
        screen.blit(txt, (x, y + i * line_h))
