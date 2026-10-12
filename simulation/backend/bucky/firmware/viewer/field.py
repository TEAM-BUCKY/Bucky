"""Right panel: the actual field — physics truth, sonar rays and line sensor points."""
from __future__ import annotations

import math

import numpy as np
import pygame

from bucky.firmware import frames, sensors
from bucky.firmware.viewer import colors as C
from bucky.firmware.viewer.layout import ARENA_L_CM, ARENA_W_CM, SCALE, field_to_screen
from bucky.game.field import BALL_RADIUS, GOAL_HALF_WIDTH, HALF_H, HALF_W, ROBOT_RADIUS

GOAL_DEPTH_CM = 7.4


def _rect(x0, y0, x1, y1) -> pygame.Rect:
    (ax, ay), (bx, by) = field_to_screen(x0, y1), field_to_screen(x1, y0)
    return pygame.Rect(round(ax), round(ay), round(bx - ax), round(by - ay))


def draw_field(screen, world, overlays: bool) -> None:
    pygame.draw.rect(screen, C.BAND_GREEN, _rect(-ARENA_W_CM / 2, -ARENA_L_CM / 2,
                                                 ARENA_W_CM / 2, ARENA_L_CM / 2))
    pygame.draw.rect(screen, C.WALL, _rect(-ARENA_W_CM / 2, -ARENA_L_CM / 2,
                                           ARENA_W_CM / 2, ARENA_L_CM / 2), 3)
    hw, hl = HALF_H * 100, HALF_W * 100       # firmware x = lateral, y = goal axis
    pygame.draw.rect(screen, C.FIELD_GREEN, _rect(-hw, -hl, hw, hl))
    pygame.draw.rect(screen, C.FIELD_LINE, _rect(-hw, -hl, hw, hl), max(1, round(2 * SCALE)))
    gw = GOAL_HALF_WIDTH * 100
    pygame.draw.rect(screen, C.GOAL_YELLOW, _rect(-gw, hl, gw, hl + GOAL_DEPTH_CM), 3)   # attacked
    pygame.draw.rect(screen, C.GOAL_BLUE, _rect(-gw, -hl - GOAL_DEPTH_CM, gw, -hl), 3)  # own
    cx, cy = field_to_screen(0, 0)
    pygame.draw.circle(screen, C.FIELD_LINE, (round(cx), round(cy)), 3)

    s = world.physics.state_a()
    rx_cm, ry_cm = frames.sim_to_fw_pos(s.robot_pos)
    theta = frames.sim_to_fw_heading(s.robot_heading)
    rx, ry = field_to_screen(rx_cm, ry_cm)
    r_px = max(6, round(ROBOT_RADIUS * 100 * SCALE))

    if overlays and world.last is not None:
        _draw_sonar(screen, world, rx_cm, ry_cm, theta)
        _draw_line_points(screen, world, s)

    bx, by = field_to_screen(*frames.sim_to_fw_pos(s.ball_pos))
    b_px = max(4, round(BALL_RADIUS * 100 * SCALE))
    pygame.draw.circle(screen, C.BALL_COLOR, (round(bx), round(by)), b_px)
    pygame.draw.circle(screen, C.BALL_OUTLINE, (round(bx), round(by)), b_px, 1)

    pygame.draw.circle(screen, C.ROBOT_BODY, (round(rx), round(ry)), r_px)
    hx, hy = rx + math.sin(theta) * r_px * 1.3, ry - math.cos(theta) * r_px * 1.3
    pygame.draw.line(screen, C.ROBOT_HEADING, (rx, ry), (hx, hy), 2)
    for wheel_deg in (60, 180, 300):         # M1, M2, M3, clockwise from the front
        a = theta + math.radians(wheel_deg)
        pygame.draw.circle(screen, C.ROBOT_WHEEL,
                           (round(rx + math.sin(a) * r_px * 0.8),
                            round(ry - math.cos(a) * r_px * 0.8)),
                           max(2, round(r_px * 0.15)))


def _draw_sonar(screen, world, rx_cm, ry_cm, theta) -> None:
    dist = world.last.readings.get("sonar_cm")
    if dist is None:
        return
    rim = world.cfg.sonar.mount_radius_cm
    for a_deg, d in zip(world.cfg.sonar.angles_cw_deg, dist):
        a = theta + math.radians(a_deg)
        u = np.array([math.sin(a), math.cos(a)])
        start = np.array([rx_cm, ry_cm]) + u * rim
        end = start + u * (d if math.isfinite(d) else world.cfg.sonar.max_range_cm)
        pygame.draw.line(screen, C.SONAR_RAY, field_to_screen(*start), field_to_screen(*end), 1)


def _draw_line_points(screen, world, s) -> None:
    pts = sensors.line_points(s.robot_pos, s.robot_heading, world.cfg.line)
    hits = sensors.line_hits(s.robot_pos, s.robot_heading, world.cfg.line)
    for p, hit in zip(pts, hits):
        x, y = field_to_screen(*frames.sim_to_fw_pos(p))
        pygame.draw.circle(screen, C.FIELD_LINE if hit else C.TEXT_DIM, (round(x), round(y)), 2)
