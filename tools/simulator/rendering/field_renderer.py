"""Right panel: draw the actual field, robot, ball, and optional sensor overlays."""

import math
import pygame
from simulation.world_state import (
    WorldState, FIELD_WIDTH, FIELD_LENGTH, GOAL_WIDTH, GOAL_DEPTH,
    ROBOT_RADIUS, BALL_RADIUS, SONAR_ANGLES, ir_sensor_angles,
)
from config.robot_config import RobotConfig
from rendering.colors import (
    FIELD_GREEN, FIELD_LINE, GOAL_YELLOW, GOAL_BLUE,
    ROBOT_BODY, ROBOT_HEADING, ROBOT_WHEEL, BALL_COLOR, BALL_OUTLINE,
    SONAR_RAY, IR_OFF, TEXT_DIM,
)
from rendering.panel_layout import (
    world_to_screen, FIELD_ORIGIN_X, FIELD_ORIGIN_Y, FIELD_SCALE,
)


def _ws(wx, wy):
    return world_to_screen(wx, wy)


def _px(cm):
    return cm * FIELD_SCALE


def draw_field(screen: pygame.Surface, world: WorldState, config: RobotConfig,
               font: pygame.font.Font, show_overlays: bool):
    fw, fl = FIELD_WIDTH, FIELD_LENGTH
    hw, hl = fw / 2, fl / 2

    # Field surface
    fx, fy = _ws(-hw, hl)
    field_rect = pygame.Rect(fx, fy, _px(fw), _px(fl))
    pygame.draw.rect(screen, FIELD_GREEN, field_rect)

    # Field border (white walls)
    pygame.draw.rect(screen, FIELD_LINE, field_rect, 2)

    # Center line
    pygame.draw.line(screen, FIELD_LINE, _ws(-hw, 0), _ws(hw, 0), 1)

    # Center circle
    cx, cy = _ws(0, 0)
    pygame.draw.circle(screen, FIELD_LINE, (int(cx), int(cy)), int(_px(25)), 1)

    # Center dot
    pygame.draw.circle(screen, FIELD_LINE, (int(cx), int(cy)), 3)

    # Goals
    gw = GOAL_WIDTH / 2
    gd = GOAL_DEPTH

    # Yellow goal (top, +Y)
    gy_tl = _ws(-gw, hl + gd)
    gy_br = _ws(gw, hl)
    goal_y_rect = pygame.Rect(gy_tl[0], gy_tl[1], gy_br[0] - gy_tl[0], gy_br[1] - gy_tl[1])
    pygame.draw.rect(screen, GOAL_YELLOW, goal_y_rect, 3)

    # Blue goal (bottom, -Y)
    gb_tl = _ws(-gw, -hl)
    gb_br = _ws(gw, -hl - gd)
    goal_b_rect = pygame.Rect(gb_tl[0], gb_tl[1], gb_br[0] - gb_tl[0], gb_br[1] - gb_tl[1])
    pygame.draw.rect(screen, GOAL_BLUE, goal_b_rect, 3)

    # Neutral spots (4 corners of center area)
    for nx, ny in [(-30, 40), (30, 40), (-30, -40), (30, -40)]:
        sx, sy = _ws(nx, ny)
        pygame.draw.circle(screen, FIELD_LINE, (int(sx), int(sy)), 3)

    # Ball
    bx, by = _ws(world.ball_x, world.ball_y)
    ball_r = max(4, int(_px(BALL_RADIUS)))
    pygame.draw.circle(screen, BALL_COLOR, (int(bx), int(by)), ball_r)
    pygame.draw.circle(screen, BALL_OUTLINE, (int(bx), int(by)), ball_r, 1)

    # Robot
    rx, ry = _ws(world.robot_x, world.robot_y)
    robot_r = max(6, int(_px(ROBOT_RADIUS)))
    pygame.draw.circle(screen, ROBOT_BODY, (int(rx), int(ry)), robot_r)

    # Heading indicator line
    h_rad = math.radians(world.robot_heading)
    hx = rx + math.sin(h_rad) * robot_r * 0.9  # sin because heading 0 = +Y = up on screen is -Y
    hy = ry - math.cos(h_rad) * robot_r * 0.9
    # Actually heading 0 = +Y (toward yellow goal) = screen up
    # In screen coords: +Y screen = down, so heading 0 should point up
    # heading in world: 0 = +Y, CW positive
    # screen: dx = sin(heading), dy = -cos(heading)
    hx = rx + math.sin(h_rad) * robot_r * 1.3
    hy = ry - math.cos(h_rad) * robot_r * 1.3
    pygame.draw.line(screen, ROBOT_HEADING, (int(rx), int(ry)), (int(hx), int(hy)), 2)

    # Wheel indicators (at 120 degree spacing, offset from heading)
    for wheel_offset in [0, 120, 240]:
        w_rad = math.radians(world.robot_heading + wheel_offset)
        wx = rx + math.sin(w_rad) * robot_r * 0.85
        wy = ry - math.cos(w_rad) * robot_r * 0.85
        pygame.draw.circle(screen, ROBOT_WHEEL, (int(wx), int(wy)), max(2, int(_px(1.5))))

    # Optional overlays
    if show_overlays:
        _draw_sonar_rays(screen, world, config, rx, ry, robot_r)
        _draw_ir_dots(screen, world, config, rx, ry, robot_r)

    # Heading text
    htxt = font.render(f"{world.robot_heading:.0f}\u00b0", True, TEXT_DIM)
    screen.blit(htxt, (int(rx) + robot_r + 4, int(ry) - 8))


def _draw_sonar_rays(screen, world, config, rx, ry, robot_r):
    for i, angle in enumerate(SONAR_ANGLES[:config.sonar.count]):
        a_rad = math.radians(world.robot_heading + angle)
        length = _px(min(80, config.sonar.max_range_cm))
        ex = rx + math.sin(a_rad) * length
        ey = ry - math.cos(a_rad) * length
        pygame.draw.line(screen, (*SONAR_RAY, 80), (int(rx), int(ry)), (int(ex), int(ey)), 1)


def _draw_ir_dots(screen, world, config, rx, ry, robot_r):
    angles = ir_sensor_angles(config.ir.total_sensor_count)
    for angle in angles:
        a_rad = math.radians(world.robot_heading + angle)
        dx = rx + math.sin(a_rad) * (robot_r + 3)
        dy = ry - math.cos(a_rad) * (robot_r + 3)
        pygame.draw.circle(screen, IR_OFF, (int(dx), int(dy)), 2)
