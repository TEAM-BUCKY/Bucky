"""Screen layout. Both panels draw in the firmware frame (cm, x right, y towards the opponent
goal = up on screen); the right panel is the field, the left one is robot-centred."""
from __future__ import annotations

from bucky.game.field import ARENA_HALF_X, ARENA_HALF_Y

WINDOW_W = 1280
WINDOW_H = 800
PANEL_W = WINDOW_W // 2
HUD_H = 30
SERIAL_H = 150
VIEW_H = WINDOW_H - HUD_H - SERIAL_H

ARENA_W_CM = 2 * ARENA_HALF_Y * 100     # lateral (screen x)
ARENA_L_CM = 2 * ARENA_HALF_X * 100     # goal to goal (screen y)

MARGIN = 30
SCALE = min((PANEL_W - 2 * MARGIN) / ARENA_W_CM, (VIEW_H - 2 * MARGIN) / ARENA_L_CM)  # px/cm
FIELD_CX = PANEL_W + PANEL_W / 2
FIELD_CY = VIEW_H / 2

PERCEPT_CX = PANEL_W / 2
PERCEPT_CY = VIEW_H / 2
PERCEPT_SCALE = 7.0   # px/cm, zoomed in on the robot


def field_to_screen(x_cm: float, y_cm: float) -> tuple[float, float]:
    return FIELD_CX + x_cm * SCALE, FIELD_CY - y_cm * SCALE


def screen_to_field(sx: float, sy: float) -> tuple[float, float]:
    return (sx - FIELD_CX) / SCALE, (FIELD_CY - sy) / SCALE


def body_to_screen(x_cm: float, y_cm: float) -> tuple[float, float]:
    """Robot body frame (cm, x right, y front) → left panel (front = up)."""
    return PERCEPT_CX + x_cm * PERCEPT_SCALE, PERCEPT_CY - y_cm * PERCEPT_SCALE


def in_field_panel(sx: float, sy: float) -> bool:
    return sx >= PANEL_W and sy < VIEW_H
