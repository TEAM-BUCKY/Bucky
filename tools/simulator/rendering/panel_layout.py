from simulation.world_state import FIELD_WIDTH, FIELD_LENGTH

WINDOW_W = 1280
WINDOW_H = 720
PANEL_W = WINDOW_W // 2
HUD_H = 30

# Right panel (actual field) layout
FIELD_MARGIN = 40
_avail_w = PANEL_W - 2 * FIELD_MARGIN
_avail_h = WINDOW_H - HUD_H - 2 * FIELD_MARGIN
FIELD_SCALE = min(_avail_w / FIELD_WIDTH, _avail_h / FIELD_LENGTH)  # px per cm

# Field rendering origin: top-left of the green rectangle, in screen coords
_field_px_w = FIELD_WIDTH * FIELD_SCALE
_field_px_h = FIELD_LENGTH * FIELD_SCALE
FIELD_ORIGIN_X = PANEL_W + (PANEL_W - _field_px_w) / 2
FIELD_ORIGIN_Y = (WINDOW_H - HUD_H - _field_px_h) / 2

# Left panel center (for robot-centric perception view)
PERCEPT_CX = PANEL_W // 2
PERCEPT_CY = (WINDOW_H - HUD_H) // 2
PERCEPT_SCALE = FIELD_SCALE  # same scale so distances are comparable


def world_to_screen(wx: float, wy: float) -> tuple[float, float]:
    """Convert world coords (cm, origin=field center) to right-panel screen coords.
    +X = right, +Y = toward yellow goal (top of screen)."""
    sx = FIELD_ORIGIN_X + (wx + FIELD_WIDTH / 2) * FIELD_SCALE
    sy = FIELD_ORIGIN_Y + (FIELD_LENGTH / 2 - wy) * FIELD_SCALE
    return sx, sy


def screen_to_world(sx: float, sy: float) -> tuple[float, float]:
    """Inverse of world_to_screen."""
    wx = (sx - FIELD_ORIGIN_X) / FIELD_SCALE - FIELD_WIDTH / 2
    wy = FIELD_LENGTH / 2 - (sy - FIELD_ORIGIN_Y) / FIELD_SCALE
    return wx, wy


def is_in_right_panel(sx: float, sy: float) -> bool:
    return sx >= PANEL_W and sy < WINDOW_H - HUD_H
