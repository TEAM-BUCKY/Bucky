"""Colour palette (from the old tools/simulator)."""

BG_RIGHT = (50, 50, 50)
BG_LEFT = (30, 30, 30)
DIVIDER = (80, 80, 80)
HUD_BG = (20, 20, 20)

FIELD_GREEN = (34, 120, 34)
BAND_GREEN = (28, 96, 28)
FIELD_LINE = (255, 255, 255)
WALL = (15, 15, 15)
GOAL_YELLOW = (255, 220, 0)
GOAL_BLUE = (0, 100, 255)

ROBOT_BODY = (0, 180, 180)
ROBOT_HEADING = (255, 255, 255)
ROBOT_WHEEL = (60, 60, 60)
BALL_COLOR = (255, 100, 0)
BALL_OUTLINE = (200, 60, 0)

SONAR_RAY = (100, 200, 255)
IR_OFF = (40, 40, 40)
IR_MAX = (255, 0, 0)

MOTOR_FWD = (80, 200, 80)
MOTOR_REV = (200, 80, 80)
TWIST_ARROW = (0, 255, 100)
PERCEIVED_BALL = (255, 140, 0)
COMPASS_ARC = (255, 255, 100)
TEXT_COLOR = (200, 200, 200)
TEXT_DIM = (120, 120, 120)
BANNER = (200, 40, 40)


def lerp(a, b, t: float) -> tuple[int, int, int]:
    t = max(0.0, min(1.0, t))
    return tuple(int(x + (y - x) * t) for x, y in zip(a, b))
