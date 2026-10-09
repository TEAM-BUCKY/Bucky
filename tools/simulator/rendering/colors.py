# Color palette for the simulator

# Backgrounds
BG_RIGHT = (50, 50, 50)
BG_LEFT = (30, 30, 30)
DIVIDER = (80, 80, 80)

# Field
FIELD_GREEN = (34, 120, 34)
FIELD_LINE = (255, 255, 255)
GOAL_YELLOW = (255, 220, 0)
GOAL_BLUE = (0, 100, 255)

# Entities
ROBOT_BODY = (0, 180, 180)
ROBOT_HEADING = (255, 255, 255)
ROBOT_WHEEL = (60, 60, 60)
BALL_COLOR = (255, 100, 0)
BALL_OUTLINE = (200, 60, 0)

# Sensors
SONAR_RAY = (100, 200, 255)
SONAR_WALL = (100, 200, 255)
IR_OFF = (40, 40, 40)
IR_MAX = (255, 0, 0)

# Perception panel
MOTOR_ARROW = (0, 255, 100)
MOTOR_FWD = (80, 200, 80)
MOTOR_REV = (200, 80, 80)
PERCEIVED_BALL = (255, 140, 0)
COMPASS_ARC = (255, 255, 100)
TEXT_COLOR = (200, 200, 200)
TEXT_DIM = (120, 120, 120)
HUD_BG = (20, 20, 20)


def ir_intensity_color(intensity: float) -> tuple[int, int, int]:
    """Map 0.0-1.0 intensity to IR_OFF -> IR_MAX gradient."""
    t = max(0.0, min(1.0, intensity))
    return (
        int(IR_OFF[0] + t * (IR_MAX[0] - IR_OFF[0])),
        int(IR_OFF[1] + t * (IR_MAX[1] - IR_OFF[1])),
        int(IR_OFF[2] + t * (IR_MAX[2] - IR_OFF[2])),
    )
