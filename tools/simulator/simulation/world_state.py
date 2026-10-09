from dataclasses import dataclass

# RoboCup Junior soccer field dimensions (cm)
FIELD_WIDTH = 182.0    # short axis (X)
FIELD_LENGTH = 243.0   # long axis (Y), goals on Y ends
GOAL_WIDTH = 45.0      # goal opening width
GOAL_DEPTH = 7.4       # goal depth behind wall
ROBOT_RADIUS = 9.0     # typical RCJ robot radius
BALL_RADIUS = 3.65     # IR ball ~7.3cm diameter

# Sonar sensor mounting angles (degrees, 0 = forward, CW positive)
SONAR_ANGLES = [0.0, 90.0, 180.0, 270.0]

# IR sensor ring: uniform distribution around the robot
def ir_sensor_angles(total_count: int) -> list[float]:
    return [i * 360.0 / total_count for i in range(total_count)]


@dataclass
class WorldState:
    # Robot pose (field center = origin)
    robot_x: float = 0.0
    robot_y: float = -50.0    # start in own half
    robot_heading: float = 0.0  # degrees, 0 = toward +Y (yellow goal), CW positive

    # Ball position
    ball_x: float = 0.0
    ball_y: float = 30.0

    # Compass reference (set at "boot")
    start_heading: float = 0.0

    # Simulation state
    paused: bool = False

    def reset(self):
        self.robot_x = 0.0
        self.robot_y = -50.0
        self.robot_heading = 0.0
        self.ball_x = 0.0
        self.ball_y = 30.0
        self.start_heading = self.robot_heading

    def compass_offset(self) -> float:
        diff = self.robot_heading - self.start_heading
        if diff > 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        return diff
