"""Simulate sensor readings from ground-truth world state."""

import math
from dataclasses import dataclass, field
from simulation.world_state import (
    WorldState, FIELD_WIDTH, FIELD_LENGTH, ROBOT_RADIUS,
    SONAR_ANGLES, ir_sensor_angles,
)
from config.robot_config import RobotConfig

IR_BEAM_HALF_ANGLE = 30.0  # degrees, half-width of IR sensor sensitivity cone
IR_REF_DIST = 80.0         # cm, reference distance for inverse-square falloff
IR_MAX_ADC = 4095           # 12-bit ADC max

# MODE_A sweep thresholds (fraction of full intensity that triggers detection)
MODE_A_THRESHOLDS = [1.0, 0.25, 0.0625, 0.015625, 1.0, 0.25, 0.0625, 0.015625]


@dataclass
class SensorReadings:
    ir_intensities: list[float] = field(default_factory=list)  # per-sensor normalized 0-1
    sonar_distances: list[float] = field(default_factory=list)  # cm per sensor
    compass_heading: float = 0.0
    compass_offset: float = 0.0


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle difference in degrees."""
    d = (a - b) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def _simulate_ir(world: WorldState, config: RobotConfig) -> list[float]:
    """Return per-sensor normalized intensity (0.0 to 1.0)."""
    total = config.ir.total_sensor_count
    if total == 0:
        return []

    angles = ir_sensor_angles(total)
    dx = world.ball_x - world.robot_x
    dy = world.ball_y - world.robot_y
    dist = math.sqrt(dx * dx + dy * dy)

    # Ball angle in world frame (0 = +Y, CW positive)
    ball_angle_world = math.degrees(math.atan2(dx, dy)) % 360.0
    # Convert to robot-local frame
    ball_angle_local = (ball_angle_world - world.robot_heading) % 360.0

    # Distance attenuation (inverse-square with reference distance)
    dist_atten = IR_REF_DIST ** 2 / (IR_REF_DIST ** 2 + dist ** 2) if dist > 0 else 1.0

    intensities = []
    for sensor_angle in angles:
        # Angular difference between sensor pointing direction and ball direction
        ang_diff = abs(_angle_diff(sensor_angle, ball_angle_local))
        # Cosine-based beam pattern: full at 0 degrees, zero at beam_half_angle
        if ang_diff >= IR_BEAM_HALF_ANGLE:
            intensities.append(0.0)
        else:
            beam_factor = math.cos(ang_diff / IR_BEAM_HALF_ANGLE * (math.pi / 2))
            intensities.append(beam_factor * dist_atten)

    return intensities


def _ray_intersect_rect(ox: float, oy: float, dx: float, dy: float,
                        half_w: float, half_h: float) -> float:
    """Distance from (ox, oy) along direction (dx, dy) to axis-aligned rect boundary.
    Returns large value if no intersection."""
    t_min = float("inf")

    # Check each of 4 walls
    for wall_val, axis in [(-half_w, "x"), (half_w, "x"), (-half_h, "y"), (half_h, "y")]:
        if axis == "x" and dx != 0:
            t = (wall_val - ox) / dx
            if t > 0:
                hit_y = oy + dy * t
                if -half_h <= hit_y <= half_h:
                    t_min = min(t_min, t)
        elif axis == "y" and dy != 0:
            t = (wall_val - oy) / dy
            if t > 0:
                hit_x = ox + dx * t
                if -half_w <= hit_x <= half_w:
                    t_min = min(t_min, t)

    return t_min


def _simulate_sonar(world: WorldState, config: RobotConfig) -> list[float]:
    """Return distance in cm for each sonar sensor."""
    hw, hl = FIELD_WIDTH / 2, FIELD_LENGTH / 2
    distances = []

    for i in range(config.sonar.count):
        angle = SONAR_ANGLES[i] if i < len(SONAR_ANGLES) else i * 90.0
        # Direction in world frame
        a_rad = math.radians(world.robot_heading + angle)
        dx = math.sin(a_rad)
        dy = math.cos(a_rad)

        dist = _ray_intersect_rect(world.robot_x, world.robot_y, dx, dy, hw, hl)
        # Subtract robot radius (sonar is on the perimeter)
        dist = max(0.0, dist - ROBOT_RADIUS)
        # Clamp to max range
        dist = min(dist, config.sonar.max_range_cm)
        distances.append(dist)

    return distances


def simulate_sensors(world: WorldState, config: RobotConfig) -> SensorReadings:
    return SensorReadings(
        ir_intensities=_simulate_ir(world, config),
        sonar_distances=_simulate_sonar(world, config),
        compass_heading=world.robot_heading,
        compass_offset=world.compass_offset(),
    )
