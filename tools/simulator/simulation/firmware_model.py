"""Replicate C++ firmware logic: PD heading control, motor kinematics, ball detection."""

import math
from dataclasses import dataclass, field
from config.robot_config import RobotConfig
from simulation.sensor_model import SensorReadings
from simulation.world_state import ir_sensor_angles


@dataclass
class PerceivedState:
    ball_angle_deg: float = 0.0
    ball_distance: float = 0.0
    ball_detected: bool = False
    sonar_distances: list[float] = field(default_factory=list)
    heading_offset: float = 0.0
    rotation_command: float = 0.0
    motor_speeds: tuple[float, float, float] = (0.0, 0.0, 0.0)
    drive_angle_deg: float = 0.0
    drive_scale: float = 0.0


class FirmwareModel:
    def __init__(self, config: RobotConfig):
        self.last_error = 0.0
        self.last_time = 0.0
        self.accumulated_time = 0.0

    def reset(self):
        self.last_error = 0.0
        self.last_time = 0.0
        self.accumulated_time = 0.0

    def _compute_rotation(self, offset: float, target_degrees: float,
                          dt: float, config: RobotConfig) -> float:
        """Replicate Compass::computeRotation (Compass.cpp:129-147)."""
        error = offset - target_degrees
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0

        rotation = 0.0
        if abs(error) > config.compass.deadzone:
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0
            rotation = -error * config.compass.kp - derivative * config.compass.kd
            rotation = max(-config.compass.max_rotation,
                           min(config.compass.max_rotation, rotation))
        self.last_error = error
        return rotation

    def _compute_motor_speeds(self, angle_rad: float, scale: float,
                              rotation: float, config: RobotConfig) -> tuple[float, float, float]:
        """Replicate MotorDriver::driveRadians (MotorDriver.cpp:159-177)."""
        sin_60 = config.motor.sin_60
        rotation_scale = max(scale, abs(rotation)) / 100.0
        scaled_rotation = rotation * rotation_scale

        sin_a = math.sin(angle_rad)
        cos_a = math.cos(angle_rad)

        m1 = (0.5 * sin_a - sin_60 * cos_a) * scale + scaled_rotation
        m2 = -sin_a * scale + scaled_rotation
        m3 = (0.5 * sin_a + sin_60 * cos_a) * scale + scaled_rotation

        m1 = max(-100.0, min(100.0, m1))
        m2 = max(-100.0, min(100.0, m2))
        m3 = max(-100.0, min(100.0, m3))

        return m1, m2, m3

    def _detect_ball(self, ir_intensities: list[float], total_sensors: int) -> tuple[float, float, bool]:
        """Weighted-average ball detection from IR sensor ring.
        Returns (angle_deg, distance_estimate, detected)."""
        if not ir_intensities or total_sensors == 0:
            return 0.0, 0.0, False

        angles = ir_sensor_angles(total_sensors)
        sum_x = 0.0
        sum_y = 0.0
        total_intensity = 0.0

        for i, intensity in enumerate(ir_intensities):
            if intensity > 0.01:
                a_rad = math.radians(angles[i])
                sum_x += math.sin(a_rad) * intensity
                sum_y += math.cos(a_rad) * intensity
                total_intensity += intensity

        if total_intensity < 0.05:
            return 0.0, 0.0, False

        ball_angle = math.degrees(math.atan2(sum_x, sum_y))
        if ball_angle < 0:
            ball_angle += 360.0
        if ball_angle >= 359.5:
            ball_angle = 0.0
        # Magnitude of resultant as distance proxy (closer ball = higher intensities = larger magnitude)
        magnitude = math.sqrt(sum_x ** 2 + sum_y ** 2)
        # Normalize: max possible magnitude = total_sensors (all at 1.0 pointing same direction)
        # Map to a rough distance: high magnitude = close, low = far
        max_magnitude = total_sensors * 0.5  # rough upper bound
        distance_est = max(5.0, 150.0 * (1.0 - magnitude / max_magnitude))

        return ball_angle, distance_est, True

    def update(self, readings: SensorReadings, config: RobotConfig,
               dt: float) -> PerceivedState:
        self.accumulated_time += dt

        # Ball detection from IR
        ball_angle, ball_dist, ball_detected = self._detect_ball(
            readings.ir_intensities, config.ir.total_sensor_count
        )

        # Heading PD control (target = 0, maintain start heading)
        rotation = self._compute_rotation(readings.compass_offset, 0.0, dt, config)

        # Compute drive command: drive toward ball if detected
        drive_angle = 0.0
        drive_scale = 0.0
        if ball_detected:
            drive_angle = ball_angle
            # Scale based on distance (closer = slower approach)
            drive_scale = min(80.0, max(20.0, ball_dist * 0.6))

        # Motor kinematics
        drive_rad = math.radians(drive_angle)
        motor_speeds = self._compute_motor_speeds(drive_rad, drive_scale, rotation, config)

        return PerceivedState(
            ball_angle_deg=ball_angle,
            ball_distance=ball_dist,
            ball_detected=ball_detected,
            sonar_distances=list(readings.sonar_distances),
            heading_offset=readings.compass_offset,
            rotation_command=rotation,
            motor_speeds=motor_speeds,
            drive_angle_deg=drive_angle,
            drive_scale=drive_scale,
        )
