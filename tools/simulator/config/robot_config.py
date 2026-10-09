from dataclasses import dataclass, field


@dataclass
class IRConfig:
    mode: str = "MODE_A"
    mux_channels: int = 16
    board1_enabled: bool = True
    board2_enabled: bool = True
    board1_sensor_count: int = 16
    board2_sensor_count: int = 16
    sweeps_per_cycle: int = 8

    @property
    def total_sensor_count(self) -> int:
        count = 0
        if self.board1_enabled:
            count += self.board1_sensor_count
        if self.board2_enabled:
            count += self.board2_sensor_count
        return count


@dataclass
class CompassConfig:
    kp: float = 0.4
    kd: float = 0.3
    max_rotation: float = 25.0
    deadzone: float = 3.0


@dataclass
class SonarConfig:
    count: int = 4
    timeout_us: int = 20000

    @property
    def max_range_cm(self) -> float:
        return 0.017 * self.timeout_us


@dataclass
class MotorConfig:
    pwm_resolution: int = 3399
    min_speed: int = 1500
    max_speed: int = 3399
    sin_60: float = 0.8660254037844
    time_per_100: float = 30000.0


@dataclass
class RobotConfig:
    ir: IRConfig = field(default_factory=IRConfig)
    compass: CompassConfig = field(default_factory=CompassConfig)
    sonar: SonarConfig = field(default_factory=SonarConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
