import os
import re
from .robot_config import RobotConfig, IRConfig, CompassConfig, SonarConfig, MotorConfig


def _read(path: str) -> str:
    with open(path, "r") as f:
        return f.read()


def _search(pattern: str, text: str, default=None):
    m = re.search(pattern, text)
    return m.group(1) if m else default


def _parse_ir(src_root: str) -> IRConfig:
    text = _read(os.path.join(src_root, "sensors", "IRSensor.h"))
    mode = _search(r"IR_MODE\s*=\s*IRBallMode::(\w+)", text, "MODE_A")
    cfg = IRConfig(
        mode=mode,
        mux_channels=int(_search(r"IR_MUX_CHANNELS\s*=\s*(\d+)", text, "16")),
        board1_enabled=_search(r"IR_BOARD1_ENABLED\s*=\s*(true|false)", text, "true") == "true",
        board2_enabled=_search(r"IR_BOARD2_ENABLED\s*=\s*(true|false)", text, "true") == "true",
        board1_sensor_count=int(_search(r"IR_BOARD1_SENSOR_COUNT\s*=\s*(\d+)", text, "16")),
        board2_sensor_count=int(_search(r"IR_BOARD2_SENSOR_COUNT\s*=\s*(\d+)", text, "16")),
        sweeps_per_cycle=1 if mode == "MODE_D" else 8,
    )
    return cfg


def _parse_compass(src_root: str) -> CompassConfig:
    text = _read(os.path.join(src_root, "sensors", "pos", "Compass.h"))
    return CompassConfig(
        kp=float(_search(r"float\s+kp\s*=\s*([0-9.]+)f?", text, "0.4")),
        kd=float(_search(r"float\s+kd\s*=\s*([0-9.]+)f?", text, "0.3")),
        max_rotation=float(_search(r"float\s+maxRotation\s*=\s*([0-9.]+)f?", text, "25.0")),
        deadzone=float(_search(r"float\s+deadzone\s*=\s*([0-9.]+)f?", text, "3.0")),
    )


def _parse_sonar(src_root: str) -> SonarConfig:
    text = _read(os.path.join(src_root, "sensors", "pos", "Sonar.h"))
    return SonarConfig(
        count=int(_search(r"SONAR_COUNT\s*=\s*(\d+)", text, "4")),
        timeout_us=int(_search(r"SONAR_TIMEOUT_US\s*=\s*(\d+)", text, "20000")),
    )


def _parse_motor(src_root: str) -> MotorConfig:
    hdr = _read(os.path.join(src_root, "motor", "MotorDriver.h"))
    src = _read(os.path.join(src_root, "motor", "MotorDriver.cpp"))

    pwm_res = int(_search(r"#define\s+PWM_RESOLUTION\s+(\d+)", hdr, "3399"))
    min_spd = int(_search(r"#define\s+MIN_SPEED\s+(\d+)", hdr, "1500"))
    # MAX_SPEED is defined as PWM_RESOLUTION in the header
    max_spd = pwm_res

    return MotorConfig(
        pwm_resolution=pwm_res,
        min_speed=min_spd,
        max_speed=max_spd,
        sin_60=float(_search(r"SIN_60\s*=\s*([0-9.]+)f?", src, "0.8660254037844")),
        time_per_100=float(_search(r"timePer100\s*=\s*([0-9.]+)", src, "30000")),
    )


# Files to watch for hot-reload (relative to src_root)
WATCHED_FILES = [
    os.path.join("sensors", "IRSensor.h"),
    os.path.join("sensors", "pos", "Compass.h"),
    os.path.join("sensors", "pos", "Sonar.h"),
    os.path.join("motor", "MotorDriver.h"),
    os.path.join("motor", "MotorDriver.cpp"),
]


def parse_all(src_root: str) -> RobotConfig:
    return RobotConfig(
        ir=_parse_ir(src_root),
        compass=_parse_compass(src_root),
        sonar=_parse_sonar(src_root),
        motor=_parse_motor(src_root),
    )


def get_mtimes(src_root: str) -> dict[str, float]:
    mtimes = {}
    for rel in WATCHED_FILES:
        path = os.path.join(src_root, rel)
        try:
            mtimes[rel] = os.path.getmtime(path)
        except OSError:
            mtimes[rel] = 0.0
    return mtimes


def has_changed(src_root: str, old_mtimes: dict[str, float]) -> bool:
    for rel in WATCHED_FILES:
        path = os.path.join(src_root, rel)
        try:
            if os.path.getmtime(path) != old_mtimes.get(rel, 0.0):
                return True
        except OSError:
            pass
    return False
