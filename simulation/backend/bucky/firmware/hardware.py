"""The robot's hardware around the simulated MCU: truth in, raw readings to the chips, PWM out.

:class:`SimHardware` is the one place that knows how the sensor models (:mod:`.sensors`) map
onto the simulated board's devices (``bucky_fw.devices``)::

    hw = SimHardware(fw, RobotHardwareConfig(), rng)
    hw.apply_truth(RobotTruth(pos_m=..., heading=..., ball_m=..., ...))   # all sensors at once
    hw.set_heading(math.radians(30))                                       # or one at a time
    act = hw.read_actuators()        # mean motor duty (3, 2) + kicker, since the last call
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import ModuleType

import numpy as np

from bucky.firmware import frames, ir_ball, sensors
from bucky.firmware.kinematics import MotorModel, wheel_rim_speeds


@dataclass(frozen=True)
class RobotHardwareConfig:
    compass: sensors.CompassModel = field(default_factory=sensors.CompassModel)
    accel: sensors.AccelModel = field(default_factory=sensors.AccelModel)
    sonar: sensors.SonarModel = field(default_factory=sensors.SonarModel)
    #: IR ball + ring optics + analog front end (time-domain, see :mod:`.ir_ball`).
    ir: ir_ball.IRRingModel = field(default_factory=ir_ball.IRRingModel)
    line: sensors.LineModel = field(default_factory=sensors.LineModel)
    encoders: sensors.EncoderModel = field(default_factory=sensors.EncoderModel)
    motors: MotorModel = field(default_factory=MotorModel)
    #: UNVERIFIED: the order the line board lights its LEDs in (firmware default R, G, B, Dark).
    line_led_order: tuple[int, int, int, int] = (0, 1, 2, 3)


@dataclass
class RobotTruth:
    """Ground truth for one instant, sim frame (m, rad CCW)."""

    pos_m: np.ndarray
    heading: float
    vel_mps: np.ndarray = field(default_factory=lambda: np.zeros(2))
    omega: float = 0.0
    accel_mps2: np.ndarray = field(default_factory=lambda: np.zeros(2))   # world frame
    ball_m: np.ndarray | None = None
    ball_vel_mps: np.ndarray = field(default_factory=lambda: np.zeros(2))
    ball_visible: bool = True
    robots: tuple = ()                    # other robots' centres (sonar and IR obstacles)


@dataclass
class Actuators:
    duty: np.ndarray        # (3, 2) mean duty of (inA, inB) per motor over the interval
    kick: float             # peak kicker duty over the interval (0..1)


class SimHardware:
    """Drives every simulated sensor of ``fw`` (the loaded ``bucky_fw`` module) from truth."""

    def __init__(self, fw: ModuleType, config: RobotHardwareConfig | None = None,
                 rng: np.random.Generator | None = None) -> None:
        self.fw = fw
        self.rng = rng
        dev = fw.devices
        ir = int(fw.GSensorKind.IR)
        self.ir_board = dev.gport1 if fw.board.G_PORT1_KIND == ir else dev.gport2
        self.cfg = config or RobotHardwareConfig()
        line = int(fw.GSensorKind.Line)
        self.line_board = dev.gport1 if fw.board.G_PORT1_KIND == line else dev.gport2
        self.line_board.led_order = list(self.cfg.line_led_order)
        dev.sonar.latency_us = self.cfg.sonar.latency_us
        # Readings at rest until told otherwise: a robot at heading 0, nothing in sight.
        self.set_heading(0.0)
        self.set_accel((0.0, 0.0))
        self.set_ir(None)

    @property
    def cfg(self) -> RobotHardwareConfig:
        return self._cfg

    @cfg.setter
    def cfg(self, config: RobotHardwareConfig) -> None:
        """Changing the config re-programs the IR board's front end."""
        self._cfg = config
        seed = int(self.rng.integers(2**63)) if self.rng is not None else 0
        config.ir.push(self.ir_board.ir, self.rng, seed)
        self.ir_board.ir.reset()
        self.ir_board.ir_physical = True
        self.ir_scene = ir_ball.IRScene(config.ir, self.rng)

    # ── individual sensors ────────────────────────────────────────────────────────────────
    def set_heading(self, heading_sim: float) -> np.ndarray:
        raw = sensors.compass_raw(heading_sim, self.cfg.compass, self.rng)
        self.fw.devices.compass.field = [int(v) for v in raw]
        return raw

    def set_accel(self, a_body_fw_g) -> np.ndarray:
        raw = sensors.accel_raw(a_body_fw_g, self.cfg.accel, self.rng)
        self.fw.devices.accel.raw = [int(v) for v in raw]
        return raw

    def set_sonar_cm(self, distances_cm) -> None:
        """Distances (cm, NaN = no echo) straight to the echo lines."""
        echo = sensors.sonar_echo_us(distances_cm, self.cfg.sonar)
        self.fw.devices.sonar.echo_us = echo + [math.nan] * (4 - len(echo))

    def set_ir(self, ball_bearing_cw: float | None, dist_cm: float = 100.0) -> np.ndarray:
        """Ball at a robot-frame bearing (rad, CW from front) and distance from the robot centre
        (cm); None = no ball. Robot at the origin, heading 0, nothing in the way. Returns the
        steady-state codes (the board's real samples also droop and reset in time)."""
        if ball_bearing_cw is None:
            return self.set_ir_scene((0.0, 0.0), 0.0, None)
        rel = frames.fw_body_to_sim_body(
            (math.sin(ball_bearing_cw) * dist_cm / 100.0, math.cos(ball_bearing_cw) * dist_cm / 100.0))
        return self.set_ir_scene((0.0, 0.0), 0.0, rel)

    def set_ir_scene(self, robot_m, heading: float, ball_m, ball_vel=(0.0, 0.0), *, robots=(),
                     visible: bool = True, dt: float = 0.0) -> np.ndarray:
        """Physical IR scene (sim frame): the ring's photocurrents go to the board's front end,
        effective from now. ``dt`` rolls the ball's LEDs on by that much time."""
        i_on = self.ir_scene.update(robot_m, heading, ball_m, ball_vel, robots=robots,
                                    visible=visible and ball_m is not None, dt=dt)
        self.ir_board.ir.set_drive([float(v) for v in i_on])
        self.ir_board.ir_physical = True
        m = self.cfg.ir
        codes = m.front_end.code_of(i_on, m.ball.carrier_hz)
        codes[m.n_sensors:] = 0
        return codes

    def set_ir_raw(self, values16) -> None:
        """Fixed ADC codes per physical sensor, bypassing the IR physics."""
        self.ir_board.ir_physical = False
        values = self.ir_board.values
        values[0] = [int(v) for v in values16]
        self.ir_board.values = values

    def set_line_raw(self, values_4x16) -> None:
        self.line_board.values = [[int(v) for v in row] for row in np.asarray(values_4x16)]

    def set_encoder_rates(self, ticks_per_s) -> None:
        self.fw.devices.encoders.rate = [float(v) for v in ticks_per_s]

    def press_button(self, which: int, at_us: float | None = None, hold_ms: float = 100.0) -> None:
        """Press button 1 or 2 (now, or at virtual time ``at_us``) for ``hold_ms``."""
        p = self.fw.board.BUTTON1 if which == 1 else self.fw.board.BUTTON2
        t0 = int((self.fw.sim.now_us() if at_us is None else at_us) * 1000)
        self.fw.sim.schedule_input(p, 1, t0)
        self.fw.sim.schedule_input(p, 0, t0 + int(hold_ms * 1e6))

    # ── everything at once ────────────────────────────────────────────────────────────────
    def apply_ir_truth(self, t: RobotTruth, dt: float = 0.0) -> np.ndarray:
        return self.set_ir_scene(t.pos_m, t.heading, t.ball_m, t.ball_vel_mps, robots=t.robots,
                                 visible=t.ball_visible, dt=dt)

    def apply_truth(self, t: RobotTruth, ir_dt: float = 0.0) -> dict:
        """Push the readings every sensor would produce for ``t`` into the simulated chips.
        Returns them (for logging / the viewer)."""
        cfg = self.cfg
        out: dict = {"compass": self.set_heading(t.heading)}

        a_fw = frames.world_to_fw_body(t.accel_mps2, t.heading) / 9.81
        out["accel"] = self.set_accel(a_fw)

        dist = sensors.sonar_distances_cm(t.pos_m, t.heading, cfg.sonar, robots=t.robots,
                                          rng=self.rng)
        self.set_sonar_cm(dist)
        out["sonar_cm"] = dist

        out["ir"] = self.apply_ir_truth(t, ir_dt)

        line = sensors.line_adc(t.pos_m, t.heading, cfg.line, self.rng)
        self.set_line_raw(line)
        out["line"] = line

        v_fw = frames.world_to_fw_body(t.vel_mps, t.heading)
        rim = wheel_rim_speeds(v_fw, -t.omega)
        rates = sensors.encoder_rates(rim, cfg.encoders)
        self.set_encoder_rates(rates)
        out["encoder_rates"] = rates
        return out

    def read_actuators(self) -> Actuators:
        avg = self.fw.devices.pwm.take_average()
        peak = self.fw.devices.pwm.take_max()
        return Actuators(duty=np.asarray(avg[:6]).reshape(3, 2), kick=float(peak[6]))
