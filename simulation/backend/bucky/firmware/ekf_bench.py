"""ekf_tracking: the firmware's EKF (robot/src/strategy/EKF.cpp), fed by every sensor through
the real drivers on the simulated board, graded against the truth.

Each scenario drives the robot along a path in the training physics (still, back-and-forth,
circle, figure-eight, turning circle, spin in place) while the ball lies still or rolls.
Every 10 ms:

    physics truth → SimHardware (raw readings into the simulated chips) → the firmware drivers
        Accelerometer::read  → EKF::predict           (every step)
        Compass::update      → EKF::updateHeading     (``compass_hz``; start heading + getOffset)
        Sonar (non-blocking) → glue.sonar_localize    → EKF::updatePosition (``sonar_hz``)
        encoder_get_speed    → glue.encoder_field_velocity → EKF::updateSpeed (``encoder_hz``)
        GPort::readIR        → glue.ir_bearing_range  → EKF::updateBall (every new IR frame)

and the estimate is compared with the truth after a settling time. The module (kind ``"ekf"``,
name ``firmware_ekf``) holds the filter's tuning (EKFNoise) and which sensors it uses; the
experiment holds the scenario and how noisy the simulated sensors are.
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from bucky.firmware import frames, glue
from bucky.firmware.hardware import RobotHardwareConfig, RobotTruth, SimHardware
from bucky.firmware.loader import load_firmware
from bucky.game.field import BALL_RADIUS, HALF_H, HALF_W
from bucky.lab.experiments.base import Experiment, register_experiment, summarize_metrics
from bucky.lab.modules import LabModule
from bucky.lab.params import Param
from bucky.lab.registry import register
from bucky.physics.python_backend import MAX_LINEAR, MAX_OMEGA, TwoRobotPhysics

DT = 0.01
_PARKED_B = np.zeros(4)
X, Y, TH, VX, VY, BX, BY, BVX, BVY = range(9)

#: EKFNoise defaults (strategy/EKF.h); tests/test_firmware_ekf.py checks they still match.
EKF_NOISE_DEFAULTS = {
    "accel": 0.05 * 981.0, "thetaDrift": 0.05, "ballAccel": 300.0, "ballSpeedInit": 100.0,
    "sonarPos": 3.0, "compassTheta": 0.05, "encoderSpeed": 5.0, "irAngle": 0.1,
    "irDistance": 10.0,
}
_NOISE_HELP = {
    "accel": "Accelerometer noise incl. vibration (σ, cm/s²) → Q",
    "thetaDrift": "Heading random walk (rad/√s) → Q",
    "ballAccel": "Unmodelled ball acceleration (σ, cm/s²) → Q",
    "ballSpeedInit": "Ball speed uncertainty on first sighting (σ, cm/s)",
    "sonarPos": "Sonar-derived x/y (σ, cm) → R",
    "compassTheta": "Compass heading (σ, rad) → R",
    "encoderSpeed": "Encoder field speed (σ, cm/s) → R",
    "irAngle": "IR ball bearing (σ, rad) → R",
    "irDistance": "IR ball distance (σ, cm) → R",
}


@register
class FirmwareEKF(LabModule):
    """The robot's EKF (robot/src/strategy/EKF.cpp) fusing accelerometer, compass, sonar,
    encoders and IR, each read through its real firmware driver on the simulated board."""

    kind = "ekf"
    name = "firmware_ekf"
    sensors = ()
    params = {
        **{f"noise_{k}": Param(v, 0.0, max(10.0, v * 10), v / 20 if v else 0.01, _NOISE_HELP[k])
           for k, v in EKF_NOISE_DEFAULTS.items()},
        "init_pos_std": Param(20.0, 0.1, 200.0, 1.0, "init(): position σ (cm)"),
        "init_theta_std": Param(0.3, 0.01, 3.2, 0.01, "init(): heading σ (rad)"),
        "use_accel": Param(True, help="Predict with the accelerometer (else zero input)"),
        "use_compass": Param(True, help="Heading updates from the compass"),
        "use_sonar": Param(True, help="Position updates from the sonar"),
        "use_encoders": Param(True, help="Speed updates from the wheel encoders"),
        "use_ir": Param(True, help="Ball updates from the IR ring"),
        "compass_hz": Param(50.0, 1.0, 100.0, 1.0, "Compass update rate"),
        "sonar_hz": Param(20.0, 1.0, 40.0, 1.0, "Sonar ping rate"),
        "encoder_hz": Param(50.0, 1.0, 100.0, 1.0, "Encoder update rate"),
    }

    def step(self, ctx, cmd):
        raise NotImplementedError("the EKF module runs through EkfBench")


class EkfBench:
    """Executor for ``kind="ekf"``: one simulated board, the drivers, the EKF and the physics."""

    def __init__(self, module: FirmwareEKF, seed: int = 0) -> None:
        self.fw = load_firmware()
        self.module = module
        self.seed = seed
        self.physics = TwoRobotPhysics(DT)
        self.physics.set_removed("b", True)
        self.cfg = RobotHardwareConfig()
        self.t = 0.0

    def configure(self, cfg: RobotHardwareConfig) -> None:
        self.cfg = cfg

    def state(self):
        return self.physics.state_a()

    # ── setup ──────────────────────────────────────────────────────────────────────────────
    def reset(self, robot_pos, robot_heading: float, ball_pos, ball_vel=(0.0, 0.0), *,
              init_pos_err_cm=(0.0, 0.0), init_theta_err: float = 0.0) -> None:
        fw, p = self.fw, self.module.p
        fw.sim.stop()
        fw.sim.reboot()            # drops the previous episode's drivers' interrupt handlers
        self.rng = np.random.default_rng(self.seed)
        self.physics.place_ball(np.asarray(ball_pos, float), np.asarray(ball_vel, float))
        self.physics.place_robot("a", np.asarray(robot_pos, float), robot_heading)
        self.hw = SimHardware(fw, self.cfg, self.rng)
        self._prev_vel = np.zeros(2)
        self._accel = np.zeros(2)
        self.hw.apply_truth(self._truth())

        self.bus = fw.make_sensor_bus()
        self.compass = fw.Compass()
        self.compass.begin(self.bus)
        while not self.compass.tick():          # ~0.52 s of virtual boot, robot held still
            fw.sim.advance_ns(1_000_000)
        self.compass.update()
        self.compass.reset()                    # offsets are relative to the start heading
        self.accel = fw.Accelerometer()
        self.accel.begin(self.bus)
        self.sonar = fw.Sonar()
        self.sonar.begin(fw.board.SONAR)
        self.pinging = False
        for i in range(3):
            fw.encoder_init(i)
        ir_hw = fw.board.G_PORT1 if fw.board.G_PORT1_KIND == int(fw.GSensorKind.IR) \
            else fw.board.G_PORT2
        self.ir = fw.GPort()
        self.ir.begin(ir_hw, fw.GSensorKind.IR)
        self.ir_seq = self.ir.frameSequence()
        fw.sim.usb_read()

        noise = fw.EKFNoise()
        for k in EKF_NOISE_DEFAULTS:
            setattr(noise, k, float(getattr(p, f"noise_{k}")))
        self.ekf = fw.EKF()
        self.ekf.setNoise(noise)
        self.theta0 = frames.sim_to_fw_heading(robot_heading)
        pos_fw = frames.sim_to_fw_pos(robot_pos)
        self.ekf.init(float(pos_fw[0] + init_pos_err_cm[0]), float(pos_fw[1] + init_pos_err_cm[1]),
                      self.theta0 + init_theta_err, p.init_pos_std, p.init_theta_std)
        self.t = 0.0
        self.k = 0
        self.counts = {"sonar": 0, "ir": 0, "compass": 0, "encoder": 0}

    def _truth(self) -> RobotTruth:
        s = self.physics.state_a()
        return RobotTruth(pos_m=s.robot_pos, heading=float(s.robot_heading), vel_mps=s.robot_vel,
                          omega=float(s.robot_omega), accel_mps2=self._accel, ball_m=s.ball_pos)

    def _every(self, hz: float) -> bool:
        return self.k % max(1, round(1.0 / (hz * DT))) == 0

    # ── one 10 ms step ─────────────────────────────────────────────────────────────────────
    def step(self, action) -> dict:
        fw, p, ekf = self.fw, self.module.p, self.ekf
        info = self.physics.step(action, _PARKED_B)
        vel = self.physics.state_a().robot_vel
        self._accel = (vel - self._prev_vel) / DT
        self._prev_vel = vel.copy()
        self.hw.apply_truth(self._truth())
        fw.sim.advance_ns(int(DT * 1e9))
        self.k += 1
        self.t += DT

        ax = ay = 0.0
        if p.use_accel:
            r = self.accel.read()
            if r is not None:
                ax, ay = r[0], r[1]
        ekf.predict(ax, ay, DT)
        st = ekf.getState()

        if p.use_compass and self._every(p.compass_hz) and self.compass.update():
            th = self.theta0 + math.radians(self.compass.getOffset())
            ekf.updateHeading(math.remainder(th, math.tau))
            self.counts["compass"] += 1

        for i in range(3):
            fw.encoder_update_speed(i)
        if p.use_encoders and self._every(p.encoder_hz):
            v = glue.encoder_field_velocity([fw.encoder_get_speed(i) for i in range(3)],
                                            st.robotTheta, self.cfg.encoders)
            ekf.updateSpeed(float(v[0]), float(v[1]))
            self.counts["encoder"] += 1

        if p.use_sonar:
            if self.pinging and self.sonar.isReadComplete():
                r = self.sonar.processRead()
                self.pinging = False
                st = ekf.getState()
                pos = glue.sonar_localize(r.distance, r.valid, st.robotTheta,
                                          (st.robotX, st.robotY), self.cfg.sonar)
                if pos is not None:
                    ekf.updatePosition(*pos)
                    self.counts["sonar"] += 1
            elif not self.pinging and self._every(p.sonar_hz):
                self.sonar.startRead()
                self.pinging = True

        if p.use_ir and self.ir.hasNewFrame(self.ir_seq):
            self.ir_seq = self.ir.frameSequence()
            frame = self.ir.readIR()
            meas = glue.ir_bearing_range(frame, baseline=self.cfg.ir.baseline,
                                         range_k=self.cfg.ir.range_k,
                                         n_sensors=self.cfg.ir.n_sensors) if frame else None
            if meas is not None:
                ekf.updateBall(*meas)
                self.counts["ir"] += 1
        fw.sim.usb_read()          # the drivers' debug prints are not needed here
        return info

    def estimate(self) -> np.ndarray:
        s = self.ekf.getState()
        return np.array([s.robotX, s.robotY, s.robotTheta, s.speedX, s.speedY,
                         s.ballX, s.ballY, s.ballSpeedX, s.ballSpeedY], dtype=float)

    def truth_fw(self) -> np.ndarray:
        s = self.physics.state_a()
        return np.array([*frames.sim_to_fw_pos(s.robot_pos), frames.sim_to_fw_heading(
            s.robot_heading), *frames.sim_to_fw_pos(s.robot_vel), *frames.sim_to_fw_pos(
            s.ball_pos), *frames.sim_to_fw_pos(s.ball_vel)])


# ── scenarios: paths the robot follows ──────────────────────────────────────────────────────

MOTIONS = ("still", "line", "circle", "figure8", "turning", "spin")


def path(motion: str, t: float, p0, h0: float, speed: float):
    """Desired (position m, velocity m/s, heading rad, ω rad/s) at time t, sim frame."""
    p0 = np.asarray(p0, float)
    zero = np.zeros(2)
    if motion == "line":
        a = 0.4
        w = speed / a
        return p0 + [a * math.sin(w * t), 0], np.array([a * w * math.cos(w * t), 0]), h0, 0.0
    if motion in ("circle", "turning"):
        r = 0.3
        w = speed / r
        pos = p0 + r * np.array([math.cos(w * t) - 1, math.sin(w * t)])
        vel = r * w * np.array([-math.sin(w * t), math.cos(w * t)])
        if motion == "turning":                  # face the direction of travel
            return pos, vel, h0 + w * t, w
        return pos, vel, h0, 0.0
    if motion == "figure8":
        a = 0.4
        w = speed / a / 1.3
        pos = p0 + [a * math.sin(w * t), a / 2 * math.sin(2 * w * t)]
        vel = np.array([a * w * math.cos(w * t), a * w * math.cos(2 * w * t)])
        return pos, vel, h0, 0.0
    if motion == "spin":
        w = math.pi                               # 180 °/s
        return p0, zero, h0 + w * t, w
    return p0, zero, h0, 0.0


def follow(state, pos, vel, heading, omega) -> np.ndarray:
    """Physics action that tracks the path (feed-forward + proportional correction)."""
    v = vel + 4.0 * (pos - state.robot_pos)
    w = omega + 6.0 * math.remainder(heading - state.robot_heading, math.tau)
    c, s = math.cos(state.robot_heading), math.sin(state.robot_heading)
    body = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]]) / MAX_LINEAR
    return np.array([body[0], body[1], float(np.clip(w / MAX_OMEGA, -1, 1)), 0.0])


def _grid(lim: float, step: float) -> np.ndarray:
    k = int(math.floor(lim / step + 1e-9))
    return np.arange(-k, k + 1) * step


@register_experiment
class EkfTracking(Experiment):
    """Drive the robot along paths all over the field and grade the firmware EKF's estimate of
    the robot (position, heading, speed) and the ball against the truth."""

    name = "ekf_tracking"
    module_kind = "ekf"
    params = {
        "mode": Param("field", options=("field",), help="Path centres over the field"),
        "robot_step_cm": Param(25.0, 10.0, 100.0, 5.0, "Grid spacing of the path centres"),
        "motion": Param("mixed", options=("mixed",) + MOTIONS,
                        help="Robot path ('mixed' runs every path at every centre)"),
        "speed_cm_s": Param(50.0, 0.0, 200.0, 5.0, "Path speed"),
        "duration_s": Param(6.0, 2.0, 30.0, 1.0, "Episode length"),
        "settle_s": Param(1.0, 0.0, 5.0, 0.5, "Errors are counted after this"),
        "ball": Param("rolling", options=("still", "rolling"), help="Ball still or rolling"),
        "ball_speed_cm_s": Param(40.0, 0.0, 200.0, 5.0, "Rolling ball start speed"),
        "init_pos_err_cm": Param(0.0, 0.0, 100.0, 5.0, "EKF starts this far off in x"),
        "init_theta_err_deg": Param(0.0, 0.0, 90.0, 5.0, "EKF starts this far off in heading"),
        "compass_noise_lsb": Param(2.0, 0.0, 30.0, 0.5, "Simulated magnetometer noise (σ, LSB)"),
        "accel_noise_mg": Param(4.0, 0.0, 200.0, 1.0, "Simulated accelerometer noise (σ, mg)"),
        "sonar_noise_cm": Param(0.5, 0.0, 20.0, 0.5, "Simulated sonar noise (σ, cm)"),
        "sonar_dropout": Param(0.0, 0.0, 1.0, 0.05, "Chance a sonar echo is lost"),
        "ir_noise": Param(3.0, 0.0, 100.0, 1.0, "Simulated IR ADC noise (σ, counts)"),
    }
    metric_specs = [
        {"key": "score", "label": "Score", "higher_is_better": True, "unit": "", "summary": True},
        {"key": "pos_rmse_cm", "label": "Position error", "higher_is_better": False,
         "unit": "cm", "summary": True},
        {"key": "heading_rmse_deg", "label": "Heading error", "higher_is_better": False,
         "unit": "deg", "summary": True},
        {"key": "speed_rmse_cm_s", "label": "Speed error", "higher_is_better": False,
         "unit": "cm/s", "summary": True},
        {"key": "ball_rmse_cm", "label": "Ball error", "higher_is_better": False, "unit": "cm",
         "summary": True},
        {"key": "ball_speed_rmse_cm_s", "label": "Ball speed error", "higher_is_better": False,
         "unit": "cm/s"},
        {"key": "pos_max_cm", "label": "Worst position error", "higher_is_better": False,
         "unit": "cm"},
        {"key": "consistency", "label": "Error / claimed σ (NEES/5)", "higher_is_better": False,
         "unit": "", "summary": True},
        {"key": "ball_in_range", "label": "Ball within IR range", "higher_is_better": True,
         "unit": "pct", "domain": [0, 1]},
        {"key": "sonar_fix_hz", "label": "Sonar fixes", "higher_is_better": True, "unit": "Hz"},
        {"key": "ir_hz", "label": "IR ball updates", "higher_is_better": True, "unit": "Hz"},
    ]

    def scenarios(self) -> list[dict]:
        p = self.p
        step = p.robot_step_cm / 100.0
        lim_x, lim_y = HALF_W - 0.45, HALF_H - 0.35       # keep the paths on the field
        motions = MOTIONS if p.motion == "mixed" else (p.motion,)
        out: list[dict] = []
        for cx in _grid(lim_x, step):
            for cy in _grid(lim_y, step):
                for motion in motions:
                    sid = len(out)
                    rng = np.random.default_rng(2000 + sid)
                    h0 = float(rng.uniform(-math.pi, math.pi))
                    robot = np.array([cx, cy])
                    # Ball 30–60 cm from the path centre, where the IR ring can see it.
                    a, d = rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 0.6)
                    ball = robot + d * np.array([math.cos(a), math.sin(a)])
                    ball = np.clip(ball, [-HALF_W + BALL_RADIUS, -HALF_H + BALL_RADIUS],
                                   [HALF_W - BALL_RADIUS, HALF_H - BALL_RADIUS])
                    bv = [0.0, 0.0]
                    if p.ball == "rolling":
                        a = rng.uniform(-math.pi, math.pi)
                        bv = (p.ball_speed_cm_s / 100.0
                              * np.array([math.cos(a), math.sin(a)])).tolist()
                    out.append({"id": sid, "robot": robot.tolist(), "heading": h0,
                                "ball": ball.tolist(), "ball_vel": bv, "motion": motion})
        return out

    def _config(self) -> RobotHardwareConfig:
        p, base = self.p, RobotHardwareConfig()
        return replace(
            base,
            compass=replace(base.compass, noise_lsb=p.compass_noise_lsb),
            accel=replace(base.accel, noise_mg=p.accel_noise_mg),
            sonar=replace(base.sonar, noise_cm=p.sonar_noise_cm, dropout=p.sonar_dropout),
            ir=replace(base.ir, noise=p.ir_noise),
        )

    def run_episode(self, ex: EkfBench, scenario: dict, record: bool = False) -> dict:
        p = self.p
        ex.configure(self._config())
        p0, h0 = np.array(scenario["robot"]), float(scenario["heading"])
        ex.reset(p0, h0, scenario["ball"], scenario.get("ball_vel", (0.0, 0.0)),
                 init_pos_err_cm=(p.init_pos_err_cm, 0.0),
                 init_theta_err=math.radians(p.init_theta_err_deg))
        motion, speed = scenario["motion"], p.speed_cm_s / 100.0
        errs, nees, ball_errs, trace = [], [], [], []
        ball_steps = ball_visible = 0
        steps = int(round(p.duration_s / DT))
        for _ in range(steps):
            pos, vel, heading, omega = path(motion, ex.t + DT, p0, h0, speed)
            ex.step(follow(ex.state(), pos, vel, heading, omega))
            est, tru = ex.estimate(), ex.truth_fw()
            e = est - tru
            e[TH] = math.remainder(e[TH], math.tau)
            if ex.t >= p.settle_s:
                errs.append(e[:5])
                P = ex.ekf.getCovariance()[:5, :5]
                try:
                    nees.append(float(e[:5] @ np.linalg.solve(P, e[:5])))
                except np.linalg.LinAlgError:
                    pass
                # Ball error only while the IR ring can see the ball (see sensor_check).
                in_range = math.hypot(*(tru[5:7] - tru[:2])) <= ex.cfg.ir.range_k / 10.0
                ball_steps += 1
                if in_range:
                    ball_visible += 1
                    if ex.counts["ir"]:
                        ball_errs.append(e[5:9])
            if record:
                s = ex.state()
                th = est[TH]
                tip = est[:2] + 15.0 * np.array([math.sin(th), math.cos(th)])
                marks = {"E": frames.fw_to_sim_pos(est[:2]).tolist(),
                         "EH": frames.fw_to_sim_pos(tip).tolist()}
                if ex.counts["ir"]:
                    marks["EB"] = frames.fw_to_sim_pos(est[5:7]).tolist()
                trace.append({"t": round(ex.t, 3),
                              "r": [float(s.robot_pos[0]), float(s.robot_pos[1]),
                                    float(s.robot_heading)],
                              "b": [float(s.ball_pos[0]), float(s.ball_pos[1])], "m": marks})

        E = np.array(errs) if errs else np.zeros((1, 5))
        B = np.array(ball_errs) if ball_errs else None
        rms = lambda a: float(np.sqrt(np.mean(a ** 2)))   # noqa: E731
        pos_err = np.hypot(E[:, 0], E[:, 1])
        metrics = {
            "pos_rmse_cm": rms(pos_err),
            "pos_max_cm": float(pos_err.max()),
            "heading_rmse_deg": math.degrees(rms(E[:, 2])),
            "speed_rmse_cm_s": rms(np.hypot(E[:, 3], E[:, 4])),
            "ball_rmse_cm": rms(np.hypot(B[:, 0], B[:, 1])) if B is not None else None,
            "ball_speed_rmse_cm_s": rms(np.hypot(B[:, 2], B[:, 3])) if B is not None else None,
            "consistency": float(np.mean(nees)) / 5.0 if nees else None,
            "ball_in_range": ball_visible / ball_steps if ball_steps else 0.0,
            "sonar_fix_hz": ex.counts["sonar"] / p.duration_s,
            "ir_hz": ex.counts["ir"] / p.duration_s,
        }
        score = 100.0 - 2.0 * metrics["pos_rmse_cm"] - metrics["heading_rmse_deg"] \
            - 0.5 * metrics["speed_rmse_cm_s"] - 0.3 * (metrics["ball_rmse_cm"] or 0.0)
        metrics["score"] = round(max(-100.0, score), 3)
        worst = max((("position", metrics["pos_rmse_cm"] / 5.0),
                     ("heading", metrics["heading_rmse_deg"] / 5.0),
                     ("speed", metrics["speed_rmse_cm_s"] / 10.0),
                     ("ball", (metrics["ball_rmse_cm"] or 0.0) / 15.0)), key=lambda kv: kv[1])
        metrics["note"] = f"{motion}: {worst[0]} worst" + \
            ("" if (metrics["consistency"] or 0) < 3 else ", overconfident")
        out = {"metrics": metrics}
        if record:
            out["trace"] = trace
        return out

    def aggregate(self, records: list[dict], worst_n: int = 25) -> dict:
        agg = super().aggregate(records, worst_n)
        by_motion: dict[str, list] = {}
        for r in records:
            by_motion.setdefault(r["scenario"]["motion"], []).append(r["metrics"])
        agg["by_motion"] = [{"motion": k, **summarize_metrics(v)} for k, v in by_motion.items()]
        return agg
