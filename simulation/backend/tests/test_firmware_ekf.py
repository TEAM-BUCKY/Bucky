"""The firmware's EKF (robot/src/strategy/EKF.cpp): each sensor update on its own, then the
whole filter fusing all of them.

Three layers per sensor:

* **math** — the compiled firmware EKF against an independent float64 numpy re-implementation
  of the model documented in EKF.h, operation by operation (state *and* covariance);
* **Jacobians** — the hand-derived Jacobians in EKF.cpp against finite differences of the
  model, so a sign or index slip in a derivative shows up even when both sides agree;
* **pipeline** — the reading travelling the real path on the simulated board: physical truth →
  simulated chip → the firmware driver (Compass, Accelerometer, Sonar, encoders, GPort) → the
  EKF. Where the firmware has no code yet between a driver and the EKF (sonar distances →
  position, encoder ticks → field velocity, IR frame → bearing), :mod:`bucky.firmware.glue`
  does that step.

EKF frame (EKF.h): field cm, x right, y towards the opponent goal; theta clockwise from +y, so
"forward" in the field is (sin θ, cos θ) and the robot's right is (cos θ, -sin θ).
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from bucky.firmware import glue, sensors
from bucky.firmware import kinematics as kin

X, Y, TH, VX, VY, BX, BY, BVX, BVY = range(9)
N = 9
G_CM = 981.0
PI_F = float(np.float32(3.14159265358979))


# ── independent reference model ──────────────────────────────────────────────────────────────

def wrap(a: float) -> float:
    """Math::wrapRadians: into [-π, π] (two folds, like the firmware)."""
    for _ in range(2):
        if a > PI_F:
            a -= 2 * PI_F
        elif a < -PI_F:
            a += 2 * PI_F
    return a


def noise_dict(n) -> dict:
    return {k: float(getattr(n, k)) for k in ("accel", "thetaDrift", "ballAccel", "ballSpeedInit",
                                               "sonarPos", "compassTheta", "encoderSpeed",
                                               "irAngle", "irDistance")}


class RefEKF:
    """The EKF.h model in float64. Written from the model description (state, f, h, Q, R), not
    by transliterating EKF.cpp: F comes from :func:`jacobian`-checked formulas and the update
    is the textbook scalar-sequential Kalman update."""

    def __init__(self, noise: dict, field_wh: tuple[float, float]) -> None:
        self.n = noise
        self.field_w, self.field_h = field_wh   # DigitalField.h FIELD_WIDTH / FIELD_HEIGHT
        self.x = np.zeros(N)
        self.P = np.zeros((N, N))
        self.ball_init = False

    def init(self, x, y, th, positionStd=20.0, thetaStd=0.3) -> None:   # noqa: N803 (C++ names)
        pos_std, th_std = positionStd, thetaStd
        n = self.n
        self.x = np.zeros(N)
        self.x[[X, Y, TH]] = x, y, wrap(th)
        self.P = np.diag([pos_std ** 2, pos_std ** 2, th_std ** 2, n["encoderSpeed"] ** 2,
                          n["encoderSpeed"] ** 2, self.field_w ** 2, self.field_h ** 2,
                          n["ballSpeedInit"] ** 2, n["ballSpeedInit"] ** 2])
        self.ball_init = False

    @staticmethod
    def f(x: np.ndarray, ax_g: float, ay_g: float, dt: float) -> np.ndarray:
        """Motion model: robot-frame acceleration → field frame, constant-acceleration step;
        ball at constant velocity."""
        th = x[TH]
        a_right, a_fwd = ax_g * G_CM, ay_g * G_CM
        af = a_right * np.array([math.cos(th), -math.sin(th)]) + \
            a_fwd * np.array([math.sin(th), math.cos(th)])
        out = x.copy()
        out[[X, Y]] += x[[VX, VY]] * dt + 0.5 * dt * dt * af
        out[[VX, VY]] += af * dt
        out[[BX, BY]] += x[[BVX, BVY]] * dt
        return out

    def predict(self, ax_g, ay_g, dt) -> None:
        F = numeric_jacobian(lambda s: self.f(s, ax_g, ay_g, dt), self.x)
        self.x = self.f(self.x, ax_g, ay_g, dt)
        n, h2 = self.n, 0.5 * dt * dt
        Q = np.zeros((N, N))
        for p, v, q in ((X, VX, n["accel"] ** 2), (Y, VY, n["accel"] ** 2),
                        (BX, BVX, n["ballAccel"] ** 2), (BY, BVY, n["ballAccel"] ** 2)):
            Q[p, p] += h2 * h2 * q
            Q[p, v] += h2 * dt * q
            Q[v, p] += h2 * dt * q
            Q[v, v] += dt * dt * q
        Q[TH, TH] = n["thetaDrift"] ** 2 * dt
        self.P = F @ self.P @ F.T + Q

    def _update(self, h, innovation, var) -> None:
        Ph = self.P @ h
        s = float(h @ Ph) + var
        if s <= 0:
            return
        K = Ph / s
        self.x = self.x + K * innovation
        self.x[TH] = wrap(self.x[TH])
        self.P = self.P - np.outer(K, Ph)
        self.P = (self.P + self.P.T) / 2

    def _unit(self, i) -> np.ndarray:
        h = np.zeros(N)
        h[i] = 1.0
        return h

    def update_position(self, x, y) -> None:
        r = self.n["sonarPos"] ** 2
        self._update(self._unit(X), x - self.x[X], r)
        self._update(self._unit(Y), y - self.x[Y], r)

    def update_heading(self, th) -> None:
        self._update(self._unit(TH), wrap(th - self.x[TH]), self.n["compassTheta"] ** 2)

    def update_speed(self, vx, vy) -> None:
        r = self.n["encoderSpeed"] ** 2
        self._update(self._unit(VX), vx - self.x[VX], r)
        self._update(self._unit(VY), vy - self.x[VY], r)

    @staticmethod
    def h_ball(x: np.ndarray) -> np.ndarray:
        """IR measurement model: [bearing (robot frame, CW), distance]."""
        dx, dy = x[BX] - x[X], x[BY] - x[Y]
        return np.array([math.atan2(dx, dy) - x[TH], math.hypot(dx, dy)])

    def update_ball(self, angle, dist) -> None:
        n = self.n
        if not self.ball_init:
            fa = self.x[TH] + angle
            self.x[[BX, BY]] = self.x[X] + dist * math.sin(fa), self.x[Y] + dist * math.cos(fa)
            self.x[[BVX, BVY]] = 0.0
            pos_var = n["irDistance"] ** 2 + dist * dist * n["irAngle"] ** 2
            for b in (BX, BY, BVX, BVY):
                self.P[b, :] = 0.0
                self.P[:, b] = 0.0
            self.P[BX, BX] = pos_var + self.P[X, X]
            self.P[BY, BY] = pos_var + self.P[Y, Y]
            self.P[BVX, BVX] = self.P[BVY, BVY] = n["ballSpeedInit"] ** 2
            self.ball_init = True
            return
        H = numeric_jacobian(self.h_ball, self.x)
        self._update(H[0], wrap(angle - self.h_ball(self.x)[0]), n["irAngle"] ** 2)
        H = numeric_jacobian(self.h_ball, self.x)       # relinearise after the bearing row
        self._update(H[1], dist - self.h_ball(self.x)[1], n["irDistance"] ** 2)


def numeric_jacobian(fn, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    y0 = np.atleast_1d(fn(x))
    J = np.zeros((len(y0), len(x)))
    for i in range(len(x)):
        d = np.zeros(len(x))
        d[i] = eps
        J[:, i] = (np.atleast_1d(fn(x + d)) - np.atleast_1d(fn(x - d))) / (2 * eps)
    return J


# ── helpers ──────────────────────────────────────────────────────────────────────────────────

def state_vec(ekf) -> np.ndarray:
    s = ekf.getState()
    return np.array([s.robotX, s.robotY, s.robotTheta, s.speedX, s.speedY,
                     s.ballX, s.ballY, s.ballSpeedX, s.ballSpeedY], dtype=float)


def assert_matches(ekf, ref: RefEKF, rtol=2e-3, atol=2e-3) -> None:
    np.testing.assert_allclose(state_vec(ekf), ref.x, rtol=rtol, atol=atol)
    np.testing.assert_allclose(ekf.getCovariance(), ref.P, rtol=rtol,
                               atol=atol * max(1.0, float(np.max(np.abs(ref.P)))) * 1e-3)


def assert_valid_covariance(P: np.ndarray) -> None:
    assert np.allclose(P, P.T, atol=1e-3 * max(1.0, np.max(np.abs(P))))
    assert np.min(np.linalg.eigvalsh((P + P.T) / 2)) > -1e-3 * max(1.0, np.max(np.abs(P)))


def pair(fw, x=10.0, y=-20.0, th=0.3, **kw):
    ekf = fw.EKF()
    ekf.init(x, y, th, **kw)
    ref = RefEKF(noise_dict(fw.EKFNoise()), (fw.FIELD_WIDTH, fw.FIELD_HEIGHT))
    ref.init(x, y, th, **kw)
    return ekf, ref


def field_from_body(v_right, v_fwd, th) -> np.ndarray:
    return v_right * np.array([math.cos(th), -math.sin(th)]) + \
        v_fwd * np.array([math.sin(th), math.cos(th)])


def body_from_field(v, th) -> np.ndarray:
    return np.array([v[0] * math.cos(th) - v[1] * math.sin(th),
                     v[0] * math.sin(th) + v[1] * math.cos(th)])


# ── init, writeTo, noise ─────────────────────────────────────────────────────────────────────

def test_init_state_and_covariance(fw):
    ekf, ref = pair(fw, 12.0, 34.0, 7.0, positionStd=5.0, thetaStd=0.1)  # 7 rad wraps
    assert_matches(ekf, ref)
    assert ekf.getState().robotTheta == pytest.approx(wrap(7.0), abs=1e-5)
    P = ekf.getCovariance()
    assert P[X, X] == pytest.approx(25.0) and P[TH, TH] == pytest.approx(0.01)
    # Ball unknown: σ = the field size from DigitalField.h.
    assert P[BX, BX] == pytest.approx(fw.FIELD_WIDTH ** 2)
    assert P[BY, BY] == pytest.approx(fw.FIELD_HEIGHT ** 2)
    assert np.count_nonzero(P - np.diag(np.diag(P))) == 0     # uncorrelated at start


def test_write_to_digital_field(fw):
    ekf, _ = pair(fw, 15.0, -40.0, 0.0)
    ekf.updateBall(0.0, 50.0)
    field = fw.DigitalField()
    ekf.writeTo(field)
    assert (field.robot.x, field.robot.y) == pytest.approx((15.0, -40.0))
    assert (field.ball.x, field.ball.y) == pytest.approx((15.0, 10.0), abs=1e-3)


def test_set_noise_is_used(fw):
    loose = fw.EKFNoise()
    loose.sonarPos = 50.0
    a, b = fw.EKF(), fw.EKF()
    for e in (a, b):
        e.init(0.0, 0.0, 0.0)
    b.setNoise(loose)
    for e in (a, b):
        e.updatePosition(30.0, 0.0)
    assert a.getState().robotX > b.getState().robotX > 0.0   # noisier sonar → smaller step


# ── accelerometer: predict ───────────────────────────────────────────────────────────────────

class TestAccelerometerPredict:
    def test_matches_reference(self, fw):
        ekf, ref = pair(fw)
        ekf.updateBall(0.4, 80.0)
        ref.update_ball(0.4, 80.0)
        ekf.updateSpeed(30.0, -10.0)
        ref.update_speed(30.0, -10.0)
        rng = np.random.default_rng(1)
        for _ in range(50):
            ax, ay, dt = rng.normal(0, 0.3), rng.normal(0, 0.3), float(rng.uniform(0.002, 0.02))
            ekf.predict(ax, ay, dt)
            ref.predict(ax, ay, dt)
        assert_matches(ekf, ref, rtol=5e-3, atol=5e-3)

    @pytest.mark.parametrize("th_deg", [0, 90, 180, -45])
    def test_forward_acceleration_moves_along_heading(self, fw, th_deg):
        th = math.radians(th_deg)
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, th)
        for _ in range(100):                       # 1 s at 0.1 g forward
            ekf.predict(0.0, 0.1, 0.01)
        s = ekf.getState()
        d = 0.5 * 0.1 * G_CM * 1.0 ** 2            # 49 cm
        assert (s.robotX, s.robotY) == pytest.approx((d * math.sin(th), d * math.cos(th)), abs=0.5)
        assert (s.speedX, s.speedY) == pytest.approx(
            (98.1 * math.sin(th), 98.1 * math.cos(th)), abs=0.1)

    def test_right_acceleration_is_robot_right(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, math.radians(90))       # facing field +x: robot right = field -y
        ekf.predict(0.2, 0.0, 0.1)
        s = ekf.getState()
        assert s.speedX == pytest.approx(0.0, abs=1e-3) and s.speedY < 0

    def test_constant_velocity_and_ball(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        ekf.updateSpeed(20.0, 10.0)
        ekf.updateBall(0.0, 100.0)
        v0 = state_vec(ekf)
        for _ in range(50):
            ekf.predict(0.0, 0.0, 0.02)
        x = state_vec(ekf)
        np.testing.assert_allclose(x[[X, Y]], v0[[X, Y]] + v0[[VX, VY]] * 1.0, atol=1e-3)
        np.testing.assert_allclose(x[[BX, BY]], v0[[BX, BY]], atol=1e-3)   # ball at rest

    def test_jacobian(self, fw):
        """EKF.cpp's F (via the reference) equals the finite-difference Jacobian of f."""
        x = np.array([10, -5, 0.7, 30, -12, 40, 60, 5, -3], float)
        ax, ay, dt = 0.3, -0.2, 0.02
        h2 = 0.5 * dt * dt
        a_r, a_f = ax * G_CM, ay * G_CM
        afx = math.cos(x[TH]) * a_r + math.sin(x[TH]) * a_f
        afy = -math.sin(x[TH]) * a_r + math.cos(x[TH]) * a_f
        F = np.eye(N)                              # exactly as written in EKF::predict
        F[X, VX] = F[Y, VY] = F[BX, BVX] = F[BY, BVY] = dt
        F[X, TH], F[Y, TH] = h2 * afy, -h2 * afx
        F[VX, TH], F[VY, TH] = afy * dt, -afx * dt
        np.testing.assert_allclose(F, numeric_jacobian(lambda s: RefEKF.f(s, ax, ay, dt), x),
                                   atol=1e-6)

    def test_uncertainty_grows(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0, positionStd=1.0, thetaStd=0.01)
        before = np.diag(ekf.getCovariance()).copy()
        for _ in range(100):
            ekf.predict(0.0, 0.0, 0.01)
        after = np.diag(ekf.getCovariance())
        assert np.all(after[[X, Y, TH, VX, VY, BX, BY]] > before[[X, Y, TH, VX, VY, BX, BY]])
        assert_valid_covariance(ekf.getCovariance())

    def test_pipeline_from_the_chip(self, sim, hw):
        """LSM303AGR → Accelerometer::read → predict: 0.5 s at (0.1 g right, 0.2 g forward)."""
        acc = sim.Accelerometer()
        acc.begin(sim.make_sensor_bus())
        hw.set_accel((0.1, 0.2))
        ekf = sim.EKF()
        th = math.radians(30)
        ekf.init(0.0, 0.0, th)
        for _ in range(50):
            ax, ay, az = acc.read()
            assert az == pytest.approx(1.0, abs=0.005)
            ekf.predict(ax, ay, 0.01)
        a = field_from_body(0.1, 0.2, th) * G_CM
        s = ekf.getState()
        assert (s.robotX, s.robotY) == pytest.approx(tuple(0.5 * a * 0.25), rel=0.01)


# ── compass: updateHeading ───────────────────────────────────────────────────────────────────

class TestCompassHeading:
    def test_matches_reference(self, fw):
        ekf, ref = pair(fw)
        for z in (0.5, 0.45, -0.2, 3.0, -3.1):
            ekf.updateHeading(z)
            ref.update_heading(z)
            ekf.predict(0.0, 0.0, 0.01)
            ref.predict(0.0, 0.0, 0.01)
        assert_matches(ekf, ref)

    def test_converges_and_shrinks_variance(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        var0 = ekf.getCovariance()[TH, TH]
        for _ in range(30):
            ekf.updateHeading(1.0)
        assert ekf.getState().robotTheta == pytest.approx(1.0, abs=0.01)
        assert ekf.getCovariance()[TH, TH] < var0 / 20

    def test_wraps_across_pi(self, fw):
        """θ = 3.1 rad and a reading of -3.1 rad are 0.08 rad apart through ±π, not 6.2 rad."""
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 3.1, thetaStd=0.05)
        ekf.updateHeading(-3.1)
        th = ekf.getState().robotTheta
        assert abs(th) > 3.05                     # stayed near ±π
        assert -PI_F <= th <= PI_F                # wrapped (to the firmware's float π)

    def test_only_heading_moves(self, fw):
        ekf = fw.EKF()
        ekf.init(5.0, 6.0, 0.0)
        ekf.updateHeading(0.4)
        s = ekf.getState()
        assert (s.robotX, s.robotY, s.speedX, s.speedY) == (5.0, 6.0, 0.0, 0.0)

    def test_pipeline_from_the_chip(self, sim, hw):
        """LIS2MDL → Compass driver (offset since reset, degrees CW) → updateHeading."""
        c = sim.Compass()
        c.begin(sim.make_sensor_bus())
        while not c.tick():
            sim.sim.advance_ns(1_000_000)
        c.reset()                                  # field heading 0 = where the robot started
        ekf = sim.EKF()
        ekf.init(0.0, 0.0, 0.0, thetaStd=0.5)
        for truth_deg in (0, 15, 40, 80, 120):
            hw.set_heading(-math.radians(truth_deg))      # sim heading is CCW
            for _ in range(100):                          # 2 s: let the filter settle
                sim.sim.advance_ns(20_000_000)
                assert c.update()
                ekf.predict(0.0, 0.0, 0.02)
                ekf.updateHeading(math.radians(c.getOffset()))
            assert math.degrees(ekf.getState().robotTheta) == pytest.approx(truth_deg, abs=2.5)


# ── sonar: updatePosition ────────────────────────────────────────────────────────────────────

class TestSonarPosition:
    def test_matches_reference(self, fw):
        ekf, ref = pair(fw)
        for z in ((12.0, -18.0), (14.0, -17.0), (11.0, -21.0)):
            ekf.updatePosition(*z)
            ref.update_position(*z)
            ekf.predict(0.05, 0.0, 0.02)
            ref.predict(0.05, 0.0, 0.02)
        assert_matches(ekf, ref)

    def test_converges_to_measurement(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        for _ in range(20):
            ekf.updatePosition(40.0, -30.0)
        s = ekf.getState()
        assert (s.robotX, s.robotY) == pytest.approx((40.0, -30.0), abs=0.5)
        P = ekf.getCovariance()
        assert P[X, X] < fw.EKFNoise().sonarPos ** 2   # better than one reading

    def test_first_update_gain(self, fw):
        """One reading with prior σ = 20 cm and sonar σ = 3 cm moves 400 / (400 + 9) of the way."""
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        ekf.updatePosition(100.0, 0.0)
        assert ekf.getState().robotX == pytest.approx(100.0 * 400 / 409, rel=1e-4)

    def test_position_correction_feeds_velocity_after_predict(self, fw):
        """Position and velocity become correlated through predict, so repeated position fixes
        of a moving robot teach the filter its speed."""
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0, positionStd=3.0)
        for k in range(1, 101):
            ekf.predict(0.0, 0.0, 0.02)
            ekf.updatePosition(50.0 * k * 0.02, 0.0)     # moving at 50 cm/s along +x
        assert ekf.getState().speedX == pytest.approx(50.0, abs=5.0)

    def test_pipeline_from_the_chip(self, sim, hw):
        """Ray-cast distances → echo pulses → Sonar driver → glue → updatePosition."""
        son = sim.Sonar()
        son.begin(sim.board.SONAR)
        truth_fw = np.array([-25.0, 40.0])                     # cm, firmware field frame
        pos_m = (truth_fw[1] / 100.0, -truth_fw[0] / 100.0)   # → sim frame
        model = sensors.SonarModel(noise_cm=0.0)
        ekf = sim.EKF()
        ekf.init(0.0, 0.0, 0.0)
        for _ in range(10):
            hw.set_sonar_cm(sensors.sonar_distances_cm(pos_m, 0.0, model))
            r = son.read()
            assert all(r.valid)
            pos = glue.sonar_localize(r.distance, r.valid, 0.0, (0.0, 0.0))
            ekf.updatePosition(*pos)
        s = ekf.getState()
        assert (s.robotX, s.robotY) == pytest.approx(tuple(truth_fw), abs=0.5)


# ── encoders: updateSpeed ────────────────────────────────────────────────────────────────────

class TestEncoderSpeed:
    def test_matches_reference(self, fw):
        ekf, ref = pair(fw)
        for z in ((20.0, 5.0), (25.0, 4.0), (-10.0, 0.0)):
            ekf.updateSpeed(*z)
            ref.update_speed(*z)
            ekf.predict(0.0, 0.1, 0.02)
            ref.predict(0.0, 0.1, 0.02)
        assert_matches(ekf, ref)

    def test_converges(self, fw):
        """init() starts the speed at 0 with σ = encoderSpeed — exactly the weight of one
        encoder reading — so n identical readings land n/(n+1) of the way."""
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        for _ in range(20):
            ekf.updateSpeed(-35.0, 60.0)
        s = ekf.getState()
        assert (s.speedX, s.speedY) == pytest.approx((-35.0 * 20 / 21, 60.0 * 20 / 21), rel=1e-4)
        for _ in range(50):                    # predict lets the speed move again
            ekf.predict(0.0, 0.0, 0.02)
            ekf.updateSpeed(-35.0, 60.0)
        s = ekf.getState()
        assert (s.speedX, s.speedY) == pytest.approx((-35.0, 60.0), abs=0.5)

    def test_speed_integrates_into_position(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0, positionStd=1.0)
        for _ in range(50):
            ekf.updateSpeed(0.0, 80.0)
            ekf.predict(0.0, 0.0, 0.02)
        assert ekf.getState().robotY == pytest.approx(80.0, abs=4.0)

    @pytest.mark.parametrize("th_deg", [0.0, 35.0, -120.0])
    def test_pipeline_from_the_chip(self, sim, hw, th_deg):
        """Wheel motion → quadrature counters → firmware encoder speed → glue → updateSpeed."""
        enc = sensors.EncoderModel()
        th = math.radians(th_deg)
        v_field = np.array([30.0, -50.0])                # cm/s, firmware field frame
        v_body = body_from_field(v_field, th) / 100.0    # m/s, (right, forward)
        rim = kin.wheel_rim_speeds(v_body, 0.0)
        for i in range(3):
            sim.encoder_init(i)
        hw.set_encoder_rates(sensors.encoder_rates(rim, enc))
        for _ in range(30):                      # the firmware's speed EMA (α = 0.3) settles
            sim.sim.advance_ns(20_000_000)
            for i in range(3):
                sim.encoder_update_speed(i)
        ekf = sim.EKF()
        ekf.init(0.0, 0.0, th)
        for _ in range(50):
            sim.sim.advance_ns(20_000_000)
            for i in range(3):
                sim.encoder_update_speed(i)
            ekf.predict(0.0, 0.0, 0.02)
            v = glue.encoder_field_velocity([sim.encoder_get_speed(i) for i in range(3)], th, enc)
            ekf.updateSpeed(float(v[0]), float(v[1]))
        s = ekf.getState()
        assert (s.speedX, s.speedY) == pytest.approx(tuple(v_field), abs=1.0)


# ── IR ball: updateBall ──────────────────────────────────────────────────────────────────────

class TestIRBall:
    def test_first_sighting_places_the_ball(self, fw):
        ekf = fw.EKF()
        ekf.init(10.0, 20.0, math.radians(90))          # facing field +x
        ekf.updateBall(math.radians(-90), 50.0)          # ball to the robot's left = field +y
        s = ekf.getState()
        assert (s.ballX, s.ballY) == pytest.approx((10.0, 70.0), abs=1e-3)
        assert (s.ballSpeedX, s.ballSpeedY) == (0.0, 0.0)
        P = ekf.getCovariance()
        n = fw.EKFNoise()
        assert P[BX, BX] == pytest.approx(n.irDistance ** 2 + 2500 * n.irAngle ** 2 + 400, rel=1e-4)
        assert P[BX, X] == 0.0                            # ball/robot correlation reset

    def test_matches_reference(self, fw):
        ekf, ref = pair(fw)
        for angle, dist in ((0.3, 60.0), (0.35, 58.0), (0.25, 63.0), (-3.0, 40.0)):
            ekf.updateBall(angle, dist)
            ref.update_ball(angle, dist)
            ekf.predict(0.0, 0.0, 0.02)
            ref.predict(0.0, 0.0, 0.02)
        assert_matches(ekf, ref, rtol=5e-3, atol=5e-3)

    def test_jacobian(self, fw):
        """The angle and distance rows written in EKF::updateBall equal the finite-difference
        Jacobian of h(x) = [atan2(dx, dy) - θ, |d|]."""
        x = np.array([10, -5, 0.7, 0, 0, 40, 60, 0, 0], float)
        dx, dy = x[BX] - x[X], x[BY] - x[Y]
        d2 = dx * dx + dy * dy
        d = math.sqrt(d2)
        angle_row = np.zeros(N)
        angle_row[[X, Y, TH, BX, BY]] = -dy / d2, dx / d2, -1.0, dy / d2, -dx / d2
        dist_row = np.zeros(N)
        dist_row[[X, Y, BX, BY]] = -dx / d, -dy / d, dx / d, dy / d
        J = numeric_jacobian(RefEKF.h_ball, x)
        np.testing.assert_allclose(angle_row, J[0], atol=1e-7)
        np.testing.assert_allclose(dist_row, J[1], atol=1e-7)

    def test_bearing_innovation_wraps(self, fw):
        """Ball almost straight behind: readings of +179° and -179° are 2° apart."""
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0, positionStd=1.0, thetaStd=0.01)
        ekf.updateBall(math.radians(179), 50.0)
        ekf.updateBall(math.radians(-179), 50.0)
        s = ekf.getState()
        assert s.ballY == pytest.approx(-50.0, abs=1.0) and abs(s.ballX) < 2.0

    def test_tracks_a_rolling_ball(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0, positionStd=1.0, thetaStd=0.01)
        ball, v = np.array([-40.0, 60.0]), np.array([40.0, 0.0])
        for _ in range(100):
            ekf.predict(0.0, 0.0, 0.02)
            ball = ball + v * 0.02
            ekf.updateBall(math.atan2(ball[0], ball[1]), float(np.hypot(*ball)))
            ekf.updateHeading(0.0)
            ekf.updatePosition(0.0, 0.0)
        s = ekf.getState()
        assert (s.ballX, s.ballY) == pytest.approx(tuple(ball), abs=1.0)
        assert (s.ballSpeedX, s.ballSpeedY) == pytest.approx(tuple(v), abs=5.0)

    def test_ball_on_the_robot_is_safe(self, fw):
        ekf = fw.EKF()
        ekf.init(0.0, 0.0, 0.0)
        ekf.updateBall(0.0, 0.0)
        ekf.updateBall(0.0, 0.0)          # d = 0: MIN_BALL_DISTANCE keeps the Jacobian finite
        assert np.all(np.isfinite(state_vec(ekf))) and np.all(np.isfinite(ekf.getCovariance()))

    @pytest.mark.parametrize("bearing_deg", [0.0, 40.0, -75.0])
    def test_pipeline_from_the_chip(self, sim, hw, bearing_deg):
        """IR ring → G port (mux, ADC, DMA) → GPort::readIR → glue → updateBall."""
        hw.cfg = type(hw.cfg)(ir=sensors.IRModel(noise=0.0))
        robot, th, dist = np.array([20.0, -30.0]), math.radians(25), 45.0
        hw.set_ir(math.radians(bearing_deg), dist)
        port = sim.GPort()
        assert port.begin(sim.board.G_PORT1, sim.GSensorKind.IR)
        sim.sim.advance_ns(150_000_000)
        meas = glue.ir_bearing_range(port.readIR())
        assert meas is not None
        ekf = sim.EKF()
        ekf.init(*robot, th, positionStd=0.5, thetaStd=0.01)
        for _ in range(5):
            ekf.updateBall(*meas)
        fa = th + math.radians(bearing_deg)
        truth = robot + dist * np.array([math.sin(fa), math.cos(fa)])
        s = ekf.getState()
        assert (s.ballX, s.ballY) == pytest.approx(tuple(truth), abs=4.0)


# ── the whole filter ─────────────────────────────────────────────────────────────────────────

def scenario(T=8.0, dt=0.01):
    """Robot on a figure-eight with a swaying heading; ball rolling across. Firmware field frame
    (cm, x right, y forward, θ CW). Yields (t, truth dict)."""
    w = 0.6
    for k in range(int(T / dt) + 1):
        t = k * dt
        p = np.array([60 * math.sin(w * t), 40 * math.sin(2 * w * t)])
        v = np.array([60 * w * math.cos(w * t), 80 * w * math.cos(2 * w * t)])
        a = np.array([-60 * w * w * math.sin(w * t), -160 * w * w * math.sin(2 * w * t)])
        th = 0.4 * math.sin(0.5 * t)
        ball = np.array([-80.0 + 20.0 * t, 70.0])
        yield t, {"p": p, "v": v, "a": a, "th": th, "ball": ball, "bv": np.array([20.0, 0.0])}


def run_fusion(fw, rng, sensors_on=("accel", "compass", "sonar", "encoder", "ir"), dt=0.01):
    n = fw.EKFNoise()
    ekf = fw.EKF()
    ekf.init(0.0, 0.0, 0.0)
    errs, nees = [], []
    for k, (t, tr) in enumerate(scenario(dt=dt)):
        if k == 0:
            continue
        # accelerometer at 100 Hz drives predict (the robot-frame acceleration it would feel)
        a_body = body_from_field(tr["a"], tr["th"]) if "accel" in sensors_on else np.zeros(2)
        a_meas = a_body / G_CM + rng.normal(0, n.accel / G_CM, 2)
        ekf.predict(float(a_meas[0]), float(a_meas[1]), dt)
        if "compass" in sensors_on and k % 2 == 0:                       # 50 Hz
            ekf.updateHeading(tr["th"] + rng.normal(0, n.compassTheta))
        if "sonar" in sensors_on and k % 5 == 0:                         # 20 Hz
            z = tr["p"] + rng.normal(0, n.sonarPos, 2)
            ekf.updatePosition(float(z[0]), float(z[1]))
        if "encoder" in sensors_on and k % 2 == 0:                       # 50 Hz
            z = tr["v"] + rng.normal(0, n.encoderSpeed, 2)
            ekf.updateSpeed(float(z[0]), float(z[1]))
        if "ir" in sensors_on and k % 3 == 0:                            # ~33 Hz
            d = tr["ball"] - tr["p"]
            ekf.updateBall(math.atan2(d[0], d[1]) - tr["th"] + rng.normal(0, n.irAngle),
                           float(np.hypot(*d)) + rng.normal(0, n.irDistance))
        if t >= 2.0:                                                     # after convergence
            x = state_vec(ekf)
            truth = np.array([*tr["p"], tr["th"], *tr["v"], *tr["ball"], *tr["bv"]])
            e = x - truth
            e[TH] = wrap(e[TH])
            errs.append(e)
            P = ekf.getCovariance()[:5, :5]
            nees.append(float(e[:5] @ np.linalg.solve(P, e[:5])))
        assert_valid_covariance(ekf.getCovariance())
    return ekf, np.array(errs), np.array(nees)


class TestWholeFilter:
    def test_all_sensors_track_the_truth(self, fw):
        _, errs, _ = run_fusion(fw, np.random.default_rng(7))
        rms = np.sqrt(np.mean(errs ** 2, axis=0))
        assert rms[X] < 3.0 and rms[Y] < 3.0                 # cm (sonar alone: σ = 3)
        assert rms[TH] < math.radians(3.0)
        assert rms[VX] < 6.0 and rms[VY] < 6.0               # cm/s (encoders alone: σ = 5)
        assert rms[BX] < 6.0 and rms[BY] < 6.0               # cm (one IR reading: ~10 cm)
        assert rms[BVX] < 25.0 and rms[BVY] < 25.0

    def test_fusion_beats_each_sensor_alone(self, fw):
        """The fused position is better than the sonar's own noise, the fused speed better than
        the encoders' — what an EKF is for."""
        n = fw.EKFNoise()
        _, errs, _ = run_fusion(fw, np.random.default_rng(3))
        rms = np.sqrt(np.mean(errs ** 2, axis=0))
        assert rms[X] < n.sonarPos and rms[Y] < n.sonarPos
        assert rms[VX] < n.encoderSpeed and rms[VY] < n.encoderSpeed

    def test_filter_is_consistent(self, fw):
        """Normalised estimation error squared of the robot state (x, y, θ, vx, vy): its mean
        should be near 5 when the covariance honestly describes the error. Far above means the
        filter is overconfident (e.g. a missing process-noise term), far below too timid."""
        nees = np.concatenate([run_fusion(fw, np.random.default_rng(s))[2] for s in range(3)])
        assert 1.0 < float(np.mean(nees)) < 15.0

    def test_dead_reckoning_without_sonar(self, fw):
        """Without position fixes, position drifts but stays bounded by the speed updates, and
        the filter knows it is less sure."""
        ekf_all, errs_all, _ = run_fusion(fw, np.random.default_rng(5))
        ekf_dr, errs_dr, _ = run_fusion(fw, np.random.default_rng(5),
                                        sensors_on=("accel", "compass", "encoder", "ir"))
        assert np.sqrt(np.mean(errs_dr[:, X] ** 2)) > np.sqrt(np.mean(errs_all[:, X] ** 2))
        assert ekf_dr.getCovariance()[X, X] > 4 * ekf_all.getCovariance()[X, X]
        assert np.sqrt(np.mean(errs_dr[:, VX] ** 2)) < 6.0     # speed still pinned by encoders

    def test_no_compass_heading_drifts_but_is_admitted(self, fw):
        ekf, errs, _ = run_fusion(fw, np.random.default_rng(9),
                                  sensors_on=("accel", "sonar", "encoder", "ir"))
        n = fw.EKFNoise()
        assert ekf.getCovariance()[TH, TH] > n.compassTheta ** 2   # uncertainty grew

    def test_whole_filter_matches_reference(self, fw):
        """The full fused run, step for step, equals the float64 reference model."""
        rng = np.random.default_rng(11)
        ekf, ref = pair(fw, 0.0, 0.0, 0.0)
        n = fw.EKFNoise()
        for k, (_, tr) in enumerate(scenario(T=2.0)):
            if k == 0:
                continue
            a = body_from_field(tr["a"], tr["th"]) / G_CM + rng.normal(0, 0.02, 2)
            for f in (ekf.predict, ref.predict):
                f(float(a[0]), float(a[1]), 0.01)
            if k % 2 == 0:
                z = tr["th"] + rng.normal(0, n.compassTheta)
                ekf.updateHeading(z)
                ref.update_heading(z)
            if k % 5 == 0:
                z = tr["p"] + rng.normal(0, n.sonarPos, 2)
                ekf.updatePosition(*z)
                ref.update_position(*z)
                zv = tr["v"] + rng.normal(0, n.encoderSpeed, 2)
                ekf.updateSpeed(*zv)
                ref.update_speed(*zv)
            if k % 3 == 0:
                d = tr["ball"] - tr["p"]
                ang = math.atan2(d[0], d[1]) - tr["th"]
                ekf.updateBall(ang, float(np.hypot(*d)))
                ref.update_ball(ang, float(np.hypot(*d)))
        assert_matches(ekf, ref, rtol=2e-2, atol=2e-2)


@pytest.mark.xfail(strict=True, reason=(
    "EKF has no angular-rate state and models heading as a slow random walk (thetaDrift = "
    "0.05 rad/sqrt(s)): turning at 90 deg/s it trails a perfect compass by ~12 deg while "
    "reporting sigma ~1 deg. Add omega to the state (or raise thetaDrift / feed the gyro or "
    "encoder rotation) and remove this marker."))
def test_heading_keeps_up_with_a_turning_robot(fw):
    """Perfect compass at 50 Hz, robot turning at 90°/s: the heading error should stay within
    3 σ of what the filter itself claims."""
    ekf = fw.EKF()
    ekf.init(0.0, 0.0, 0.0)
    w = math.radians(90.0)
    worst = 0.0
    for k in range(1, 301):
        t = k * 0.01
        ekf.predict(0.0, 0.0, 0.01)
        if k % 2 == 0:
            ekf.updateHeading(wrap(w * t))
        if t > 1.0:
            err = abs(wrap(ekf.getState().robotTheta - w * t))
            worst = max(worst, err / math.sqrt(ekf.getCovariance()[TH, TH]))
    assert worst < 3.0
