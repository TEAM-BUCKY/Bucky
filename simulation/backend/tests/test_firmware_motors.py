"""Motors, encoders and kicker: firmware PWM/timer code against the simulated timers."""
from __future__ import annotations

import pytest

from bucky.firmware import kinematics as kin
from bucky.firmware import sensors


def make_motors(fw, encoders=False):
    b = fw.board
    m = fw.Motors(b.MOTOR1, b.MOTOR2, b.MOTOR3)
    m.init(encoders=encoders)
    return m


def duties(fw):
    pads = [p for pair in fw.board.MOTOR_PINS for p in pair]
    return [fw.sim.pad_duty(p) for p in pads]


def test_direct_drive_duty_mapping(sim):
    m = make_motors(sim)
    # pwm_init: PSC = 250 MHz / (5 kHz · 3400) - 1 truncates to 13 → 5.25 kHz, not 5 kHz.
    assert sim.sim.pad_period_ns(sim.board.MOTOR1.inA) == pytest.approx(14 * 3400 / 250e6 * 1e9)
    m.driveMotorsDirect(50, -30, 0.04)       # 0.04 % is inside the stop deadband
    m.updateAllMotors()
    model = kin.MotorModel()
    assert duties(sim) == pytest.approx([model.duty_for_percent(50), 0.0,
                                         0.0, model.duty_for_percent(30),
                                         0.0, 0.0], abs=1e-6)


def test_staged_update_matches_immediate(sim):
    m = make_motors(sim)
    m.driveMotorsDirect(-80, 20, 100)
    m.syncUpdateAllMotors()
    staged = duties(sim)
    m.updateAllMotors()
    assert duties(sim) == pytest.approx(staged)
    assert staged[1] > 0 and staged[2] > 0 and staged[4] == pytest.approx(1.0, abs=1e-3)


def test_smooth_ramp_on_drive(sim):
    m = make_motors(sim)
    m.driveDegrees(0, 50, 0)                 # M1 → -43.3 %, ramp over |0 - 50| · 30 ms = 1.5 s
    sim.sim.advance_ns(750_000_000)
    m.updateAllMotors()
    want = kin.MotorModel().duty_for_percent(43.3 / 2)
    assert duties(sim)[1] == pytest.approx(want, abs=0.01)     # M1 reverse pin, half-way
    sim.sim.advance_ns(1_000_000_000)
    m.updateAllMotors()
    assert duties(sim)[1] == pytest.approx(kin.MotorModel().duty_for_percent(43.3), abs=0.002)


def test_encoder_counts_and_wraps(sim, hw):
    for i in range(3):
        sim.encoder_init(i)
        assert sim.encoder_backend(i) == sim.EncoderBackend.Timer
    hw.set_encoder_rates([100_000.0, -100_000.0, 0.0])
    for _ in range(10):                      # 10 000 ticks per sample; the 16-bit CNT wraps
        sim.sim.advance_ns(100_000_000)
        ticks = [sim.encoder_get_ticks(i) for i in range(3)]
    assert ticks == [pytest.approx(100_000, abs=2), pytest.approx(-100_000, abs=2), 0]
    assert sim.encoder_is_active(0) and not sim.encoder_is_active(2)


def _run_pi(fw, hw, gains, plant_gain=0.8, seconds=3.0):
    """M2 at 50 % against a wheel that only reaches plant_gain of the nominal speed."""
    m = make_motors(fw, encoders=True)
    m.setPIGains(*gains)
    model = kin.MotorModel(polarity=(1.0, 1.0, 1.0), max_rim_mps=kin.MAX_LINEAR * plant_gain)
    enc = sensors.EncoderModel()
    m.driveMotorsDirect(0, 50, 0)
    rim = [0.0, 0.0, 0.0]
    dt = 0.005
    for _ in range(int(seconds / dt)):
        hw.set_encoder_rates(sensors.encoder_rates(rim, enc))
        fw.sim.advance_ns(int(dt * 1e9))
        m.updateAllMotors()
        target = model.rim_speeds([[fw.sim.pad_duty(a), fw.sim.pad_duty(b)]
                                   for a, b in fw.board.MOTOR_PINS])
        rim = [r + (t - r) * dt / 0.05 for r, t in zip(rim, target)]   # 50 ms motor lag
    return m.getEncoderSpeeds()[1]


def test_speed_loop_reduces_error(sim, hw):
    open_loop = _run_pi(sim, hw, (0.0, 0.0, 0.0))
    assert open_loop == pytest.approx(40.0, abs=1.0)   # 80 % plant → 40 % of nominal
    sim.sim.reboot()
    closed = _run_pi(sim, hw, (0.5, 0.05, 30.0))
    assert abs(50.0 - closed) < abs(50.0 - open_loop) * 0.75


def test_kicker(sim):
    k = sim.Kicker(sim.board.KICKER)
    k.init()
    assert sim.sim.pad_duty(sim.board.KICKER) == 0.0
    k.kick(0.5)
    assert sim.sim.pad_duty(sim.board.KICKER) == pytest.approx(0.5, abs=1e-3)
    k.kick(3.0)                               # clamped to full power
    assert sim.sim.pad_duty(sim.board.KICKER) == pytest.approx(1.0, abs=1e-3)
    k.stop()
    assert sim.sim.pad_duty(sim.board.KICKER) == 0.0


def test_pwm_probe_averages_over_time(sim):
    m = make_motors(sim)
    probe = sim.devices.pwm
    probe.take_average()
    m.driveMotorsDirect(100, 0, 0)
    m.updateAllMotors()
    sim.sim.advance_ns(10_000_000)
    m.driveMotorsDirect(0, 0, 0)
    m.updateAllMotors()
    sim.sim.advance_ns(10_000_000)
    avg = probe.take_average()
    assert avg[0] == pytest.approx(0.5 * kin.MotorModel().duty_for_percent(100), abs=0.01)
