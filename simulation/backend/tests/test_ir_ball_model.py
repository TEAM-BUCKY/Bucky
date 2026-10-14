"""IR ball + ring physics (bucky.firmware.ir_ball): numbers from the PCB and the ball's design."""
from __future__ import annotations

import math

import numpy as np
import pytest

from bucky.firmware.ir_ball import (BallOrientation, IRBall, IRFrontEnd, IRRingGeometry,
                                    IRRingModel, fibonacci_sphere, with_ball)
from bucky.game.field import ROBOT_RADIUS


def ball_at(bearing_cw_deg: float, centre_cm: float) -> tuple[float, float]:
    """Sim-frame ball position for a robot at the origin facing +x."""
    b = math.radians(bearing_cw_deg)
    return math.cos(b) * centre_cm / 100, -math.sin(b) * centre_cm / 100


def test_chain_gain_from_the_transfer_functions():
    fe = IRFrontEnd()
    assert fe.chain_gain(40e3) == pytest.approx(1.225e5, rel=0.01)
    assert fe.chain_gain(41.67e3) == pytest.approx(1.207e5, rel=0.01)
    assert abs(fe.h_bp(fe.bp_f0_hz)) == pytest.approx(1.0)
    assert fe.tau_bp() == pytest.approx(7.92e-6, rel=0.01)


def test_tia_ceiling():
    fe = IRFrontEnd()
    assert fe.i_sat() == pytest.approx(15.9e-6, rel=0.01)
    f = IRBall().carrier_hz
    assert 2370 <= fe.code_of([1e-4], f)[0] <= 2400
    lit = IRFrontEnd(ambient_a=5e-6)       # room light eats the headroom
    assert lit.clip_code(f) == pytest.approx(fe.clip_code(f) * (1 - 5e-6 / fe.i_sat()), rel=0.01)


def test_ball_power_and_k_match_the_firmware(fw):
    m = IRRingModel()
    assert m.ball.total_w == pytest.approx(3.96, rel=0.01)
    assert m.ball.iso_w_sr == pytest.approx(0.315, rel=0.01)
    assert m.range_k == pytest.approx(fw.IRConfig().kCm, rel=0.03)
    assert m.baseline == pytest.approx(fw.IRConfig().baselineCode, abs=0.5)
    assert m.clip_code > fw.IRConfig().clipCode       # the firmware can see a clip


def test_on_axis_square_law():
    m = IRRingModel()
    f = m.ball.carrier_hz
    for d_sensor in (20.0, 50.0, 150.0):
        i = m.photocurrents((0, 0), 0.0, ball_at(0.0, d_sensor + ROBOT_RADIUS * 100))
        amp = m.front_end.code_of(i, f)[0] - m.baseline
        assert amp == pytest.approx((m.range_k / d_sensor) ** 2, rel=0.01, abs=0.6)


def test_angular_response_half_at_15_degrees():
    m = IRRingModel(geometry=IRRingGeometry(mount_radius_m=0.0, own_body_radius_m=0.0))  # no parallax
    on = m.photocurrents((0, 0), 0.0, ball_at(0.0, 300))[0]
    assert m.photocurrents((0, 0), 0.0, ball_at(15.0, 300))[0] / on == pytest.approx(0.5, abs=0.01)
    assert m.photocurrents((0, 0), 0.0, ball_at(95.0, 300))[0] == 0.0


def test_channels_past_the_ring_are_dark():
    i = IRRingModel().photocurrents((0, 0), 0.0, ball_at(0.0, 30))
    assert np.all(i[12:] == 0)


def test_rolling_led_ball_matches_isotropic_on_average():
    iso = IRRingModel()
    leds = with_ball(iso, mode="leds")
    rng = np.random.default_rng(1)
    ref = iso.photocurrents((0, 0), 0.0, ball_at(0.0, 100))[0]
    v = np.array([leds.photocurrents((0, 0), 0.0, ball_at(0.0, 100),
                                     BallOrientation.random(rng).rot)[0] for _ in range(500)])
    assert v.mean() == pytest.approx(ref, rel=0.03)
    assert (v.max() - v.min()) / v.mean() < 0.08       # 42 LEDs: ~4 % ripple


def test_fibonacci_leds_are_unit_and_spread():
    d = fibonacci_sphere(42)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    assert np.linalg.norm(d.mean(axis=0)) < 0.05


def test_rolling_one_circumference_is_a_full_turn():
    o = BallOrientation()
    r = 0.021
    for _ in range(1000):
        o.advance((1.0, 0.0), 2 * math.pi * r / 1000, r)
    assert np.allclose(o.rot, np.eye(3), atol=1e-9)
    o = BallOrientation()
    o.advance((1.0, 0.0), math.pi / 2 * r, r)            # quarter turn about ẑ × x̂ = +ŷ
    assert np.allclose(o.rot @ [0, 0, 1], [1, 0, 0], atol=1e-12)


def test_robot_in_the_way_blocks_the_ball():
    m = IRRingModel()
    clear = m.photocurrents((0, 0), 0.0, (0.6, 0.0))
    blocked = m.photocurrents((0, 0), 0.0, (0.6, 0.0), robots=[(0.35, 0.0)])
    assert clear[0] > 0 and blocked[0] == 0 and blocked[11] == 0 and blocked[1] == 0
    leds = with_ball(m, mode="leds").photocurrents((0, 0), 0.0, (0.6, 0.0), robots=[(0.35, 0.0)])
    assert leds[0] == 0


def test_own_body_shadows_the_far_side():
    m = IRRingModel(geometry=IRRingGeometry(cos_power=0.0))    # omnidirectional sensors
    i = m.photocurrents((0, 0), 0.0, ball_at(0.0, 60))
    assert i[0] > 0 and np.all(i[[5, 6, 7]] == 0)


def test_sensor_height_costs_cos_n_of_the_elevation():
    flat = IRRingModel()
    dz = 0.05
    high = IRRingModel(geometry=IRRingGeometry(sensor_height_m=flat.ball.center_height_m + dz))
    d = 0.4
    ratio = (high.photocurrents((0, 0), 0.0, (ROBOT_RADIUS + d, 0))[0]
             / flat.photocurrents((0, 0), 0.0, (ROBOT_RADIUS + d, 0))[0])
    el = math.atan2(dz, d)
    assert ratio == pytest.approx(math.cos(el) ** 20 * d ** 2 / (d ** 2 + dz ** 2), rel=1e-6)
