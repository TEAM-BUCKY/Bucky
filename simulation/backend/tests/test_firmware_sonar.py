"""Sonar (trigger pulse → echo edges → EXTI timing) and buttons (EXTI + debounce)."""
from __future__ import annotations

import math
import re

import pytest

from bucky.firmware import sensors
from bucky.game.field import ARENA_HALF_X, ARENA_HALF_Y, ROBOT_RADIUS


def test_sonar_read(sim, hw):
    s = sim.Sonar()
    s.begin(sim.board.SONAR)
    hw.set_sonar_cm([50.0, 80.0, 123.4, 7.5])
    r = s.read()
    assert r.valid == [True] * 4
    assert r.distance == pytest.approx([50.0, 80.0, 123.4, 7.5], abs=0.02)
    assert sim.devices.sonar.pings == 1


def test_sonar_timeout_and_missing_echo(sim, hw):
    s = sim.Sonar()
    s.begin(sim.board.SONAR)
    hw.set_sonar_cm([math.nan, 60.0, 400.0, math.nan])   # 400 cm → 23.5 ms > 20 ms timeout
    t0 = sim.sim.now_ns()
    r = s.read()
    assert r.valid == [False, True, False, False]
    assert r.distance[1] == pytest.approx(60.0, abs=0.02)
    assert (sim.sim.now_ns() - t0) / 1e6 == pytest.approx(20.0, abs=0.1)


def test_sonar_non_blocking_read(sim, hw):
    s = sim.Sonar()
    s.begin(sim.board.SONAR)
    hw.set_sonar_cm([30.0] * 4)
    s.startRead()
    assert not s.isReadComplete()
    sim.sim.advance_ns(3_000_000)
    assert s.isReadComplete()
    assert s.processRead().distance == pytest.approx([30.0] * 4, abs=0.02)


def test_sonar_model_in_the_arena():
    d = sensors.sonar_distances_cm((0.0, 0.0), 0.0, sensors.SonarModel(noise_cm=0.0))
    rim = ROBOT_RADIUS * 100
    assert d == pytest.approx([ARENA_HALF_X * 100 - rim, ARENA_HALF_Y * 100 - rim,
                               ARENA_HALF_X * 100 - rim, ARENA_HALF_Y * 100 - rim])
    blocked = sensors.sonar_distances_cm((0.0, 0.0), 0.0, sensors.SonarModel(noise_cm=0.0),
                                         robots=[(0.5, 0.0)])
    assert blocked[0] == pytest.approx(50.0 - 2 * rim)


def test_sonar_program(hw, run_program):
    hw.set_sonar_cm([42.0, math.nan, 99.0, 12.3])
    out = run_program("testSonar", 1.0)
    rows = re.findall(r"S0: (\S+) cm\tS1: TIMEOUT\tS2: (\S+) cm\tS3: (\S+) cm", out)
    assert rows and rows[-1] == ("42.0", "99.0", "12.3")


def test_button_debounce(sim, hw):
    b = sim.Button()
    assert b.begin(sim.board.BUTTON1)
    assert not b.pressed()
    hw.press_button(1, hold_ms=100)
    sim.sim.advance_ns(5_000_000)
    assert not b.pressed()                     # within 25 ms of the last edge: still bouncing
    sim.sim.advance_ns(30_000_000)
    assert b.pressed() and not b.pressed()     # one press, reported once


def test_buttons_in_main_loop(sim, hw, run_program):
    out = run_program("main_loop", 0.8)
    hw.press_button(2, hold_ms=60)
    sim.sim.run_until_ns(sim.sim.now_ns() + 200_000_000)
    assert "Button 2 pressed" in sim.sim.usb_read().decode()
    assert "Button 1 pressed" not in out
