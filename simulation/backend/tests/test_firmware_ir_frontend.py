"""The IR boards' time-domain front end (robot/sim/src/devices/ir_frontend.cpp) under the real
GPort driver: burst timing, peak hold, droop, RESET, clipping and occlusion."""
from __future__ import annotations

import math

import numpy as np
import pytest


def begin_ir(fw, rate_hz: int | None = None):
    port = fw.GPort()
    kind = fw.GSensorKind.IR
    if rate_hz is None:
        assert port.begin(fw.board.G_PORT1, kind)
    else:
        assert port.begin_timed(fw.board.G_PORT1, kind, rate_hz)
    return port


def frames(fw, port, n: int, max_s: float = 10.0) -> np.ndarray:
    """The next ``n`` complete frames (physical order)."""
    out, seq = [], port.frameSequence()
    t_end = fw.sim.now_ns() + int(max_s * 1e9)
    while len(out) < n and fw.sim.now_ns() < t_end:
        fw.sim.advance_ns(500_000)
        if port.frameSequence() != seq:
            seq = port.frameSequence()
            out.append(port.readIR())
    assert len(out) == n
    return np.array(out, dtype=float)


def wait_reset(fw, port) -> None:
    seq = port.frameSequence()
    while port.frameSequence() == seq:
        fw.sim.advance_ns(100_000)


def quiet(fe, *, jitter: float = 0.0, ppm: float = 0.0):
    b = fe.burst
    b.jitter_s, b.period_ppm = jitter, ppm
    fe.burst = b
    p = fe.params
    p.noise_v, p.adc_noise_lsb = 0.0, 0.0
    fe.params = p


def test_hold_follows_the_burst_envelope_and_droops(sim, hw):
    fe = hw.ir_board.ir
    quiet(fe)
    fe.reset()
    g = fe.params.gain_v_per_a
    fe.set_drive([1.0 / g] + [0.0] * 15)                 # 1 V carrier amplitude on channel 0
    s, tau = fe.next_burst_start_s, fe.params.tau_bp_s
    length = fe.burst.length_s
    t = lambda sec: int(round(sec * 1e9))
    assert fe.peek(t(s + 10e-6))[0] == pytest.approx(1 - math.exp(-10e-6 / tau), abs=1e-3)
    full = fe.peek(t(s + length))[0]
    assert full == pytest.approx(1 - math.exp(-length / tau), abs=1e-6)
    sim.sim.advance_ns(t(s + length + 1e-6) - sim.sim.now_ns())
    fe.set_drive([0.0] * 16)                             # ball gone: only droop now
    assert fe.peek(t(s + length + 5e-3))[0] == pytest.approx(full * math.exp(-0.5), rel=1e-3)
    assert fe.amplitude(0) == 0.0


def test_idle_ring_reads_the_offset(sim, hw):
    hw.set_ir(None)
    port = begin_ir(sim)
    f = frames(sim, port, 3)
    assert np.all(f[:, :12] == round(3e-3 * 4095 / 3.3)) and np.all(f[:, 12:] == 0)


def test_steady_ball_reads_its_peak_with_at_most_one_burst_of_droop(sim, hw):
    ideal = hw.set_ir(0.0, 61.0)                           # 50 cm from sensor 0
    base = hw.cfg.ir.baseline
    port = begin_ir(sim)
    f = frames(sim, port, 60)[:, 0]
    amp, top = f - base, ideal[0] - base
    droop = math.exp(-hw.cfg.ir.ball.burst_period_s / hw.cfg.ir.front_end.tau_hold_s)
    assert np.all(amp <= top + 1) and np.all(amp >= droop * top - 1)


def test_reset_clears_the_holds_not_the_droop(sim, hw):
    hw.set_ir(0.0, 61.0)
    port = begin_ir(sim)
    frames(sim, port, 3)
    wait_reset(sim, port)
    hw.set_ir(None)                                        # sensor 0 is sampled 12.5 ms later
    wait_reset(sim, port)
    f = np.array(port.readIR(), float)
    # 12.5 ms of droop alone would leave ~0.29 × 230 ≈ 66 codes on sensor 0.
    assert np.all(f[:12] == round(3e-3 * 4095 / 3.3))


def test_first_slot_after_reset_underreads_when_the_clock_is_fast(sim, hw):
    """Mux index 0 shows physical sensor 7, sampled (1 − duty)·T after RESET. At 2400 Hz that is
    208 µs — shorter than the 833 µs burst period, so it often holds nothing."""
    base = hw.cfg.ir.baseline

    def fraction_good(rate: int, sensor: int) -> float:
        sim.sim.reboot()
        quiet(hw.ir_board.ir, jitter=5e-6, ppm=3000.0)      # ball clock drifts → phase sweeps
        bearing = math.remainder(math.radians(30.0 * sensor), math.tau)
        ideal = hw.set_ir(bearing, 61.0)                     # on that sensor's axis
        port = begin_ir(sim, rate)
        f = frames(sim, port, 150)
        return float(np.mean(f[:, sensor] - base >= 0.9 * (ideal[sensor] - base)))

    assert fraction_good(2400, 7) < 0.6
    assert fraction_good(2400, 0) > 0.98                    # mux index 7: 3.1 ms after RESET
    assert fraction_good(600, 7) > 0.95


def test_burst_phase_locks_to_a_frame_when_the_ball_clock_is_exact(sim, hw):
    quiet(hw.ir_board.ir)
    sim.sim.reboot()
    hw.set_ir(math.radians(210.0), 61.0)
    port = begin_ir(sim, 2400)                              # frame = 6.67 ms = 8 bursts exactly
    f = frames(sim, port, 60)[:, 7]
    assert f.max() - f.min() <= 1


def test_same_seed_same_frames(sim, hw):
    fe = hw.ir_board.ir
    p = fe.params
    p.adc_noise_lsb = 1.0
    fe.params = p

    def run(seed: int):
        fe.seed(seed)
        sim.sim.reboot()
        hw.set_ir(0.3, 80.0)
        port = begin_ir(sim)
        return frames(sim, port, 10), fe.last_burst_start_s

    a, sa = run(7)
    b, sb = run(7)
    c, sc = run(8)
    assert np.array_equal(a, b) and sa == sb
    assert sc != sa


def test_tia_clips_close_up(sim, hw):
    hw.set_ir(0.0, 21.0)                                    # 10 cm from sensor 0
    port = begin_ir(sim)
    f = frames(sim, port, 30)[:, 0]
    ceiling = hw.cfg.ir.clip_code
    assert f.max() <= ceiling + 1 and f.max() >= ceiling - 30


def test_robot_between_ball_and_ring(sim, hw):
    hw.set_ir_scene((0.0, 0.0), 0.0, (0.6, 0.0), robots=[(0.35, 0.0)])
    port = begin_ir(sim)
    f = frames(sim, port, 3)
    assert np.all(f[:, [11, 0, 1]] == round(3e-3 * 4095 / 3.3))
