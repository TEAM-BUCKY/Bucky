#include "devices/ir_frontend.h"

#include <algorithm>
#include <cmath>

namespace sim {

namespace {
constexpr double NS = 1e-9;
}

double IrFrontEnd::uniform() {
    return static_cast<double>(rng_() >> 11) * 0x1.0p-53;   // portable, unlike std:: distributions
}

double IrFrontEnd::normal() {
    const double u1 = std::max(uniform(), 1e-300), u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

double IrFrontEnd::envelope(const double u) const {
    if (u <= 0.0) return 0.0;
    const double x = u / params.tau_bp_s;
    if (params.envelope_order >= 2) return 1.0 - (1.0 + x) * std::exp(-x);
    return 1.0 - std::exp(-x);
}

double IrFrontEnd::amplitude(const int k) const {
    const double headroom = std::max(0.0, params.i_sat_a - i_ambient[k]);
    return params.gain_v_per_a * std::min(std::max(i_on_[k], 0.0), headroom);
}

void IrFrontEnd::reset(const uint64_t now_ns) {
    rng_.seed(seed_);
    v_.fill(0.0);
    t_last_ = static_cast<double>(now_ns) * NS;
    blank_until_ = -1.0;
    s_cur_ = -1e300;
    n_ = 0;
    phase_ = t_last_ + uniform() * burst.period_s;
    s_next_ = draw_next();
}

double IrFrontEnd::draw_next() {
    const double period = burst.period_s * (1.0 + burst.period_ppm * 1e-6);
    const double jitter = burst.jitter_s > 0.0 ? (2.0 * uniform() - 1.0) * burst.jitter_s : 0.0;
    double s = phase_ + static_cast<double>(n_) * period + jitter;
    if (s_cur_ > -1e299) s = std::max(s, s_cur_ + burst.length_s() + 1e-9);
    s = std::max(s, t_last_);
    return s;
}

void IrFrontEnd::advance(double t0, const double t1) {
    if (t1 <= t0) return;
    if (t1 <= blank_until_) {   // RESET still holding the caps at 0
        v_.fill(0.0);
        return;
    }
    if (t0 < blank_until_) {
        v_.fill(0.0);
        t0 = blank_until_;
    }
    const double droop = std::exp(-(t1 - t0) / params.tau_hold_s);
    const double s = s_cur_, end = s + burst.length_s();
    const double u_star = std::min(t1, end);
    const bool charging = u_star >= t0 && t1 > s;
    // A·g(u)·e^{u/τh} rises over the burst, so the held value is set at u* (the latest burst
    // instant inside this segment) and then droops to t1.
    const double g = charging ? envelope(u_star - s) * std::exp(-(t1 - u_star) / params.tau_hold_s) : 0.0;
    for (int k = 0; k < CHANNELS; k++) {
        double v = v_[k] * droop;
        if (charging) v = std::max(v, amplitude(k) * g);
        v_[k] = v;
    }
}

void IrFrontEnd::catch_up(const double t) {
    while (s_next_ <= t) {
        advance(t_last_, s_next_);
        t_last_ = std::max(t_last_, s_next_);
        s_cur_ = s_next_;
        n_++;
        s_next_ = draw_next();
    }
    advance(t_last_, t);
    t_last_ = std::max(t_last_, t);
}

void IrFrontEnd::set_drive(const uint64_t now_ns, const std::array<double, CHANNELS>& i_on_a) {
    catch_up(static_cast<double>(now_ns) * NS);
    i_on_ = i_on_a;
}

void IrFrontEnd::on_reset_pulse(const uint64_t now_ns) {
    const double t = static_cast<double>(now_ns) * NS;
    catch_up(t);
    v_.fill(0.0);
    blank_until_ = t + params.reset_width_s;
}

uint16_t IrFrontEnd::sample(const uint64_t now_ns, const uint32_t channel) {
    if (channel >= CHANNELS) return 0;
    const double t = static_cast<double>(now_ns) * NS;
    catch_up(t);
    const double v = (t < blank_until_ ? 0.0 : v_[channel]) + offset_v[channel] +
                     (params.noise_v > 0.0 ? params.noise_v * normal() : 0.0);
    double code = v * 4095.0 / params.vref;
    if (params.adc_noise_lsb > 0.0) code += params.adc_noise_lsb * normal();
    return static_cast<uint16_t>(std::clamp(std::lround(code), 0L, 4095L));
}

std::array<double, IrFrontEnd::CHANNELS> IrFrontEnd::peek(const uint64_t t_ns) const {
    IrFrontEnd copy = *this;
    const double t = static_cast<double>(t_ns) * NS;
    copy.catch_up(t);
    if (t < copy.blank_until_) copy.v_.fill(0.0);
    return copy.v_;
}

}  // namespace sim
