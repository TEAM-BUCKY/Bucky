// Time-domain model of the IR ring's analog front end, one per IR G-port board.
//
// Per sensor (IR_SENSOR board): VEMD2020X01 photodiode → 220k‖10p TIA → two 40.19 kHz Q=1 MFB
// band-pass stages → precision peak detector holding on 10 nF with a 1 MΩ droop resistor. The
// controller's 4040 pulses RESET at every frame wrap, which dumps all holds at once.
//
// The ball (RCJ IR golf ball) sends bursts of `pulses` carrier cycles every `period_s`. Python
// sets each sensor's LED-on photocurrent (optics, occlusion, rolling LEDs); this class turns it
// into the held voltage at the exact instant the ADC samples it:
//
//   A_k   = G · min(i_on, i_sat − i_ambient)            carrier amplitude after the band-pass
//   env   = A_k · g(u),  g(u) = 1 − e^{−u/τbp}          u = time since the burst started
//   V_k   = max over the burst of env, then droops with e^{−t/τh}; RESET forces V_k = 0
//   code  = round((V_k + offset_k + noise) · 4095 / vref)
//
// Evaluation is lazy and exact for any split of time: catch_up(t) walks the burst schedule.
#pragma once

#include <array>
#include <cstdint>
#include <random>

namespace sim {

struct IrBurstParams {
    double period_s = 833.333e-6;   // nominal burst repeat
    double period_ppm = 0.0;        // ball oscillator error
    double jitter_s = 5e-6;         // uniform ± per burst, not cumulative
    int pulses = 8;
    double on_s = 12e-6;
    double off_s = 12e-6;

    double length_s() const { return pulses * (on_s + off_s) - off_s; }
};

struct IrFrontEndParams {
    double gain_v_per_a = 1.207e5;   // held peak volts per amp of LED-on photocurrent
    double i_sat_a = 3.5 / 220e3;    // TIA output rail / feedback resistor
    double tau_bp_s = 7.92e-6;       // band-pass envelope time constant (2Q / ω0)
    int envelope_order = 1;          // 1: 1−e^{−u/τ}; 2: 1−(1+u/τ)e^{−u/τ}
    double tau_hold_s = 10e-3;       // R12 · C10
    double reset_width_s = 0.7e-6;   // R1 · C3 · ln 2 on the controller
    double vref = 3.3;
    double noise_v = 10e-6;          // rms at the hold output
    double adc_noise_lsb = 0.0;      // rms
};

class IrFrontEnd {
public:
    static constexpr int CHANNELS = 16;

    IrBurstParams burst;
    IrFrontEndParams params;
    std::array<double, CHANNELS> i_ambient{};   // A, DC photocurrent (eats TIA headroom)
    std::array<double, CHANNELS> offset_v{};    // peak-detector offset per channel

    void seed(uint64_t s) { seed_ = s; }
    void reset(uint64_t now_ns);
    void set_drive(uint64_t now_ns, const std::array<double, CHANNELS>& i_on_a);
    void on_reset_pulse(uint64_t now_ns);
    uint16_t sample(uint64_t now_ns, uint32_t channel);

    const std::array<double, CHANNELS>& drive() const { return i_on_; }
    std::array<double, CHANNELS> hold_v() const { return v_; }
    std::array<double, CHANNELS> peek(uint64_t t_ns) const;   // hold voltages at t, no side effects
    uint64_t bursts() const { return n_; }
    double last_burst_start_s() const { return s_cur_; }
    double next_burst_start_s() const { return s_next_; }
    double amplitude(int k) const;

private:
    void catch_up(double t);
    void advance(double t0, double t1);
    double draw_next();
    double uniform();
    double normal();
    double envelope(double u) const;

    std::array<double, CHANNELS> i_on_{};
    std::array<double, CHANNELS> v_{};
    double t_last_ = 0.0;
    double blank_until_ = -1.0;
    double s_cur_ = -1e300;
    double s_next_ = 0.0;
    double phase_ = 0.0;
    uint64_t n_ = 0;
    uint64_t seed_ = 0;
    std::mt19937_64 rng_;
};

}  // namespace sim
