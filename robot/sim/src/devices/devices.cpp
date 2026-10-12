#include "devices/devices.h"

#include <cstring>

#include <PeripheralPins.h>
#include <pinmap.h>

#include "hardware/io/timer/Timer.h"

namespace sim {

// ---- LIS2MDL ---------------------------------------------------------------------------------

namespace {
constexpr uint8_t LIS2MDL_WHO_AM_I = 0x4F;
constexpr uint8_t LIS2MDL_CFG_A = 0x60;
constexpr uint8_t LIS2MDL_STATUS = 0x67;
constexpr uint8_t LIS2MDL_OUTX_L = 0x68;

uint64_t lis2mdl_period_ns(const uint8_t cfg_a) {
    static constexpr uint64_t ODR_HZ[4] = {10, 20, 50, 100};
    return 1'000'000'000ULL / ODR_HZ[(cfg_a >> 2) & 3U];
}
}  // namespace

void Lis2mdl::reset() {
    regs_.fill(0);
    regs_[LIS2MDL_WHO_AM_I] = 0x40;
    regs_[LIS2MDL_CFG_A] = 0x03;   // idle
    mode_start_ = world().now_ns();
    last_index_ = 0;
    latched_ = 0;
}

void Lis2mdl::write_reg(const uint8_t reg, const uint8_t value) {
    if (reg >= regs_.size()) return;
    if (reg == LIS2MDL_CFG_A) {
        if (value & 0x20) {   // SOFT_RST: configuration back to defaults
            const auto keep = regs_;
            reset();
            for (uint8_t r = LIS2MDL_OUTX_L; r < LIS2MDL_OUTX_L + 6; r++) regs_[r] = keep[r];
            return;
        }
        const bool was_continuous = (regs_[LIS2MDL_CFG_A] & 3U) == 0;
        regs_[reg] = value;
        if ((value & 3U) == 0 && !was_continuous) {
            mode_start_ = world().now_ns();
            last_index_ = 0;
        }
        return;
    }
    regs_[reg] = value;
}

void Lis2mdl::latch() {
    if ((regs_[LIS2MDL_CFG_A] & 3U) != 0) return;   // not in continuous mode
    const uint64_t index = (world().now_ns() - mode_start_) / lis2mdl_period_ns(regs_[LIS2MDL_CFG_A]);
    if (index == 0 || index == last_index_) return;
    last_index_ = index;
    for (int a = 0; a < 3; a++) {
        const auto v = static_cast<uint16_t>(field[a]);
        regs_[LIS2MDL_OUTX_L + 2 * a] = v & 0xFF;
        regs_[LIS2MDL_OUTX_L + 2 * a + 1] = v >> 8;
    }
    regs_[LIS2MDL_STATUS] |= 0x08;   // Zyxda
    latched_++;
}

void Lis2mdl::read_regs(const uint8_t reg, uint8_t* out, const size_t n) {
    latch();
    for (size_t i = 0; i < n; i++) out[i] = regs_[(reg + i) & 0x7F];
    if (reg <= LIS2MDL_OUTX_L + 5 && reg + n > LIS2MDL_OUTX_L) regs_[LIS2MDL_STATUS] &= ~0x08;
}

// ---- LSM303AGR accelerometer -----------------------------------------------------------------

namespace {
constexpr uint8_t LSM_WHO_AM_I = 0x0F;
constexpr uint8_t LSM_CTRL_REG1 = 0x20;
constexpr uint8_t LSM_OUT_X_L = 0x28;
}  // namespace

void Lsm303Acc::reset() {
    regs_.fill(0);
    regs_[LSM_WHO_AM_I] = 0x33;
    regs_[LSM_CTRL_REG1] = 0x07;   // power-down, XYZ enabled
}

void Lsm303Acc::write_reg(const uint8_t reg, const uint8_t value) {
    const uint8_t r = reg & 0x7F;
    if (r < regs_.size()) regs_[r] = value;
}

void Lsm303Acc::read_regs(const uint8_t reg, uint8_t* out, const size_t n) {
    if ((regs_[LSM_CTRL_REG1] >> 4) != 0) {   // ODR set: sampling
        for (int a = 0; a < 3; a++) {
            const auto v = static_cast<uint16_t>(raw[a]);
            regs_[LSM_OUT_X_L + 2 * a] = v & 0xFF;
            regs_[LSM_OUT_X_L + 2 * a + 1] = v >> 8;
        }
    }
    const bool autoinc = (reg & 0x80) != 0;
    const uint8_t start = reg & 0x7F;
    for (size_t i = 0; i < n; i++) out[i] = regs_[(autoinc ? start + i : start) & 0x7F];
}

// ---- G-port boards ---------------------------------------------------------------------------

GPortBoard::GPortBoard(const GPortHardware& hw, const GSensorKind kind) : hw_(hw), kind_(kind) {
    // The firmware may also step the mux by hand (LineSensor show mode): the board counts the
    // clock pad's falling edges whenever the pad is a GPIO output.
    io::on_output(hw.clockPin, [this](const int level) {
        if (level == 0 && io::pin_mode(hw_.clockPin) == io::MODE_OUTPUT) advance_mux();
    });
}

void GPortBoard::reset() {
    mux_ = 0;
    scheduled_ = false;
    gen_++;
    samples_ = 0;
    resets_ = 0;
}

void GPortBoard::sync(const uint64_t now) {
    if (scheduled_) return;
    const io::Waveform w = io::pad_waveform(hw_.clockPin);
    if (!w.timer || w.period_ns == 0) return;
    scheduled_ = true;
    const uint64_t gen = ++gen_;
    world().schedule(now + w.period_ns, [this, gen] { on_rise(gen); });
}

void GPortBoard::on_rise(const uint64_t gen) {
    if (gen != gen_) return;
    const io::Waveform w = io::pad_waveform(hw_.clockPin);
    if (!w.timer || w.period_ns == 0) {
        scheduled_ = false;
        return;
    }
    const uint64_t now = world().now_ns();

    // The timer's rising edge is the ADC trigger (TRGO / LPTIM CH1 output).
    auto* adc = static_cast<ADC_TypeDef*>(pinmap_peripheral(hw_.adcPin, PinMap_ADC));
    if (io::adc_trigger(adc, io::adc_trigger_id(w), true, hw_.adcPin, current_value())) samples_++;

    const auto high = static_cast<uint64_t>(w.duty * static_cast<double>(w.period_ns));
    if (high > 0 && high < w.period_ns)
        world().schedule(now + high, [this, gen] { on_fall(gen); });
    world().schedule(now + w.period_ns, [this, gen] { on_rise(gen); });
}

void GPortBoard::on_fall(const uint64_t gen) {
    if (gen != gen_) return;
    if (io::pin_mode(hw_.clockPin) == io::MODE_AF) advance_mux();
}

void GPortBoard::advance_mux() {
    mux_ = (mux_ + 1) % frame_length();
    if (mux_ != 0 || !is_present) return;
    // Board reset pulse: the firmware's EXTI handler marks a frame boundary.
    resets_++;
    io::set_input(hw_.resetPin, 1);
    world().schedule(world().now_ns() + 1000, [this] { io::set_input(hw_.resetPin, 0); });
}

uint16_t GPortBoard::current_value() const {
    if (!is_present) return 0;
    const uint32_t sweep = mux_ / 16, ch = mux_ % 16;
    const uint32_t phys = ch < 8 ? 7 - ch : ch;   // the boards' flipped first 8 channels
    if (kind_ == GSensorKind::Line) return values[static_cast<uint8_t>(led_order[sweep])][phys];
    return values[0][phys];
}

// ---- Sonar -----------------------------------------------------------------------------------

SonarArray::SonarArray(const SonarPins& pins) : pins_(pins) {
    io::on_output(pins.trigPin, [this](const int level) { on_trigger(level); });
}

void SonarArray::reset() {
    high_ = false;
    pings_ = 0;
    for (const PinName p : pins_.echoPins)
        if (p != NC) io::set_input(p, 0);
}

void SonarArray::on_trigger(const int level) {
    const uint64_t now = world().now_ns();
    if (level) {
        high_ = true;
        rise_ns_ = now;
        return;
    }
    if (!high_) return;
    high_ = false;
    if (now - rise_ns_ < 10'000) return;   // HC-SR04 needs a >= 10 us trigger pulse
    pings_++;
    const auto lat = static_cast<uint64_t>(latency_us * 1000.0);
    for (int i = 0; i < SONAR_COUNT; i++) {
        const PinName p = pins_.echoPins[i];
        if (p == NC || !std::isfinite(echo_us[i]) || echo_us[i] < 0) continue;
        const uint64_t up = now + lat;
        const uint64_t down = up + static_cast<uint64_t>(echo_us[i] * 1000.0);
        world().schedule(up, [p] { io::set_input(p, 1); });
        world().schedule(down, [p] { io::set_input(p, 0); });
    }
}

// ---- Encoders --------------------------------------------------------------------------------

EncoderPlant::EncoderPlant(const EncoderPins (&pins)[3]) {
    for (int i = 0; i < 3; i++) tims_[i] = tim_from_pin(pins[i].pinA, nullptr, nullptr);
}

void EncoderPlant::reset() {
    frac_.fill(0);
    total_.fill(0);
}

void EncoderPlant::integrate(const uint64_t t0, const uint64_t t1) {
    const double dt = static_cast<double>(t1 - t0) * 1e-9;
    for (int i = 0; i < 3; i++) {
        TIM_TypeDef* t = tims_[i];
        if (t == nullptr || rate[i] == 0.0) continue;
        if (!(t->CR1 & TIM_CR1_CEN) || (t->SMCR & TIM_SMCR_SMS_Msk) != TIM_SMCR_SMS_0) continue;
        frac_[i] += rate[i] * dt;
        const auto whole = static_cast<int64_t>(std::floor(frac_[i]));
        if (whole == 0) continue;
        frac_[i] -= static_cast<double>(whole);
        total_[i] += whole;
        const int64_t mod = static_cast<int64_t>(t->ARR) + 1;
        int64_t cnt = (static_cast<int64_t>(t->CNT) + whole) % mod;
        if (cnt < 0) cnt += mod;
        t->CNT = static_cast<uint32_t>(cnt);
    }
}

// ---- PWM probe -------------------------------------------------------------------------------

PwmProbe::PwmProbe(const PinName (&pads)[PADS]) {
    for (int i = 0; i < PADS; i++) pads_[i] = pads[i];
}

void PwmProbe::reset() {
    acc_.fill(0);
    max_.fill(0);
    span_ = 0;
}

double PwmProbe::duty_now(const int i) const {
    const io::Waveform w = io::pad_waveform(pads_[i]);
    return w.timer ? w.duty : static_cast<double>(w.static_level);
}

void PwmProbe::integrate(const uint64_t t0, const uint64_t t1) {
    const double dt = static_cast<double>(t1 - t0);
    for (int i = 0; i < PADS; i++) {
        const double d = duty_now(i);
        acc_[i] += d * dt;
        if (d > max_[i]) max_[i] = d;
    }
    span_ += dt;
}

std::array<double, PwmProbe::PADS> PwmProbe::take_average() {
    std::array<double, PADS> out{};
    for (int i = 0; i < PADS; i++) out[i] = span_ > 0 ? acc_[i] / span_ : duty_now(i);
    acc_.fill(0);
    span_ = 0;
    return out;
}

std::array<double, PwmProbe::PADS> PwmProbe::take_max() {
    std::array<double, PADS> out{};
    for (int i = 0; i < PADS; i++) out[i] = std::max(max_[i], duty_now(i));
    max_.fill(0);
    return out;
}

}  // namespace sim
