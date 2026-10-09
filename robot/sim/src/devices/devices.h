// Simulated chips and boards around the MCU. Each one only talks to the firmware through the
// peripherals it is wired to (I2C registers, pin levels, the ADC/DMA stream, timer counters);
// Python sets their physical inputs (raw sensor values) and reads their outputs (PWM duty).
#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "core/world.h"
#include "hardware/io/encoder/Encoder.h"
#include "hardware/sensors/GPort.h"
#include "hardware/sensors/pos/Sonar.h"
#include "io/io.h"

namespace sim {

// LIS2MDL magnetometer (I2C 0x1E). Output registers latch the Python-set field at the configured
// output data rate, and only in continuous mode — so a wrong init sequence reads zeros.
class Lis2mdl final : public Device, public io::I2cDevice {
public:
    std::array<int16_t, 3> field{};   // raw X/Y/Z in LSB (1.5 mGauss/LSB)
    bool is_present = true;

    uint8_t address() const override { return 0x1E; }
    bool present() const override { return is_present; }
    void reset() override;
    void write_reg(uint8_t reg, uint8_t value) override;
    void read_regs(uint8_t reg, uint8_t* out, size_t n) override;
    const std::array<uint8_t, 128>& regs() const { return regs_; }
    uint32_t samples_latched() const { return latched_; }

private:
    void latch();
    std::array<uint8_t, 128> regs_{};
    uint64_t mode_start_ = 0;
    uint64_t last_index_ = 0;
    uint32_t latched_ = 0;
};

// LSM303AGR accelerometer (I2C 0x19). Register address auto-increments only when bit 7 of the
// sub-address is set. Python provides the raw left-justified 16-bit words.
class Lsm303Acc final : public Device, public io::I2cDevice {
public:
    std::array<int16_t, 3> raw{};
    bool is_present = true;

    uint8_t address() const override { return 0x19; }
    bool present() const override { return is_present; }
    void reset() override;
    void write_reg(uint8_t reg, uint8_t value) override;
    void read_regs(uint8_t reg, uint8_t* out, size_t n) override;
    const std::array<uint8_t, 128>& regs() const { return regs_; }

private:
    std::array<uint8_t, 128> regs_{};
};

// A G-port sensor board (IR ring or line ring): a 16-channel analog mux clocked by the MCU.
// On each clock rising edge the ADC samples the selected channel (if its trigger is routed to
// that timer); the falling edge advances the mux; wrapping to channel 0 pulses the reset line.
// Mux index i shows physical sensor translatePosition(i % 16), so after GPort's own
// un-flipping, out[k] is physical sensor k. The line board lights one LED colour per 16-channel
// sweep, in `led_order`.
class GPortBoard final : public Device {
public:
    GPortBoard(const GPortHardware& hw, GSensorKind kind);

    std::array<std::array<uint16_t, 16>, 4> values{};   // [colour or 0 for IR][physical sensor]
    std::array<LineColor, 4> led_order{LineColor::Red, LineColor::Green, LineColor::Blue, LineColor::Dark};
    bool is_present = true;

    void reset() override;
    void sync(uint64_t now) override;

    GSensorKind kind() const { return kind_; }
    uint32_t frame_length() const { return kind_ == GSensorKind::Line ? 64 : 16; }
    uint32_t mux() const { return mux_; }
    uint64_t samples() const { return samples_; }
    uint64_t resets() const { return resets_; }
    void skip_step() { advance_mux(); }   // glitch injection: lose one mux clock

private:
    void on_rise(uint64_t gen);
    void on_fall(uint64_t gen);
    void advance_mux();
    uint16_t current_value() const;

    const GPortHardware& hw_;
    GSensorKind kind_;
    uint32_t mux_ = 0;
    bool scheduled_ = false;
    uint64_t gen_ = 0;
    uint64_t samples_ = 0;
    uint64_t resets_ = 0;
};

// HC-SR04-style ultrasonic sensors sharing one trigger. A trigger pulse's falling edge starts a
// ping; each echo line goes high after `latency_us` for echo_us[i] (NaN = no echo).
class SonarArray final : public Device {
public:
    explicit SonarArray(const SonarPins& pins);

    std::array<double, SONAR_COUNT> echo_us{NAN, NAN, NAN, NAN};
    double latency_us = 200.0;
    uint64_t pings() const { return pings_; }
    void reset() override;

private:
    void on_trigger(int level);
    SonarPins pins_;
    uint64_t rise_ns_ = 0;
    bool high_ = false;
    uint64_t pings_ = 0;
};

// Quadrature encoders read by timers in encoder mode: CNT integrates the Python-set tick rate
// (ticks/s, signed) whenever the timer is counting, wrapping at ARR like the real counter.
class EncoderPlant final : public Device {
public:
    explicit EncoderPlant(const EncoderPins (&pins)[3]);

    std::array<double, 3> rate{};
    void reset() override;
    void integrate(uint64_t t0, uint64_t t1) override;
    std::array<int64_t, 3> total_ticks() const { return total_; }

private:
    std::array<TIM_TypeDef*, 3> tims_{};
    std::array<double, 3> frac_{};
    std::array<int64_t, 3> total_{};
};

// Motor H-bridge inputs and the kicker: averages each pad's duty over time.
class PwmProbe final : public Device {
public:
    static constexpr int PADS = 7;   // M1A M1B M2A M2B M3A M3B KICK

    PwmProbe(const PinName (&pads)[PADS]);
    void reset() override;
    void integrate(uint64_t t0, uint64_t t1) override;

    double duty_now(int i) const;
    // Mean duty per pad since the last call (current duty when no time has passed); resets.
    std::array<double, PADS> take_average();
    std::array<double, PADS> take_max();

private:
    std::array<PinName, PADS> pads_{};
    std::array<double, PADS> acc_{};
    std::array<double, PADS> max_{};
    double span_ = 0;
};

struct Devices {
    Lis2mdl compass;
    Lsm303Acc accel;
    GPortBoard gport1;
    GPortBoard gport2;
    SonarArray sonar;
    EncoderPlant encoders;
    PwmProbe pwm;
};

// The board's devices, created and wired on first use (wiring.cpp).
Devices& devices();

}  // namespace sim
