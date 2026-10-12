// Simulator-side view of the MCU peripherals: what the device models (G-port boards, sonar,
// encoders, PWM probe, I2C chips) use to observe and drive the firmware's pins and registers.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <Arduino.h>

namespace sim::io {

// ---- GPIO ------------------------------------------------------------------------------------
constexpr uint32_t PORT_COUNT = 9;   // A..I

// The physical pad of a PinName (drops the _ALTn timer/ADC selector bits).
inline PinName pad(const PinName p) { return static_cast<PinName>(p & PNAME_MASK); }

GPIO_TypeDef* port(uint32_t index);
int port_index(const volatile void* reg_in_port);   // -1 when not inside a GPIO block

enum PinMode : uint32_t { MODE_INPUT = 0, MODE_OUTPUT = 1, MODE_AF = 2, MODE_ANALOG = 3 };
uint32_t pin_mode(PinName p);
uint32_t pin_af(PinName p);
int odr_level(PinName p);
int input_level(PinName p);

// Drive an input pad from the outside world (a button, an echo line, a reset pulse). Updates
// IDR and raises the EXTI interrupt when the line is armed for that port and edge.
void set_input(PinName p, int level);

// Called when the firmware changes the ODR bit of a pad configured as GPIO output.
using OutputListener = std::function<void(int level)>;
void on_output(PinName p, OutputListener cb);   // device wiring; survives reboot

// Called after an external edge on an input pad (input-capture timers). Registered by firmware
// at run time, so these are cleared on reboot.
void on_input_edge(PinName p, OutputListener cb);

// EXTI: an edge on (port, line). Raises the attached handler when armed for that edge.
void exti_edge(uint32_t port, uint32_t line, bool rising);

// ---- Timers ------------------------------------------------------------------------------------
constexpr uint64_t TIMER_CLOCK_HZ = 250'000'000;   // H562: APB1/APB2 = 250 MHz
constexpr uint64_t LPTIM_CLOCK_HZ = 250'000'000;   // LPTIM1 kernel clock = PCLK3

// What a pad currently outputs. AF pads follow their timer channel's PWM; GPIO outputs are static.
struct Waveform {
    bool timer = false;       // driven by a timer that is counting
    uint64_t period_ns = 0;
    double duty = 0.0;        // fraction of the period the pad is high
    int static_level = 0;     // when !timer
    // Identity of the driving source, for trigger routing (ADC EXTSEL).
    TIM_TypeDef* tim = nullptr;
    LPTIM_TypeDef* lptim = nullptr;
    uint8_t channel = 0;
};
Waveform pad_waveform(PinName p);

// The ADC EXTSEL code a timer channel's rising edge triggers (or -1). TIM15 counts only when
// CR2.MMS routes OCxREF of that channel to TRGO.
int adc_trigger_id(const Waveform& w);

// ---- ADC + DMA -------------------------------------------------------------------------------
// One conversion of `value` on `adc` (as if its external trigger fired): writes DR and moves it
// through the DMA channel that serves the ADC's request, if one is set up and enabled.
// Returns false when the ADC is not enabled / not converting the channel of `pin`.
bool adc_trigger(ADC_TypeDef* adc, int trigger_id, bool rising, PinName pin, uint16_t value);

// ---- I2C -------------------------------------------------------------------------------------
class I2cDevice {
public:
    virtual ~I2cDevice() = default;
    virtual uint8_t address() const = 0;
    virtual bool present() const { return true; }
    virtual void write_reg(uint8_t reg, uint8_t value) = 0;
    // Burst read starting at `reg` (the device decides how the address advances).
    virtual void read_regs(uint8_t reg, uint8_t* out, size_t n) = 0;
};
void i2c_attach(I2C_TypeDef* bus, I2cDevice* dev);

// ---- Serial ------------------------------------------------------------------------------------
std::string& usb_tx();          // everything the firmware wrote after host.begin()
std::string& usb_rx();          // bytes waiting for the firmware to read
bool& usb_connected();
std::string& uart_tx(USART_TypeDef* u);
std::string& uart_rx(USART_TypeDef* u);

// ---- Analog pins (analogRead) ------------------------------------------------------------------
void set_analog(uint32_t pin, uint16_t value);
uint16_t analog(uint32_t pin);

}  // namespace sim::io
