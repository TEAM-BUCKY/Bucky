// bucky_fw: the simulated board as a Python module.
//
//   bucky_fw.sim       virtual clock, program runner, serial/EEPROM/GPIO access
//   bucky_fw.devices   the chips and boards around the MCU (Python sets their physical inputs)
//   bucky_fw.board     pins and peripherals of the simulated PCB
//   bucky_fw.globals   main.cpp's global objects (compass, motorDriver, gPort1, …)
//   bucky_fw.<Class>   the firmware's own classes, to instantiate and unit-test directly
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <utility/stm32_eeprom.h>

#include "bindings/bindings.h"
#include "core/world.h"
#include "devices/devices.h"
#include "firmware/firmware.h"
#include "io/io.h"

namespace py = pybind11;
using sim::world;

namespace sim::eeprom {
extern uint8_t flash[E2END + 1];
void erase();
}  // namespace sim::eeprom

namespace {

const char* status_name(const sim::RunStatus s) {
    switch (s) {
        case sim::RunStatus::Idle: return "idle";
        case sim::RunStatus::Parked: return "parked";
        case sim::RunStatus::Finished: return "finished";
        case sim::RunStatus::Stopped: return "stopped";
        case sim::RunStatus::Error: return "error";
        case sim::RunStatus::Hung: return "hung";
    }
    return "?";
}

PinName pin_arg(const uint32_t p) { return static_cast<PinName>(p); }

py::bytes take(std::string& s) {
    py::bytes out(s);
    s.clear();
    return out;
}

void bind_sim(py::module_& m) {
    auto s = m.def_submodule("sim", "Virtual clock, program runner and MCU-side access");

    s.def("reboot", [] { world().reboot(); },
          "Power-on reset: peripherals, device models and firmware globals. EEPROM persists.");
    s.def("now_ns", [] { return world().now_ns(); });
    s.def("now_us", [] { return static_cast<double>(world().now_ns()) / 1e3; });
    s.def("micros", [] { return world().micros32(); }, "What micros() would return (no time passes)");
    s.def("millis", [] { return world().millis32(); });
    s.def("advance_ns", [](const uint64_t ns) {
        world().check_usable();
        if (world().program_active())
            throw sim::SimMisuse("a program is running: use run_until() to move time");
        world().advance_by(ns);
    }, py::arg("ns"), "Let the hardware run for `ns` (unit mode: no program thread).");
    s.def("set_poll_cost_ns", [](const uint64_t ns) { world().poll_cost_ns = ns; },
          "Virtual time each millis()/micros() call costs (default 1000 ns)");
    s.def("poll_cost_ns", [] { return world().poll_cost_ns; });
    s.def("set_start_offset_us", [](const uint64_t us) { world().set_start_offset_us(us); },
          "Offset added to micros()/millis(), e.g. to test 32-bit wrap-around");
    s.def("set_budget_end_ns", [](const uint64_t ns) { world().budget_end_ns = ns; },
          "Unit mode: raise SimBudgetExceeded when time would pass this point (0 = off)");

    // programs
    s.def("programs", &sim::program_names);
    s.def("run_test_symbol", [] { return std::string(sim::run_test_symbol()); },
          "The test main.cpp's RUN_TEST names ('' when it has none)");
    s.def("start", [](const std::string& name) { sim::start_program(name); }, py::arg("program"),
          "Boot `program` on the firmware thread. It does not run until run_until().");
    s.def("run_until_ns", [](const uint64_t t_ns, const double wall_timeout_s) {
        sim::RunStatus st;
        {
            py::gil_scoped_release release;
            st = world().run_until(t_ns, wall_timeout_s);
        }
        return std::string(status_name(st));
    }, py::arg("t_ns"), py::arg("wall_timeout_s") = 10.0,
       "Run the firmware (and hardware) until virtual time t_ns. Returns the program status: "
       "'parked' (still running), 'finished', 'stopped', 'error', 'idle' (no program) or 'hung'.");
    s.def("stop", [](const double wall_timeout_s) {
        py::gil_scoped_release release;
        world().stop(wall_timeout_s);
    }, py::arg("wall_timeout_s") = 5.0);
    s.def("status", [] { return std::string(status_name(world().status())); });
    s.def("error", [] { return world().error(); });
    s.def("program_active", [] { return world().program_active(); });
    s.def("poisoned", [] { return world().poisoned(); });

    // serial
    s.def("usb_read", [] { return take(sim::io::usb_tx()); }, "Drain the firmware's USB output");
    s.def("usb_peek", [] { return py::bytes(sim::io::usb_tx()); });
    s.def("usb_inject", [](const py::bytes& b) { sim::io::usb_rx() += std::string(b); });
    s.def("set_usb_connected", [](const bool c) { sim::io::usb_connected() = c; });
    s.def("uart_read", [](const uint32_t index) {
        static USART_TypeDef* const UARTS[] = {USART1, USART2, USART3, UART4, UART5};
        if (index < 1 || index > 5) throw py::index_error("USART1..UART5");
        return take(sim::io::uart_tx(UARTS[index - 1]));
    }, py::arg("index"));
    s.def("uart_inject", [](const uint32_t index, const py::bytes& b) {
        static USART_TypeDef* const UARTS[] = {USART1, USART2, USART3, UART4, UART5};
        if (index < 1 || index > 5) throw py::index_error("USART1..UART5");
        sim::io::uart_rx(UARTS[index - 1]) += std::string(b);
    }, py::arg("index"), py::arg("data"));

    // EEPROM (flash emulation)
    s.def("eeprom_read", [] {
        return py::bytes(reinterpret_cast<const char*>(sim::eeprom::flash), sizeof sim::eeprom::flash);
    });
    s.def("eeprom_write", [](const py::bytes& b, const uint32_t offset) {
        const std::string data = b;
        if (offset + data.size() > sizeof sim::eeprom::flash) throw py::value_error("past E2END");
        std::memcpy(sim::eeprom::flash + offset, data.data(), data.size());
    }, py::arg("data"), py::arg("offset") = 0);
    s.def("eeprom_erase", &sim::eeprom::erase);

    // GPIO / analog
    s.def("gpio_level", [](const uint32_t p) { return sim::io::input_level(pin_arg(p)); }, py::arg("pin"));
    s.def("gpio_output", [](const uint32_t p) { return sim::io::odr_level(pin_arg(p)); }, py::arg("pin"));
    s.def("gpio_mode", [](const uint32_t p) { return sim::io::pin_mode(pin_arg(p)); }, py::arg("pin"),
          "0 input, 1 output, 2 alternate function, 3 analog");
    s.def("set_input", [](const uint32_t p, const int level) { sim::io::set_input(pin_arg(p), level); },
          py::arg("pin"), py::arg("level"));
    s.def("schedule_input", [](const uint32_t p, const int level, const uint64_t at_ns) {
        world().schedule(at_ns, [p, level] { sim::io::set_input(pin_arg(p), level); });
    }, py::arg("pin"), py::arg("level"), py::arg("at_ns"));
    s.def("set_analog", [](const uint32_t p, const uint16_t v) { sim::io::set_analog(p, v); },
          py::arg("pin"), py::arg("value"));
    s.def("pad_duty", [](const uint32_t p) {
        const sim::io::Waveform w = sim::io::pad_waveform(pin_arg(p));
        return w.timer ? w.duty : static_cast<double>(w.static_level);
    }, py::arg("pin"), "Duty the pad outputs right now (PWM duty, or 0/1 for a GPIO)");
    s.def("pad_period_ns", [](const uint32_t p) { return sim::io::pad_waveform(pin_arg(p)).period_ns; },
          py::arg("pin"));
}

void bind_devices(py::module_& m) {
    auto d = m.def_submodule("devices", "Simulated chips and boards (their physical inputs)");

    py::class_<sim::Lis2mdl>(d, "Lis2mdl")
        .def_readwrite("field", &sim::Lis2mdl::field, "Raw X/Y/Z, LSB (1.5 mGauss/LSB)")
        .def_readwrite("present", &sim::Lis2mdl::is_present)
        .def("regs", &sim::Lis2mdl::regs)
        .def_property_readonly("samples_latched", &sim::Lis2mdl::samples_latched);

    py::class_<sim::Lsm303Acc>(d, "Lsm303Acc")
        .def_readwrite("raw", &sim::Lsm303Acc::raw, "Raw left-justified X/Y/Z words")
        .def_readwrite("present", &sim::Lsm303Acc::is_present)
        .def("regs", &sim::Lsm303Acc::regs);

    py::class_<sim::IrBurstParams>(d, "IrBurstParams")
        .def_readwrite("period_s", &sim::IrBurstParams::period_s)
        .def_readwrite("period_ppm", &sim::IrBurstParams::period_ppm)
        .def_readwrite("jitter_s", &sim::IrBurstParams::jitter_s)
        .def_readwrite("pulses", &sim::IrBurstParams::pulses)
        .def_readwrite("on_s", &sim::IrBurstParams::on_s)
        .def_readwrite("off_s", &sim::IrBurstParams::off_s)
        .def_property_readonly("length_s", &sim::IrBurstParams::length_s);

    py::class_<sim::IrFrontEndParams>(d, "IrFrontEndParams")
        .def_readwrite("gain_v_per_a", &sim::IrFrontEndParams::gain_v_per_a)
        .def_readwrite("i_sat_a", &sim::IrFrontEndParams::i_sat_a)
        .def_readwrite("tau_bp_s", &sim::IrFrontEndParams::tau_bp_s)
        .def_readwrite("envelope_order", &sim::IrFrontEndParams::envelope_order)
        .def_readwrite("tau_hold_s", &sim::IrFrontEndParams::tau_hold_s)
        .def_readwrite("reset_width_s", &sim::IrFrontEndParams::reset_width_s)
        .def_readwrite("vref", &sim::IrFrontEndParams::vref)
        .def_readwrite("noise_v", &sim::IrFrontEndParams::noise_v)
        .def_readwrite("adc_noise_lsb", &sim::IrFrontEndParams::adc_noise_lsb);

    py::class_<sim::IrFrontEnd>(d, "IrFrontEnd",
                                "Time-domain IR sensor chain: burst schedule, peak hold, droop, RESET")
        .def_property("burst", [](sim::IrFrontEnd& f) -> sim::IrBurstParams& { return f.burst; },
                      [](sim::IrFrontEnd& f, const sim::IrBurstParams& b) { f.burst = b; },
                      py::return_value_policy::reference_internal)
        .def_property("params", [](sim::IrFrontEnd& f) -> sim::IrFrontEndParams& { return f.params; },
                      [](sim::IrFrontEnd& f, const sim::IrFrontEndParams& p) { f.params = p; },
                      py::return_value_policy::reference_internal)
        .def_readwrite("i_ambient", &sim::IrFrontEnd::i_ambient, "DC photocurrent per channel (A)")
        .def_readwrite("offset_v", &sim::IrFrontEnd::offset_v, "Peak-detector offset per channel (V)")
        .def("seed", &sim::IrFrontEnd::seed, "RNG seed, applied at the next reboot/reset")
        .def("reset", [](sim::IrFrontEnd& f) { f.reset(world().now_ns()); },
             "Clear holds and restart the burst schedule now (reseeds)")
        .def("set_drive",
             [](sim::IrFrontEnd& f, const std::array<double, 16>& i) { f.set_drive(world().now_ns(), i); },
             "LED-on photocurrent per channel (A), effective from now")
        .def_property_readonly("drive", &sim::IrFrontEnd::drive)
        .def_property_readonly("hold_v", &sim::IrFrontEnd::hold_v)
        .def("peek", &sim::IrFrontEnd::peek, py::arg("t_ns"), "Hold voltages at t_ns (no side effects)")
        .def("amplitude", &sim::IrFrontEnd::amplitude, "Steady-state held amplitude of a channel (V)")
        .def_property_readonly("bursts", &sim::IrFrontEnd::bursts)
        .def_property_readonly("last_burst_start_s", &sim::IrFrontEnd::last_burst_start_s)
        .def_property_readonly("next_burst_start_s", &sim::IrFrontEnd::next_burst_start_s);

    py::class_<sim::GPortBoard>(d, "GPortBoard")
        .def_readwrite("values", &sim::GPortBoard::values,
                       "[LineColor (or 0 for IR)][physical sensor] ADC counts")
        .def_property("led_order",
            [](const sim::GPortBoard& b) {
                std::vector<int> o;
                for (auto c : b.led_order) o.push_back(static_cast<int>(c));
                return o;
            },
            [](sim::GPortBoard& b, const std::vector<int>& o) {
                if (o.size() != 4) throw py::value_error("4 colours");
                for (int i = 0; i < 4; i++) b.led_order[i] = static_cast<LineColor>(o[i]);
            })
        .def_readwrite("present", &sim::GPortBoard::is_present)
        .def_readwrite("ir_physical", &sim::GPortBoard::ir_physical,
                       "IR: sample the time-domain front end (ir) instead of values")
        .def_property_readonly("ir", [](sim::GPortBoard& b) -> sim::IrFrontEnd& { return b.ir; },
                               py::return_value_policy::reference_internal)
        .def_property_readonly("kind", [](const sim::GPortBoard& b) { return static_cast<int>(b.kind()); })
        .def_property_readonly("frame_length", &sim::GPortBoard::frame_length)
        .def_property_readonly("mux", &sim::GPortBoard::mux)
        .def_property_readonly("samples", &sim::GPortBoard::samples)
        .def_property_readonly("resets", &sim::GPortBoard::resets)
        .def("skip_step", &sim::GPortBoard::skip_step, "Glitch: lose one mux clock");

    py::class_<sim::SonarArray>(d, "SonarArray")
        .def_readwrite("echo_us", &sim::SonarArray::echo_us, "Echo pulse width per sensor; NaN = none")
        .def_readwrite("latency_us", &sim::SonarArray::latency_us)
        .def_property_readonly("pings", &sim::SonarArray::pings);

    py::class_<sim::EncoderPlant>(d, "EncoderPlant")
        .def_readwrite("rate", &sim::EncoderPlant::rate, "Ticks per second per wheel (signed)")
        .def_property_readonly("total_ticks", &sim::EncoderPlant::total_ticks);

    py::class_<sim::PwmProbe>(d, "PwmProbe")
        .def("duty_now", &sim::PwmProbe::duty_now)
        .def("take_average", &sim::PwmProbe::take_average,
             "Mean duty per pad [M1A, M1B, M2A, M2B, M3A, M3B, KICK] since the last call")
        .def("take_max", &sim::PwmProbe::take_max);

    auto& dev = sim::devices();
    const auto ref = py::return_value_policy::reference;
    d.attr("compass") = py::cast(&dev.compass, ref);
    d.attr("accel") = py::cast(&dev.accel, ref);
    d.attr("gport1") = py::cast(&dev.gport1, ref);
    d.attr("gport2") = py::cast(&dev.gport2, ref);
    d.attr("sonar") = py::cast(&dev.sonar, ref);
    d.attr("encoders") = py::cast(&dev.encoders, ref);
    d.attr("pwm") = py::cast(&dev.pwm, ref);
}

}  // namespace

PYBIND11_MODULE(bucky_fw, m) {
    m.doc() = "Bucky firmware (robot/src) compiled for the host against a simulated STM32H562 board";

    static py::exception<sim::SimError> sim_error(m, "SimError");
    static py::exception<sim::SimBudgetExceeded> budget(m, "SimBudgetExceeded", sim_error.ptr());
    static py::exception<sim::SimMisuse> misuse(m, "SimMisuse", sim_error.ptr());
    static py::exception<sim::FirmwareHang> hang(m, "FirmwareHang", sim_error.ptr());
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const sim::SimBudgetExceeded& e) {
            py::set_error(budget, e.what());
        } catch (const sim::SimMisuse& e) {
            py::set_error(misuse, e.what());
        } catch (const sim::FirmwareHang& e) {
            py::set_error(hang, e.what());
        } catch (const sim::SimError& e) {
            py::set_error(sim_error, e.what());
        } catch (const sim::SimStop&) {
            py::set_error(PyExc_RuntimeError, "firmware stopped");
        }
    });

    sim::devices();   // wire the board before anything boots
    world().reboot();

    bind_sim(m);
    bind_devices(m);
    bucky_bindings::bind_board(m);
    bucky_bindings::bind_firmware(m);
}
