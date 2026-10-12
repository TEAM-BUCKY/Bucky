// Simulator core: virtual clock, event queue, interrupt dispatch and the lockstep program runner.
//
// Everything the firmware experiences as "hardware over time" goes through World::advance_to():
//
//   * Time only moves inside the firmware's time hooks (millis/micros cost poll_cost_ns,
//     delay/delayMicroseconds cost exactly their argument) or when Python advances the world.
//   * Hardware events (a sonar echo edge, a G-port mux clock edge, a reset pulse) are scheduled
//     callbacks. They update pin levels / DMA buffers and may raise interrupts.
//   * Interrupts run immediately at the hook point when PRIMASK is clear and no ISR is active,
//     otherwise they pend until __enable_irq()/__set_PRIMASK(0) or the running ISR returns.
//     Time hooks called from inside an ISR read the clock without advancing it.
//
// Lockstep: a whole firmware program (main(), a RUN_TEST test) runs on its own thread. Python
// calls run_until(t); the firmware runs until its virtual clock reaches t, then parks inside the
// time hook it was in. Exactly one of the two threads runs at a time (a turn flag), so Python may
// poke device models and read firmware state while the firmware is parked, without locking.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware/io/irq/IRQ.h"

namespace sim {

// Thrown from a time hook on the firmware thread to unwind a program that is being stopped.
// Deliberately not a std::exception so firmware code can never swallow it by accident.
struct SimStop {};

struct SimError : std::runtime_error { using std::runtime_error::runtime_error; };
struct SimBudgetExceeded : SimError { using SimError::SimError; };
struct SimMisuse : SimError { using SimError::SimError; };
struct FirmwareHang : SimError { using SimError::SimError; };

class Device {
public:
    virtual ~Device() = default;
    // Back to power-on state (sim.reboot()). Python-set inputs (field, distances, …) survive.
    virtual void reset() {}
    // Called at every step of advance_to(); cheap. Lets a model notice register changes
    // (a timer that was just started) and schedule its own events.
    virtual void sync(uint64_t /*now_ns*/) {}
    // Continuous models (encoder counters, PWM duty integration) over [t0, t1).
    virtual void integrate(uint64_t /*t0_ns*/, uint64_t /*t1_ns*/) {}
};

enum class RunStatus { Idle, Parked, Finished, Stopped, Error, Hung };

class World {
public:
    static World& get();

    // ---- time -------------------------------------------------------------------------
    uint64_t now_ns() const { return now_; }
    uint32_t micros32() const { return static_cast<uint32_t>(now_ / 1000 + start_offset_us_); }
    uint32_t millis32() const {
        return static_cast<uint32_t>((now_ / 1000 + start_offset_us_) / 1000);
    }

    void hook_poll();                 // millis()/micros()
    void hook_delay(uint64_t ns);     // delay()/delayMicroseconds()
    void advance_to(uint64_t t_ns);   // run hardware (and parked handoffs) up to t
    void advance_by(uint64_t dt_ns) { advance_to(now_ + dt_ns); }

    uint64_t poll_cost_ns = 1000;
    void set_start_offset_us(uint64_t us) { start_offset_us_ = us; }
    uint64_t start_offset_us() const { return start_offset_us_; }

    // Unit mode (no program thread): calls that would move time past this raise
    // SimBudgetExceeded. 0 = unlimited.
    uint64_t budget_end_ns = 0;

    // ---- events -----------------------------------------------------------------------
    void schedule(uint64_t t_ns, std::function<void()> fn);

    // ---- interrupts -------------------------------------------------------------------
    void raise_irq(IrqHandler handler, void* ctx);
    uint32_t primask() const { return primask_; }
    void set_primask(uint32_t v);
    bool in_isr() const { return in_isr_; }

    // ---- devices ----------------------------------------------------------------------
    void add_device(Device* d) { devices_.push_back(d); }

    // Power-on reset of every peripheral, device model and firmware global. Requires that no
    // program is running.
    void reboot();
    std::vector<std::function<void()>> reset_hooks;   // io modules register their state reset

    // ---- lockstep program runner --------------------------------------------------------
    void start(std::function<void()> entry);
    RunStatus run_until(uint64_t t_ns, double wall_timeout_s);
    void stop(double wall_timeout_s = 5.0);
    RunStatus status() const;
    const std::string& error() const { return error_; }
    bool program_active() const { return thread_.joinable() && !finished_; }
    bool poisoned() const { return poisoned_; }
    bool on_firmware_thread() const { return std::this_thread::get_id() == fw_id_; }
    void check_usable() const;

private:
    World() = default;

    void check_thread() const;
    void park();
    void dispatch_pending();
    void run_isr(IrqHandler handler, void* ctx);

    uint64_t now_ = 0;
    uint64_t start_offset_us_ = 0;

    struct Event {
        uint64_t t;
        uint64_t seq;
        std::function<void()> fn;
        bool operator>(const Event& o) const { return t != o.t ? t > o.t : seq > o.seq; }
    };
    std::priority_queue<Event, std::vector<Event>, std::greater<>> events_;
    uint64_t seq_ = 0;

    uint32_t primask_ = 0;
    bool in_isr_ = false;
    std::vector<std::pair<IrqHandler, void*>> pending_;

    std::vector<Device*> devices_;

    // runner
    std::thread thread_;
    std::thread::id fw_id_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool fw_turn_ = false;
    bool stop_requested_ = false;
    bool finished_ = false;
    bool poisoned_ = false;
    uint64_t deadline_ = 0;
    RunStatus last_status_ = RunStatus::Idle;
    std::string error_;
};

inline World& world() { return World::get(); }

}  // namespace sim
