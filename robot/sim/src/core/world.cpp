#include "core/world.h"

#include <chrono>

#include "firmware/firmware.h"

namespace sim {

World& World::get() {
    // Leaked on purpose: a parked firmware thread may still be waiting on the condition variable
    // when the interpreter exits, so the world (and its thread) must never be destroyed.
    static World* w = new World;
    return *w;
}

void World::check_usable() const {
    if (poisoned_)
        throw FirmwareHang("the simulated firmware hung earlier (a loop with no time hook); "
                           "this process can no longer run firmware — start a fresh one");
}

void World::check_thread() const {
    if (thread_.joinable() && !finished_ && !on_firmware_thread())
        throw SimMisuse("firmware code that reads the clock was called from Python while a "
                        "firmware program is running; only side-effect-free getters may be used "
                        "between steps (or stop() the program first)");
}

void World::hook_poll() {
    if (in_isr_) return;
    check_thread();
    advance_to(now_ + poll_cost_ns);
}

void World::hook_delay(const uint64_t ns) {
    if (in_isr_) return;   // a delay inside an ISR does not let time pass for the ISR
    check_thread();
    advance_to(now_ + ns);
}

void World::schedule(const uint64_t t_ns, std::function<void()> fn) {
    events_.push(Event{t_ns, seq_++, std::move(fn)});
}

void World::advance_to(const uint64_t target) {
    const bool fw = on_firmware_thread();
    if (!fw && budget_end_ns != 0 && target > budget_end_ns)
        throw SimBudgetExceeded("simulated time budget exceeded (sim.budget)");

    while (true) {
        if (fw && stop_requested_) throw SimStop{};

        for (Device* d : devices_) d->sync(now_);

        if (fw && target > now_ && now_ >= deadline_) {
            park();
            continue;
        }

        uint64_t step = target;
        if (!events_.empty() && events_.top().t < step) step = events_.top().t;
        if (fw && deadline_ < step) step = deadline_;
        if (step < now_) step = now_;

        if (step > now_) {
            for (Device* d : devices_) d->integrate(now_, step);
            now_ = step;
        }

        while (!events_.empty() && events_.top().t <= now_) {
            auto fn = std::move(const_cast<Event&>(events_.top()).fn);
            events_.pop();
            fn();
        }

        if (now_ >= target) return;
    }
}

void World::run_isr(const IrqHandler handler, void* ctx) {
    in_isr_ = true;
    try {
        handler(ctx);
    } catch (...) {
        in_isr_ = false;
        throw;
    }
    in_isr_ = false;
}

void World::raise_irq(const IrqHandler handler, void* ctx) {
    if (handler == nullptr) return;
    if (primask_ != 0 || in_isr_) {
        pending_.emplace_back(handler, ctx);
        return;
    }
    run_isr(handler, ctx);
    dispatch_pending();
}

void World::dispatch_pending() {
    while (primask_ == 0 && !in_isr_ && !pending_.empty()) {
        const auto [handler, ctx] = pending_.front();
        pending_.erase(pending_.begin());
        run_isr(handler, ctx);
    }
}

void World::set_primask(const uint32_t v) {
    primask_ = v & 1U;
    if (primask_ == 0) dispatch_pending();
}

void World::reboot() {
    check_usable();
    if (program_active()) throw SimMisuse("stop() the running program before reboot()");
    if (thread_.joinable()) thread_.join();
    thread_ = std::thread();
    fw_id_ = std::thread::id();
    finished_ = false;
    stop_requested_ = false;
    fw_turn_ = false;
    last_status_ = RunStatus::Idle;
    error_.clear();

    now_ = 0;
    events_ = {};
    seq_ = 0;
    primask_ = 0;
    in_isr_ = false;
    pending_.clear();
    budget_end_ns = 0;

    for (auto& hook : reset_hooks) hook();
    for (Device* d : devices_) d->reset();
    firmware_reboot();
}

// ---- runner ---------------------------------------------------------------------------------

void World::start(std::function<void()> entry) {
    check_usable();
    if (thread_.joinable()) {
        if (!finished_) throw SimMisuse("a firmware program is already running");
        thread_.join();
    }
    finished_ = false;
    stop_requested_ = false;
    fw_turn_ = false;
    deadline_ = now_;
    error_.clear();
    last_status_ = RunStatus::Parked;

    thread_ = std::thread([this, entry = std::move(entry)] {
        {
            std::unique_lock lock(mu_);
            cv_.wait(lock, [this] { return fw_turn_; });
        }
        RunStatus result = RunStatus::Finished;
        std::string err;
        if (!stop_requested_) {
            try {
                entry();
            } catch (const SimStop&) {
                result = RunStatus::Stopped;
            } catch (const std::exception& e) {
                result = RunStatus::Error;
                err = e.what();
            } catch (...) {
                result = RunStatus::Error;
                err = "unknown C++ exception in firmware";
            }
        } else {
            result = RunStatus::Stopped;
        }
        std::lock_guard lock(mu_);
        finished_ = true;
        last_status_ = result;
        error_ = err;
        fw_turn_ = false;
        cv_.notify_all();
    });
    fw_id_ = thread_.get_id();
}

void World::park() {
    std::unique_lock lock(mu_);
    last_status_ = RunStatus::Parked;
    fw_turn_ = false;
    cv_.notify_all();
    cv_.wait(lock, [this] { return fw_turn_; });
    if (stop_requested_) throw SimStop{};
}

RunStatus World::run_until(const uint64_t t_ns, const double wall_timeout_s) {
    check_usable();
    if (on_firmware_thread()) throw SimMisuse("run_until() called from the firmware thread");

    if (program_active()) {
        std::unique_lock lock(mu_);
        if (t_ns > deadline_) deadline_ = t_ns;
        fw_turn_ = true;
        cv_.notify_all();
        const bool done = cv_.wait_for(lock, std::chrono::duration<double>(wall_timeout_s),
                                       [this] { return !fw_turn_; });
        if (!done) {
            poisoned_ = true;
            last_status_ = RunStatus::Hung;
            return RunStatus::Hung;
        }
        if (!finished_) return RunStatus::Parked;
    }
    // No program (or it just ended): the hardware keeps running on its own.
    if (thread_.joinable() && finished_) thread_.join(), fw_id_ = std::thread::id();
    if (t_ns > now_) advance_to(t_ns);
    return last_status_ == RunStatus::Parked ? RunStatus::Idle : last_status_;
}

void World::stop(const double wall_timeout_s) {
    if (!thread_.joinable()) return;
    {
        std::unique_lock lock(mu_);
        stop_requested_ = true;
        fw_turn_ = true;
        cv_.notify_all();
        if (!cv_.wait_for(lock, std::chrono::duration<double>(wall_timeout_s),
                          [this] { return finished_; })) {
            poisoned_ = true;
            last_status_ = RunStatus::Hung;
            thread_.detach();
            return;
        }
    }
    thread_.join();
    fw_id_ = std::thread::id();
}

RunStatus World::status() const {
    std::lock_guard lock(mu_);
    if (!thread_.joinable()) return last_status_ == RunStatus::Parked ? RunStatus::Idle : last_status_;
    return last_status_;
}

}  // namespace sim
