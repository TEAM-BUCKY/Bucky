// Glue between the simulator and the compiled firmware (main.cpp, tests/*.cpp).
#pragma once

#include <string>
#include <vector>

struct TestContext;

namespace sim {

// Destroys and re-constructs main.cpp's globals (and the header-inline ones: dbg, buttons,
// pwm_sync) in place, so every boot starts from the same state as a power-on.
void firmware_reboot();

struct Program {
    const char* name;
    void (*test)(const TestContext&);   // nullptr for the built-ins below
};

// "firmware"  : main() exactly as built (runs main.cpp's RUN_TEST, if any)
// "main_loop" : main() with the RUN_TEST call skipped (the real main loop)
// "testX"     : main() with RUN_TEST(ctx) replaced by testX(ctx)
// "__spin"    : a hook-free infinite loop, used to test the hang watchdog
std::vector<std::string> program_names();
const char* run_test_symbol();   // what main.cpp's RUN_TEST names, or "" when unset
void start_program(const std::string& name);

}  // namespace sim
