// Program table: which firmware entry point a simulated boot runs.
//
// main.cpp is compiled unmodified with -Dmain=fw_main and, when it defines RUN_TEST as a plain
// test name, with -D<that name>=bucky_sim_run_test. Its `RUN_TEST(ctx)` then calls the function
// below, which runs whichever test Python selected (or returns, so main() falls through into its
// real main loop). Every program therefore boots through the real init()/setupEnvironment().
#include "firmware/firmware.h"

#include <cstring>
#include <stdexcept>

#include "board/board.h"
#include "core/world.h"
#include "hardware/sensors/Button.h"
#include "programs_generated.h"
#include "tests/tests.h"

[[noreturn]] int fw_main();
void setupEnvironment();

namespace {

using TestFn = void (*)(const TestContext&);

const sim::Program TESTS[] = {
#define BUCKY_SIM_PROGRAM(name) {#name, &name},
    BUCKY_SIM_TESTS(BUCKY_SIM_PROGRAM)
#undef BUCKY_SIM_PROGRAM
};

TestFn selected = nullptr;

TestFn find_test(const std::string& name) {
    for (const auto& p : TESTS)
        if (name == p.name) return p.test;
    return nullptr;
}

[[noreturn]] void spin() {
    volatile uint32_t counter = 0;
    while (true) counter = counter + 1;   // no time hook: only the wall-clock watchdog stops this
}

}  // namespace

#ifdef BUCKY_SIM_RUN_TEST
// Called by main.cpp's RUN_TEST(ctx).
void bucky_sim_run_test(const TestContext& ctx) {
    if (selected != nullptr) selected(ctx);
}
#else
// main.cpp has no RUN_TEST: re-create its test prelude here (globals from main.cpp).
extern Motors motorDriver;
extern I2CDMABus sensorI2C;
extern Compass compass;
extern Accelerometer accel;
extern Sonar sonar;
extern Kicker kicker;
extern GPort gPort1;
extern GPort gPort2;

static void run_test_without_main(const TestFn test) {
    init();
    setupEnvironment();
    button1.begin(Board::BUTTON1);
    button2.begin(Board::BUTTON2);
    dbg.setBlocking(true);
    GPort* ir = Board::G_PORT1_KIND == GSensorKind::IR ? &gPort1
              : Board::G_PORT2_KIND == GSensorKind::IR ? &gPort2 : nullptr;
    GPort* line = Board::G_PORT1_KIND == GSensorKind::Line ? &gPort1
                : Board::G_PORT2_KIND == GSensorKind::Line ? &gPort2 : nullptr;
    const TestContext ctx = {motorDriver, &kicker, compass, accel, sonar, sensorI2C, ir, line};
    test(ctx);
    fw_main();
}
#endif

namespace sim {

const char* run_test_symbol() {
#ifdef BUCKY_SIM_RUN_TEST
    return BUCKY_SIM_RUN_TEST_NAME;
#else
    return "";
#endif
}

std::vector<std::string> program_names() {
    std::vector<std::string> names = {"firmware", "main_loop"};
    for (const auto& p : TESTS) names.emplace_back(p.name);
    return names;
}

void start_program(const std::string& name) {
    std::function<void()> entry;
    if (name == "__spin") {
        entry = [] { spin(); };
    } else if (name == "firmware") {
#ifdef BUCKY_SIM_RUN_TEST
        selected = find_test(BUCKY_SIM_RUN_TEST_NAME);
#else
        selected = nullptr;
#endif
        entry = [] { fw_main(); };
    } else if (name == "main_loop") {
        selected = nullptr;
        entry = [] { fw_main(); };
    } else {
        const TestFn test = find_test(name);
        if (test == nullptr) throw std::invalid_argument("unknown firmware program: " + name);
        selected = test;
#ifdef BUCKY_SIM_RUN_TEST
        entry = [] { fw_main(); };
#else
        entry = [test] { run_test_without_main(test); };
#endif
    }
    world().start(std::move(entry));
}

}  // namespace sim
