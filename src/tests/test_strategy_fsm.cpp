#include "tests.h"
#include "debug.h"
#include "strategy/StrategyFSM.h"
#include "field/DigitalField.h"
#include "control/ball/IMMBallTracker.h"
#include "control/pos/SelfLocalizationEKF.h"
#include "control/pos/EnemyTracker.h"

#include <Arduino.h>
#include <cmath>

// Pure-logic tests for StrategyFSM::update and the IMM lost_ms saturation.
// Runs entirely on synthetic DigitalField snapshots — no sensors needed.
// Each case prints PASS/FAIL with the expected vs actual state.

namespace
{
int g_pass = 0;
int g_fail = 0;

const char* stateName(GameState_t s)
{
    switch (s)
    {
        case STATE_FIND_BALL:       return "FIND_BALL";
        case STATE_CHASE_BALL:      return "CHASE_BALL";
        case STATE_DRIBBLE:         return "DRIBBLE";
        case STATE_SHOOT:           return "SHOOT";
        case STATE_DEFEND:          return "DEFEND";
        case STATE_INTERCEPT:       return "INTERCEPT";
        case STATE_RETURN_POSITION: return "RETURN_POSITION";
        case STATE_LINE_AVOID:      return "LINE_AVOID";
        case STATE_STUCK_RECOVERY:  return "STUCK_RECOVERY";
    }
    return "?";
}

void check(const char* label, GameState_t expected, GameState_t actual)
{
    const bool ok = (expected == actual);
    DBG_PRINT("  ");
    DBG_PRINT(label);
    DBG_PRINT(" expected=");
    DBG_PRINT(stateName(expected));
    DBG_PRINT(" actual=");
    DBG_PRINT(stateName(actual));
    DBG_PRINTLN(ok ? " -> PASS" : " -> FAIL");
    (ok ? g_pass : g_fail)++;
}

void checkTrue(const char* label, bool cond)
{
    DBG_PRINT("  ");
    DBG_PRINT(label);
    DBG_PRINTLN(cond ? " -> PASS" : " -> FAIL");
    (cond ? g_pass : g_fail)++;
}

// Default field: robot at origin, no ball, theta=0 (facing +Y).
DigitalField baseField()
{
    DigitalField f = {};
    f.self.x = 0.0f; f.self.y = -0.3f; f.self.theta = 0.0f;
    f.ball.mu[0] = 1.0f; f.ball.mu[1] = 0.0f; f.ball.mu[2] = 0.0f;
    f.ball.visible = 0;
    for (int i = 0; i < 4; ++i) f.sonar_mm[i] = 2400.0f;
    return f;
}
} // namespace

static void testTransitionsBasic()
{
    DBG_PRINTLN("-- Transitions: basic --");

    // Ball not visible, S0 locked (normal gameplay) → RETURN_POSITION.
    {
        DigitalField f = baseField();
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, false, false, /*s0Locked=*/true);
        check("ball lost post-S0-lock", STATE_RETURN_POSITION, out.state);
    }

    // Ball not visible, S0 not yet locked → stay in FIND_BALL (drive forward).
    {
        DigitalField f = baseField();
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_FIND_BALL, false, false, /*s0Locked=*/false);
        check("ball lost pre-S0-lock", STATE_FIND_BALL, out.state);
    }

    // Ball visible, not close to us → CHASE.
    {
        DigitalField f = baseField();
        f.ball.visible = 1;
        f.ball.bx = 0.5f; f.ball.by = 0.2f;
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_FIND_BALL, false, false, /*s0Locked=*/true);
        check("ball visible far", STATE_CHASE_BALL, out.state);
    }

    // Ball in cage → DRIBBLE (ball at 0, 0.1m in front of robot at -0.3).
    {
        DigitalField f = baseField();
        f.ball.visible = 1;
        f.ball.bx = 0.0f; f.ball.by = -0.2f;  // 0.1 m ahead (theta=0, body +Y is field +Y)
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, false, false, /*s0Locked=*/true);
        check("ball in cage far from goal", STATE_DRIBBLE, out.state);
    }

    // Ball in cage AND close to opponent goal → SHOOT.
    {
        DigitalField f = baseField();
        f.self.y = 0.4f;
        f.ball.visible = 1;
        f.ball.bx = 0.0f; f.ball.by = 0.5f;  // ball at (0, 0.5), goal at (0, 0.9), dist 0.4 < 0.6
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_DRIBBLE, false, false, /*s0Locked=*/true);
        check("ball in cage near goal", STATE_SHOOT, out.state);
    }

    // Enemy probability high → DEFEND regardless of our possession.
    {
        DigitalField f = baseField();
        f.ball.visible = 1;
        f.ball.bx = 0.0f; f.ball.by = 0.0f;
        f.ball.mu[0] = 0.2f; f.ball.mu[1] = 0.0f; f.ball.mu[2] = 0.8f;
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, false, false, /*s0Locked=*/true);
        check("enemy has ball", STATE_DEFEND, out.state);
    }

    // lineDetected overrides everything.
    {
        DigitalField f = baseField();
        f.ball.visible = 1;
        f.ball.bx = 0.0f; f.ball.by = 0.0f;
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, true, false, /*s0Locked=*/true);
        check("line detected", STATE_LINE_AVOID, out.state);
    }

    // stuckDetected overrides chase/return but not line.
    {
        DigitalField f = baseField();
        f.ball.visible = 1;
        f.ball.bx = 0.5f; f.ball.by = 0.0f;
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, false, true, /*s0Locked=*/true);
        check("stuck triggers recovery", STATE_STUCK_RECOVERY, out.state);
    }

    // line beats stuck.
    {
        DigitalField f = baseField();
        digitalFieldRefreshBallCache(f);
        auto out = StrategyFSM::update(f, STATE_CHASE_BALL, true, true, /*s0Locked=*/true);
        check("line beats stuck", STATE_LINE_AVOID, out.state);
    }
}

static void testDefendEnemyAware()
{
    DBG_PRINTLN("-- DEFEND target shifts toward confident enemy --");

    // Ball at (0.5, 0) in enemy possession, enemy at (-0.4, 0.1) with high
    // confidence. Target should lean toward x<0 (enemy side), not x>0 (ball
    // side).
    DigitalField f = baseField();
    f.ball.visible = 1;
    f.ball.bx = 0.5f; f.ball.by = 0.0f;
    f.ball.mu[0] = 0.2f; f.ball.mu[1] = 0.0f; f.ball.mu[2] = 0.8f;
    f.enemy[0].x = -0.4f; f.enemy[0].y = 0.1f;
    f.enemy[0].confidence = 0.9f;
    digitalFieldRefreshBallCache(f);

    auto out = StrategyFSM::update(f, STATE_DEFEND, false, false, /*s0Locked=*/true);
    check("DEFEND with enemy", STATE_DEFEND, out.state);

    // Now drop enemy confidence, target should flip sign of x (toward ball at
    // x=+0.5).
    DigitalField f2 = f;
    f2.enemy[0].confidence = 0.1f;
    auto out2 = StrategyFSM::update(f2, STATE_DEFEND, false, false, /*s0Locked=*/true);
    check("DEFEND falls back to ball", STATE_DEFEND, out2.state);

    // The drive vector's x-component sign differs between the two cases;
    // that's the observable behavior of the enemy-aware nudge.
    checkTrue("enemy-aware drive differs from ball-only drive",
              (out.drive.x * out2.drive.x) < 0.0f
              || fabsf(out.drive.x - out2.drive.x) > 1.0f);
}

static void testLostMsSaturation()
{
    DBG_PRINTLN("-- IMM lost_ms saturates instead of wrapping --");

    IMMBallTracker tracker;
    tracker.reset();
    SelfLocState self = {};
    EnemyState enemy = {};

    // Seed a valid observation so lastSeenMs_ is set.
    tracker.setPossessionHint(BALL_MODE_FREE);
    tracker.step(0.02f, true, 0.0f, 0.0f, 0.3f, self, enemy, /*nowMs=*/1000u);

    // Jump the clock past 65.5 s — the old cast would wrap to 0, the
    // saturating cast caps at 0xFFFF.
    tracker.step(0.02f, false, 0.0f, 0.0f, 0.0f, self, enemy,
                 /*nowMs=*/1000u + 90000u);

    const auto est = tracker.getEstimate();
    DBG_PRINT("  lost_ms=");
    DBG_PRINT(est.lost_ms);
    DBG_PRINT(" (expect 0xFFFF=65535)");
    DBG_PRINTLN();
    checkTrue("lost_ms saturated at 0xFFFF", est.lost_ms == 0xFFFFu);
}

void testStrategyFSM(const TestContext& /*ctx*/)
{
    DBG_PRINTLN("=== Strategy FSM Logic Tests ===");
    g_pass = 0;
    g_fail = 0;

    testTransitionsBasic();
    testDefendEnemyAware();
    testLostMsSaturation();

    DBG_PRINT("=== Summary: ");
    DBG_PRINT(g_pass);
    DBG_PRINT(" passed, ");
    DBG_PRINT(g_fail);
    DBG_PRINT(" failed ");
    DBG_PRINTLN(g_fail == 0 ? "-> ALL PASS" : "-> FAIL");

    while (true) delay(1000);
}
