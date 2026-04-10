#include "HUD.h"
#include <cstdio>
#include <cmath>

static const char* noiseNames[] = {"NONE", "LOW", "HIGH"};

void HUD::drawPanelLabel(const PanelRect& panel, const char* label) const
{
    int fontSize = 16;
    int textW = MeasureText(label, fontSize);
    DrawText(label, (int)(panel.x + panel.width * 0.5f - textW * 0.5f),
             (int)(panel.y + 6.0f), fontSize, {220, 220, 220, 220});
}

void HUD::draw(const SimLoop& sim, const SplitScreen& screen,
               const FieldRenderer& fieldRenderer,
               const EntityRenderer& entityRenderer,
               const OverlayRenderer& overlayRenderer,
               int screenWidth, int screenHeight) const
{
    PanelRect left = screen.leftPanel();
    PanelRect right = screen.rightPanel();

    // Panel labels
    drawPanelLabel(left, "ROBOT BELIEF");
    drawPanelLabel(right, "GROUND TRUTH");

    // Divider line
    float divX = left.x + left.width;
    DrawLineEx({divX, 0.0f}, {divX, (float)screenHeight}, SplitScreen::DIVIDER_WIDTH,
               {100, 100, 100, 255});

    // --- Bottom HUD bar ---
    float hudY = screenHeight - SplitScreen::HUD_HEIGHT;
    DrawRectangle(0, (int)hudY, screenWidth, (int)SplitScreen::HUD_HEIGHT, {25, 25, 25, 240});
    DrawLineEx({0.0f, hudY}, {(float)screenWidth, hudY}, 1.0f, {80, 80, 80, 255});

    int y = (int)hudY + 8;
    int fontSize = 14;
    int col1 = 12;
    int col2 = 220;
    int col3 = 440;
    int col4 = 680;

    const DigitalField& field = sim.belief();

    // Column 1: Strategy state
    char buf[128];
    Color stateColor = sim.paused ? Color{255, 200, 80, 255} : Color{80, 255, 120, 255};
    snprintf(buf, sizeof(buf), "State: %s", sim.gameStateName());
    DrawText(buf, col1, y, fontSize, stateColor);

    snprintf(buf, sizeof(buf), "Speed: %dx  %s", sim.speedMultiplier,
             sim.paused ? "[PAUSED]" : "");
    DrawText(buf, col1, y + 18, fontSize, {180, 180, 180, 255});

    // Column 2: Ball info
    snprintf(buf, sizeof(buf), "Ball: (%.2f, %.2f)", field.ball.bx, field.ball.by);
    DrawText(buf, col2, y, fontSize, {255, 200, 100, 255});

    snprintf(buf, sizeof(buf), "Mode: F=%.0f%% Fr=%.0f%% En=%.0f%%",
             field.ball.mu[0] * 100, field.ball.mu[1] * 100, field.ball.mu[2] * 100);
    DrawText(buf, col2, y + 18, fontSize, {200, 200, 200, 255});

    // Column 3: Self-loc info
    snprintf(buf, sizeof(buf), "Self: (%.2f, %.2f) %.1f deg",
             field.self.x, field.self.y,
             field.self.theta * 180.0f / 3.14159f);
    DrawText(buf, col3, y, fontSize, {100, 180, 255, 255});

    snprintf(buf, sizeof(buf), "P_xy: %.4f", field.self.P_xy);
    DrawText(buf, col3, y + 18, fontSize, {180, 180, 180, 255});

    // Column 4: Toggles and FPS
    snprintf(buf, sizeof(buf), "Noise: %s  FPS: %d", noiseNames[sim.sensorSim().getNoiseLevel()],
             GetFPS());
    DrawText(buf, col4, y, fontSize, {180, 180, 180, 255});

    snprintf(buf, sizeof(buf), "Time: %.1fs", field.timestamp_ms / 1000.0f);
    DrawText(buf, col4, y + 18, fontSize, {150, 150, 150, 255});

    // Key hints (far right)
    int hintX = screenWidth - 280;
    DrawText("[Space] Pause  [R] Reset  [1-5] Speed", hintX, y, 10, {120, 120, 120, 200});
    DrawText("[U]ncert [V]eloc [I]R [G]rid [N]oise [S]tep", hintX, y + 14, 10, {120, 120, 120, 200});

    // Strategy state label on belief panel
    snprintf(buf, sizeof(buf), "%s", sim.gameStateName());
    int labelW = MeasureText(buf, 14);
    DrawText(buf, (int)(left.x + left.width - labelW - 10),
             (int)(left.y + left.height - 40), 14, stateColor);
}
