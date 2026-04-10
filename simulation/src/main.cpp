#include <raylib.h>

#include "simulation/SimLoop.h"
#include "rendering/SplitScreen.h"
#include "rendering/FieldRenderer.h"
#include "rendering/EntityRenderer.h"
#include "rendering/OverlayRenderer.h"
#include "ui/Controls.h"
#include "ui/HUD.h"

int main()
{
    const int initialWidth = 1200;
    const int initialHeight = 540;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(initialWidth, initialHeight, "Bucky Simulation — Ground Truth vs Robot Belief");
    SetTargetFPS(60);

    SimLoop sim;
    SplitScreen screen;
    FieldRenderer fieldRenderer;
    EntityRenderer entityRenderer;
    OverlayRenderer overlayRenderer;
    Controls controls;
    HUD hud;

    while (!WindowShouldClose())
    {
        int w = GetScreenWidth();
        int h = GetScreenHeight();
        float dt = GetFrameTime();

        // Update layout
        screen.update(w, h);

        // Input
        controls.update(sim, screen, fieldRenderer, entityRenderer, overlayRenderer);

        // Simulation step
        sim.step(dt);

        // Render
        BeginDrawing();
        ClearBackground({20, 20, 20, 255});

        // --- Left panel: Robot's belief ---
        {
            PanelRect panel = screen.leftPanel();
            BeginScissorMode((int)panel.x, (int)panel.y, (int)panel.width, (int)panel.height);

            fieldRenderer.draw(screen, panel);
            entityRenderer.drawBelief(screen, panel, sim.belief());
            overlayRenderer.draw(screen, panel, sim.belief());

            EndScissorMode();
        }

        // --- Right panel: Ground truth ---
        {
            PanelRect panel = screen.rightPanel();
            BeginScissorMode((int)panel.x, (int)panel.y, (int)panel.width, (int)panel.height);

            fieldRenderer.draw(screen, panel);
            entityRenderer.drawGroundTruth(screen, panel, sim.groundTruth());

            EndScissorMode();
        }

        // --- HUD (spans full width) ---
        hud.draw(sim, screen, fieldRenderer, entityRenderer, overlayRenderer, w, h);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
