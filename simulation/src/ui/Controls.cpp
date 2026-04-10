#include "Controls.h"
#include <cmath>

void Controls::handleKeyboard(SimLoop& sim, FieldRenderer& fieldRenderer,
                              EntityRenderer& entityRenderer, OverlayRenderer& overlayRenderer)
{
    if (IsKeyPressed(KEY_SPACE))
        sim.paused = !sim.paused;

    if (IsKeyPressed(KEY_S) && sim.paused)
        sim.step(1.0f / 60.0f);

    if (IsKeyPressed(KEY_R))
        sim.reset();

    // Speed multiplier 1-5
    for (int i = 1; i <= 5; ++i)
        if (IsKeyPressed(KEY_ONE + i - 1))
            sim.speedMultiplier = i;

    if (IsKeyPressed(KEY_U))
        overlayRenderer.showUncertainty = !overlayRenderer.showUncertainty;

    if (IsKeyPressed(KEY_V))
        entityRenderer.showVelocity = !entityRenderer.showVelocity;

    if (IsKeyPressed(KEY_I))
        overlayRenderer.showSensors = !overlayRenderer.showSensors;

    if (IsKeyPressed(KEY_G))
        fieldRenderer.showGrid = !fieldRenderer.showGrid;

    if (IsKeyPressed(KEY_N))
    {
        NoiseLevel current = sim.sensorSim().getNoiseLevel();
        if (current == NOISE_NONE) sim.sensorSim().setNoiseLevel(NOISE_LOW);
        else if (current == NOISE_LOW) sim.sensorSim().setNoiseLevel(NOISE_HIGH);
        else sim.sensorSim().setNoiseLevel(NOISE_NONE);
    }
}

void Controls::handleMouse(SimLoop& sim, const SplitScreen& screen)
{
    PanelRect right = screen.rightPanel();
    Vector2 mouse = GetMousePosition();
    float fx, fy;

    // Only interact with right panel (ground truth)
    bool inRight = screen.screenToField(right, mouse.x, mouse.y, &fx, &fy);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inRight)
    {
        GroundTruth& truth = sim.groundTruth();

        // Find closest entity
        float bestDist = 0.15f;  // max grab distance in meters
        dragTarget_ = DRAG_NONE;

        // Ball
        float db = sqrtf((fx - truth.ball.x) * (fx - truth.ball.x) +
                         (fy - truth.ball.y) * (fy - truth.ball.y));
        if (db < bestDist) { bestDist = db; dragTarget_ = DRAG_BALL; }

        // Enemies
        for (int i = 0; i < 2; ++i)
        {
            if (!truth.enemies[i].active) continue;
            float de = sqrtf((fx - truth.enemies[i].x) * (fx - truth.enemies[i].x) +
                             (fy - truth.enemies[i].y) * (fy - truth.enemies[i].y));
            if (de < bestDist) { bestDist = de; dragTarget_ = (DragTarget)(DRAG_ENEMY0 + i); }
        }

        // Robot (shift+click)
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
        {
            float dr = sqrtf((fx - truth.robot.x) * (fx - truth.robot.x) +
                             (fy - truth.robot.y) * (fy - truth.robot.y));
            if (dr < 0.2f) { dragTarget_ = DRAG_ROBOT; }
        }
    }

    // Right-click to toggle enemy
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && inRight)
    {
        GroundTruth& truth = sim.groundTruth();
        // Toggle nearest inactive enemy, or place one
        for (int i = 0; i < 2; ++i)
        {
            if (!truth.enemies[i].active)
            {
                truth.enemies[i].active = true;
                truth.enemies[i].x = fx;
                truth.enemies[i].y = fy;
                truth.enemies[i].vx = 0.0f;
                truth.enemies[i].vy = 0.0f;
                break;
            }
            else
            {
                float de = sqrtf((fx - truth.enemies[i].x) * (fx - truth.enemies[i].x) +
                                 (fy - truth.enemies[i].y) * (fy - truth.enemies[i].y));
                if (de < 0.15f)
                {
                    truth.enemies[i].active = false;
                    break;
                }
            }
        }
    }

    // Drag
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && dragTarget_ != DRAG_NONE && inRight)
    {
        GroundTruth& truth = sim.groundTruth();
        switch (dragTarget_)
        {
            case DRAG_BALL:
                truth.ball.x = fx;
                truth.ball.y = fy;
                truth.ball.vx = 0.0f;
                truth.ball.vy = 0.0f;
                break;
            case DRAG_ENEMY0:
                truth.enemies[0].x = fx;
                truth.enemies[0].y = fy;
                break;
            case DRAG_ENEMY1:
                truth.enemies[1].x = fx;
                truth.enemies[1].y = fy;
                break;
            case DRAG_ROBOT:
                truth.robot.x = fx;
                truth.robot.y = fy;
                break;
            default: break;
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        dragTarget_ = DRAG_NONE;
}

void Controls::update(SimLoop& sim, const SplitScreen& screen,
                      FieldRenderer& fieldRenderer, EntityRenderer& entityRenderer,
                      OverlayRenderer& overlayRenderer)
{
    handleKeyboard(sim, fieldRenderer, entityRenderer, overlayRenderer);
    handleMouse(sim, screen);
}
