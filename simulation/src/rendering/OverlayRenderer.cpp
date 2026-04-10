#include "OverlayRenderer.h"
#include "helpers/Math.h"
#include <cmath>
#include <cstdio>

static constexpr float kSensorOffsets[4] = {0.0f, 0.5f * PI_F, PI_F, -0.5f * PI_F};

void OverlayRenderer::drawUncertaintyCircle(const SplitScreen& screen, const PanelRect& panel,
                                            float x, float y, float P_xy, Color color) const
{
    if (P_xy <= 0.0f) return;

    Vector2 pos = screen.fieldToScreen(panel, x, y);
    // P_xy is trace of covariance (var_x + var_y), radius = 2*sigma for 95% confidence
    float sigma = sqrtf(P_xy * 0.5f);  // average sigma
    float r = 2.0f * sigma * screen.scale();
    r = fmaxf(r, 3.0f);
    r = fminf(r, 200.0f);
    DrawCircleLinesV(pos, r, color);
}

void OverlayRenderer::drawSonarBeams(const SplitScreen& screen, const PanelRect& panel,
                                     const DigitalField& field) const
{
    float rx = field.self.x;
    float ry = field.self.y;
    float theta = field.self.theta;

    for (int i = 0; i < 4; ++i)
    {
        float angle = theta + kSensorOffsets[i];
        float beamLen = 0.5f;  // show 50cm beam length
        float ex = rx + beamLen * cosf(angle);
        float ey = ry + beamLen * sinf(angle);

        Vector2 start = screen.fieldToScreen(panel, rx, ry);
        Vector2 end = screen.fieldToScreen(panel, ex, ey);
        DrawLineEx(start, end, 1.0f, {0, 255, 255, 80});
    }
}

void OverlayRenderer::drawBallModeBar(const PanelRect& panel, const DigitalField& field) const
{
    // Draw ball mode probability bar at bottom of panel
    float barX = panel.x + 10.0f;
    float barY = panel.y + panel.height - 25.0f;
    float barW = 150.0f;
    float barH = 14.0f;

    // Background
    DrawRectangle((int)barX, (int)barY, (int)barW, (int)barH, {40, 40, 40, 200});

    // Three segments: FREE (green), FRIENDLY (blue), ENEMY (red)
    float freeW = barW * field.ball.mu[0];
    float friendlyW = barW * field.ball.mu[1];
    float enemyW = barW * field.ball.mu[2];

    DrawRectangle((int)barX, (int)barY, (int)freeW, (int)barH, {80, 200, 80, 220});
    DrawRectangle((int)(barX + freeW), (int)barY, (int)friendlyW, (int)barH, {80, 130, 255, 220});
    DrawRectangle((int)(barX + freeW + friendlyW), (int)barY, (int)enemyW, (int)barH, {255, 80, 80, 220});

    DrawRectangleLinesEx({barX, barY, barW, barH}, 1.0f, {200, 200, 200, 150});

    // Label
    DrawText("F  Fr  En", (int)barX, (int)(barY - 12.0f), 10, {200, 200, 200, 180});
}

void OverlayRenderer::draw(const SplitScreen& screen, const PanelRect& panel,
                           const DigitalField& field) const
{
    if (showUncertainty)
    {
        // Self-loc uncertainty
        drawUncertaintyCircle(screen, panel, field.self.x, field.self.y,
                              field.self.P_xy, {100, 180, 255, 120});

        // Ball uncertainty
        if (field.ball.visible || field.ball.lost_ms < 2000)
        {
            drawUncertaintyCircle(screen, panel, field.ball.bx, field.ball.by,
                                  field.ball.P_xy, {255, 200, 80, 120});
        }
    }

    if (showSensors)
    {
        drawSonarBeams(screen, panel, field);

        // IR bearing line toward detected ball
        if (field.ball.visible)
        {
            Vector2 robotPos = screen.fieldToScreen(panel, field.self.x, field.self.y);
            Vector2 ballPos = screen.fieldToScreen(panel, field.ball.bx, field.ball.by);
            DrawLineEx(robotPos, ballPos, 1.0f, {255, 200, 0, 60});
        }
    }

    drawBallModeBar(panel, field);
}
