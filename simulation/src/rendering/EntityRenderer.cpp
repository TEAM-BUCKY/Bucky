#include "EntityRenderer.h"
#include <cmath>

static constexpr float ROBOT_RADIUS_M = 0.09f;
static constexpr float BALL_RADIUS_M = 0.021f;
static constexpr float ENEMY_RADIUS_M = 0.09f;
static constexpr float GAP_DEPTH_M = 0.035f;
static constexpr float GAP_HALF_WIDTH_M = 0.04f;

void EntityRenderer::drawRobot(const SplitScreen& screen, const PanelRect& panel,
                               float x, float y, float theta, Color color) const
{
    Vector2 pos = screen.fieldToScreen(panel, x, y);
    float r = ROBOT_RADIUS_M * screen.scale();
    float gapDepth = GAP_DEPTH_M * screen.scale();
    float gapHW = GAP_HALF_WIDTH_M * screen.scale();

    // Draw robot body circle
    DrawCircleV(pos, r, color);
    DrawCircleLinesV(pos, r, WHITE);

    // Draw the dribble gap as a dark notch at the front.
    // The gap is a small rectangle cut into the front of the robot.
    // Screen coords: theta direction, with y-axis inverted.
    float fwdX = cosf(theta);
    float fwdY = -sinf(theta);  // screen y inverted
    float latX = -fwdY;
    float latY = fwdX;

    // Gap center is at the front edge of the robot
    float gapCX = pos.x + fwdX * (r - gapDepth * 0.5f);
    float gapCY = pos.y + fwdY * (r - gapDepth * 0.5f);

    // Four corners of the gap rectangle
    Vector2 g0 = {gapCX - latX * gapHW - fwdX * gapDepth * 0.5f,
                  gapCY - latY * gapHW - fwdY * gapDepth * 0.5f};
    Vector2 g1 = {gapCX + latX * gapHW - fwdX * gapDepth * 0.5f,
                  gapCY + latY * gapHW - fwdY * gapDepth * 0.5f};
    Vector2 g2 = {gapCX + latX * gapHW + fwdX * gapDepth * 0.5f,
                  gapCY + latY * gapHW + fwdY * gapDepth * 0.5f};
    Vector2 g3 = {gapCX - latX * gapHW + fwdX * gapDepth * 0.5f,
                  gapCY - latY * gapHW + fwdY * gapDepth * 0.5f};

    // Fill gap with dark color (looks like a notch)
    DrawTriangle(g0, g2, g1, {20, 20, 20, 255});
    DrawTriangle(g0, g3, g2, {20, 20, 20, 255});
    // Gap outline
    DrawLineEx(g0, g1, 1.0f, {200, 200, 200, 150});
    DrawLineEx(g0, g3, 1.0f, {200, 200, 200, 150});
    DrawLineEx(g1, g2, 1.0f, {200, 200, 200, 150});

    // Heading arrow
    float arrowLen = r * 1.5f;
    Vector2 tip = {
        pos.x + arrowLen * fwdX,
        pos.y + arrowLen * fwdY
    };
    DrawLineEx(pos, tip, 2.5f, WHITE);

    // Arrowhead
    float headLen = 6.0f;
    float headAngle = 0.4f;
    Vector2 left = {
        tip.x - headLen * cosf(theta - headAngle),
        tip.y + headLen * sinf(theta - headAngle)
    };
    Vector2 right = {
        tip.x - headLen * cosf(theta + headAngle),
        tip.y + headLen * sinf(theta + headAngle)
    };
    DrawTriangle(tip, right, left, WHITE);
}

void EntityRenderer::drawBall(const SplitScreen& screen, const PanelRect& panel,
                              float x, float y, Color color) const
{
    Vector2 pos = screen.fieldToScreen(panel, x, y);
    float r = fmaxf(BALL_RADIUS_M * screen.scale(), 5.0f);
    DrawCircleV(pos, r, color);
    DrawCircleLinesV(pos, r, {255, 255, 255, 180});
}

void EntityRenderer::drawEnemy(const SplitScreen& screen, const PanelRect& panel,
                               float x, float y, float confidence, Color color) const
{
    Vector2 pos = screen.fieldToScreen(panel, x, y);
    float r = ENEMY_RADIUS_M * screen.scale();
    unsigned char alpha = (unsigned char)(confidence * 255.0f);
    Color c = {color.r, color.g, color.b, alpha};
    DrawCircleV(pos, r, c);
    DrawCircleLinesV(pos, r, {255, 255, 255, alpha});
}

void EntityRenderer::drawVelocityArrow(const SplitScreen& screen, const PanelRect& panel,
                                       float x, float y, float vx, float vy, Color color) const
{
    float speed = sqrtf(vx * vx + vy * vy);
    if (speed < 0.01f) return;

    Vector2 start = screen.fieldToScreen(panel, x, y);
    Vector2 end = screen.fieldToScreen(panel, x + vx * 0.3f, y + vy * 0.3f);
    DrawLineEx(start, end, 1.5f, color);
}

void EntityRenderer::drawGroundTruth(const SplitScreen& screen, const PanelRect& panel,
                                     const GroundTruth& truth) const
{
    // Draw enemies first (behind)
    for (int i = 0; i < 2; ++i)
    {
        if (truth.enemies[i].active)
        {
            drawEnemy(screen, panel, truth.enemies[i].x, truth.enemies[i].y,
                      1.0f, {220, 50, 50, 255});
            if (showVelocity)
                drawVelocityArrow(screen, panel, truth.enemies[i].x, truth.enemies[i].y,
                                  truth.enemies[i].vx, truth.enemies[i].vy, {255, 100, 100, 180});
        }
    }

    // Draw robot (before ball so ball renders on top when captured)
    drawRobot(screen, panel, truth.robot.x, truth.robot.y, truth.robot.theta,
              {60, 120, 220, 255});
    if (showVelocity)
        drawVelocityArrow(screen, panel, truth.robot.x, truth.robot.y,
                          truth.robot.vx, truth.robot.vy, {120, 180, 255, 180});

    // Draw ball
    Color ballColor = truth.ball.captured ? Color{255, 220, 50, 255} : Color{255, 160, 0, 255};
    drawBall(screen, panel, truth.ball.x, truth.ball.y, ballColor);
    if (showVelocity && !truth.ball.captured)
        drawVelocityArrow(screen, panel, truth.ball.x, truth.ball.y,
                          truth.ball.vx, truth.ball.vy, {255, 200, 100, 180});
}

void EntityRenderer::drawBelief(const SplitScreen& screen, const PanelRect& panel,
                                const DigitalField& field) const
{
    // Draw enemies
    for (int i = 0; i < 2; ++i)
    {
        if (field.enemy[i].confidence > 0.01f)
        {
            drawEnemy(screen, panel, field.enemy[i].x, field.enemy[i].y,
                      field.enemy[i].confidence, {220, 50, 50, 255});
        }
    }

    // Draw self estimate (before ball)
    float vxField = 0.0f, vyField = 0.0f;
    bodyToField(field.self.vx, field.self.vy, field.self.theta, &vxField, &vyField);

    drawRobot(screen, panel, field.self.x, field.self.y, field.self.theta,
              {60, 120, 220, 255});
    if (showVelocity)
        drawVelocityArrow(screen, panel, field.self.x, field.self.y,
                          vxField, vyField, {120, 180, 255, 180});

    // Draw ball estimate
    if (field.ball.visible || field.ball.lost_ms < 2000)
    {
        unsigned char ballAlpha = field.ball.visible ? 255 : (unsigned char)(200 * fmaxf(0.0f, 1.0f - field.ball.lost_ms / 2000.0f));
        drawBall(screen, panel, field.ball.bx, field.ball.by, {255, 160, 0, ballAlpha});
        if (showVelocity)
            drawVelocityArrow(screen, panel, field.ball.bx, field.ball.by,
                              field.ball.bvx, field.ball.bvy, {255, 200, 100, 180});
    }
}
