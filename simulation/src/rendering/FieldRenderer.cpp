#include "FieldRenderer.h"
#include "field/DigitalField.h"
#include <cmath>

void FieldRenderer::draw(const SplitScreen& screen, const PanelRect& panel) const
{
    DigitalField ref{};
    float xMin = ref.field.x_min;
    float xMax = ref.field.x_max;
    float yMin = ref.field.y_min;
    float yMax = ref.field.y_max;
    float goalW = ref.field.goal_width;

    // Background
    DrawRectangle((int)panel.x, (int)panel.y, (int)panel.width, (int)panel.height,
                  {30, 30, 30, 255});

    // Field surface (dark green)
    Vector2 topLeft = screen.fieldToScreen(panel, xMin, yMax);
    Vector2 botRight = screen.fieldToScreen(panel, xMax, yMin);
    float fw = botRight.x - topLeft.x;
    float fh = botRight.y - topLeft.y;

    DrawRectangle((int)topLeft.x, (int)topLeft.y, (int)fw, (int)fh, {34, 120, 34, 255});

    // White boundary lines (2px thick)
    float lineThick = 2.0f;
    DrawRectangleLinesEx({topLeft.x, topLeft.y, fw, fh}, lineThick, WHITE);

    // Center line (horizontal, across the field at y=0)
    Vector2 cl = screen.fieldToScreen(panel, xMin, 0.0f);
    Vector2 cr = screen.fieldToScreen(panel, xMax, 0.0f);
    DrawLineEx(cl, cr, lineThick, WHITE);

    // Center circle (60cm = 0.6m diameter -> 0.3m radius)
    Vector2 center = screen.fieldToScreen(panel, 0.0f, 0.0f);
    float circleR = 0.30f * screen.scale();
    DrawCircleLinesV(center, circleR, WHITE);

    // Center dot
    DrawCircleV(center, 3.0f, WHITE);

    // Goals
    // Enemy goal at y_max
    Vector2 goalTL = screen.fieldToScreen(panel, -goalW / 2.0f, yMax);
    Vector2 goalTR = screen.fieldToScreen(panel, goalW / 2.0f, yMax);
    float goalDepth = 0.08f * screen.scale();
    DrawRectangleLinesEx({goalTL.x, goalTL.y - goalDepth,
                          goalTR.x - goalTL.x, goalDepth}, lineThick, {255, 200, 0, 255});

    // Own goal at y_min
    Vector2 goalBL = screen.fieldToScreen(panel, -goalW / 2.0f, yMin);
    Vector2 goalBR = screen.fieldToScreen(panel, goalW / 2.0f, yMin);
    DrawRectangleLinesEx({goalBL.x, goalBL.y,
                          goalBR.x - goalBL.x, goalDepth}, lineThick, {100, 150, 255, 255});

    if (showGrid)
        drawGrid(screen, panel);
}

void FieldRenderer::drawGrid(const SplitScreen& screen, const PanelRect& panel) const
{
    DigitalField ref{};
    Color gridColor = {255, 255, 255, 30};

    // Vertical lines every 0.3m
    for (float x = ref.field.x_min; x <= ref.field.x_max + 0.01f; x += 0.3f)
    {
        Vector2 top = screen.fieldToScreen(panel, x, ref.field.y_max);
        Vector2 bot = screen.fieldToScreen(panel, x, ref.field.y_min);
        DrawLineEx(top, bot, 1.0f, gridColor);
    }

    // Horizontal lines every 0.3m
    for (float y = ref.field.y_min; y <= ref.field.y_max + 0.01f; y += 0.3f)
    {
        Vector2 left = screen.fieldToScreen(panel, ref.field.x_min, y);
        Vector2 right = screen.fieldToScreen(panel, ref.field.x_max, y);
        DrawLineEx(left, right, 1.0f, gridColor);
    }
}
