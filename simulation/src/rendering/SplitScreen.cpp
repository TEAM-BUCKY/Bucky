#include "SplitScreen.h"
#include <cmath>

void SplitScreen::update(int screenWidth, int screenHeight)
{
    float halfW = (screenWidth - DIVIDER_WIDTH) * 0.5f;
    float availH = screenHeight - HUD_HEIGHT;

    left_  = {0.0f, 0.0f, halfW, availH};
    right_ = {halfW + DIVIDER_WIDTH, 0.0f, halfW, availH};

    // Compute scale from field dimensions (from DigitalField defaults)
    DigitalField ref{};
    float fieldW = ref.field.x_max - ref.field.x_min;  // 2.4m
    float fieldH = ref.field.y_max - ref.field.y_min;   // 1.8m

    float scaleX = (halfW - 2.0f * PADDING) / fieldW;
    float scaleY = (availH - 2.0f * PADDING) / fieldH;
    scale_ = fminf(scaleX, scaleY);

    // Center the field within the panel
    fieldOffsetX_ = PADDING + ((halfW - 2.0f * PADDING) - fieldW * scale_) * 0.5f;
    fieldOffsetY_ = PADDING + ((availH - 2.0f * PADDING) - fieldH * scale_) * 0.5f;
}

Vector2 SplitScreen::fieldToScreen(const PanelRect& panel, float fx, float fy) const
{
    DigitalField ref{};
    // Map field x [-1.2, 1.2] to [0, fieldW*scale]
    // Map field y [-0.9, 0.9] to [fieldH*scale, 0] (y-axis flipped: +y in field = up on screen)
    float sx = panel.x + fieldOffsetX_ + (fx - ref.field.x_min) * scale_;
    float sy = panel.y + fieldOffsetY_ + (ref.field.y_max - fy) * scale_;
    return {sx, sy};
}

bool SplitScreen::screenToField(const PanelRect& panel, float sx, float sy, float* fx, float* fy) const
{
    if (sx < panel.x || sx > panel.x + panel.width ||
        sy < panel.y || sy > panel.y + panel.height)
        return false;

    DigitalField ref{};
    *fx = ref.field.x_min + (sx - panel.x - fieldOffsetX_) / scale_;
    *fy = ref.field.y_max - (sy - panel.y - fieldOffsetY_) / scale_;
    return true;
}
