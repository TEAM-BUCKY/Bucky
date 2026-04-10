#ifndef BUCKY_SPLITSCREEN_H
#define BUCKY_SPLITSCREEN_H

#include <raylib.h>
#include "field/DigitalField.h"

struct PanelRect {
    float x, y, width, height;
};

class SplitScreen {
public:
    void update(int screenWidth, int screenHeight);

    PanelRect leftPanel() const { return left_; }
    PanelRect rightPanel() const { return right_; }

    // Convert field coordinates (meters) to screen pixels within a panel
    Vector2 fieldToScreen(const PanelRect& panel, float fx, float fy) const;

    // Convert screen pixels to field coordinates within a panel
    // Returns true if the point is inside the panel
    bool screenToField(const PanelRect& panel, float sx, float sy, float* fx, float* fy) const;

    // Get the scale (pixels per meter) for a panel
    float scale() const { return scale_; }

    // Field padding in pixels around the field within each panel
    static constexpr float PADDING = 30.0f;
    static constexpr float HUD_HEIGHT = 60.0f;
    static constexpr float DIVIDER_WIDTH = 2.0f;

private:
    PanelRect left_ = {};
    PanelRect right_ = {};
    float scale_ = 1.0f;
    float fieldOffsetX_ = 0.0f;
    float fieldOffsetY_ = 0.0f;
};

#endif // BUCKY_SPLITSCREEN_H
