#ifndef BUCKY_FIELDRENDERER_H
#define BUCKY_FIELDRENDERER_H

#include <raylib.h>
#include "SplitScreen.h"

class FieldRenderer {
public:
    // Draw the field (green surface, white lines, goals, center circle)
    // Uses DigitalField constants so it auto-updates from source.
    void draw(const SplitScreen& screen, const PanelRect& panel) const;

    bool showGrid = false;

private:
    void drawGrid(const SplitScreen& screen, const PanelRect& panel) const;
};

#endif // BUCKY_FIELDRENDERER_H
