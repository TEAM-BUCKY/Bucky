#ifndef BUCKY_OVERLAYRENDERER_H
#define BUCKY_OVERLAYRENDERER_H

#include <raylib.h>
#include "SplitScreen.h"
#include "field/DigitalField.h"
#include "simulation/GroundTruth.h"

class OverlayRenderer {
public:
    bool showUncertainty = true;
    bool showSensors = true;

    // Draw belief-side overlays: uncertainty ellipses, sensor rays, mode bar
    void draw(const SplitScreen& screen, const PanelRect& panel,
              const DigitalField& field) const;

private:
    void drawUncertaintyCircle(const SplitScreen& screen, const PanelRect& panel,
                               float x, float y, float P_xy, Color color) const;

    void drawSonarBeams(const SplitScreen& screen, const PanelRect& panel,
                        const DigitalField& field) const;

    void drawBallModeBar(const PanelRect& panel, const DigitalField& field) const;
};

#endif // BUCKY_OVERLAYRENDERER_H
