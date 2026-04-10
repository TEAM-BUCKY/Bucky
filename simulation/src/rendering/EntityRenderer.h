#ifndef BUCKY_ENTITYRENDERER_H
#define BUCKY_ENTITYRENDERER_H

#include <raylib.h>
#include "SplitScreen.h"
#include "simulation/GroundTruth.h"
#include "field/DigitalField.h"

class EntityRenderer {
public:
    bool showVelocity = true;

    // Draw ground truth entities (right panel)
    void drawGroundTruth(const SplitScreen& screen, const PanelRect& panel,
                         const GroundTruth& truth) const;

    // Draw belief entities (left panel)
    void drawBelief(const SplitScreen& screen, const PanelRect& panel,
                    const DigitalField& field) const;

private:
    void drawRobot(const SplitScreen& screen, const PanelRect& panel,
                   float x, float y, float theta, Color color) const;

    void drawBall(const SplitScreen& screen, const PanelRect& panel,
                  float x, float y, Color color) const;

    void drawEnemy(const SplitScreen& screen, const PanelRect& panel,
                   float x, float y, float confidence, Color color) const;

    void drawVelocityArrow(const SplitScreen& screen, const PanelRect& panel,
                           float x, float y, float vx, float vy, Color color) const;
};

#endif // BUCKY_ENTITYRENDERER_H
