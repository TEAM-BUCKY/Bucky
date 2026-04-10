#ifndef BUCKY_CONTROLS_H
#define BUCKY_CONTROLS_H

#include "rendering/SplitScreen.h"
#include "simulation/SimLoop.h"
#include "rendering/FieldRenderer.h"
#include "rendering/EntityRenderer.h"
#include "rendering/OverlayRenderer.h"

enum DragTarget { DRAG_NONE, DRAG_BALL, DRAG_ENEMY0, DRAG_ENEMY1, DRAG_ROBOT };

class Controls {
public:
    void update(SimLoop& sim, const SplitScreen& screen,
                FieldRenderer& fieldRenderer, EntityRenderer& entityRenderer,
                OverlayRenderer& overlayRenderer);

    DragTarget currentDrag() const { return dragTarget_; }

private:
    DragTarget dragTarget_ = DRAG_NONE;
    void handleKeyboard(SimLoop& sim, FieldRenderer& fieldRenderer,
                        EntityRenderer& entityRenderer, OverlayRenderer& overlayRenderer);
    void handleMouse(SimLoop& sim, const SplitScreen& screen);
};

#endif // BUCKY_CONTROLS_H
