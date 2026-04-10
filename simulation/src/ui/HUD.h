#ifndef BUCKY_HUD_H
#define BUCKY_HUD_H

#include <raylib.h>
#include "simulation/SimLoop.h"
#include "rendering/SplitScreen.h"
#include "rendering/OverlayRenderer.h"
#include "rendering/FieldRenderer.h"
#include "rendering/EntityRenderer.h"

class HUD {
public:
    void draw(const SimLoop& sim, const SplitScreen& screen,
              const FieldRenderer& fieldRenderer,
              const EntityRenderer& entityRenderer,
              const OverlayRenderer& overlayRenderer, int screenWidth, int screenHeight) const;

private:
    void drawPanelLabel(const PanelRect& panel, const char* label) const;
};

#endif // BUCKY_HUD_H
