#ifndef FOX_CLIENT_HUD_H
#define FOX_CLIENT_HUD_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"

#include <stdbool.h>
#include <stdint.h>

/* Text and graphs over the view: a panel in the top left corner with lines
 * in a 5x7 pixel font that knows Latin and Cyrillic, and next to them a
 * column of graphs. It is held just beyond the near plane and turned with
 * the camera; the mesh is rebuilt when the content changes and drawn every
 * frame as one instance that faces the eye. Needs generic instancing;
 * without it the panel is not shown. */

typedef struct FoxHud FoxHud;

/* A history to plot, oldest value first: bars from zero to `maximum`, and
 * optionally a second series stacked on them. */
typedef struct FoxHudGraph
{
    /* UTF-8, shown above the graph. */
    const char *title;
    const float *values;
    const float *stacked;
    uint32_t count;
    float maximum;
    uint32_t color;
    uint32_t stackedColor;
} FoxHudGraph;

/* NULL when the device cannot show it. */
FoxHud *FoxHudCreate(LaiueGraphicsDeviceV2 *device);
void FoxHudDestroy(FoxHud *hud);

/* Lines of UTF-8 separated by '\n', letters shown as capitals; a line that
 * starts with "--" is a heading. The graphs stand in a column to the right. */
void FoxHudSetContent(FoxHud *hud, const char *text, const FoxHudGraph *graphs,
                      uint32_t graphCount);
void FoxHudDraw(FoxHud *hud, const FoxCameraView *camera);

#endif
