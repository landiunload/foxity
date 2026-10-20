#ifndef FOX_CLIENT_HUD_H
#define FOX_CLIENT_HUD_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"

#include <stdbool.h>
#include <stdint.h>

/* Text over the view: a panel in the top left corner with lines in a 5x7
 * pixel font, held just beyond the near plane and turned with the camera.
 * The mesh is rebuilt only when the text changes; every frame it is drawn as
 * one instance that faces the eye. Needs generic instancing; without it the
 * panel is not shown. */

typedef struct FoxHud FoxHud;

/* NULL when the device cannot show it. */
FoxHud *FoxHudCreate(LaiueGraphicsDeviceV2 *device);
void FoxHudDestroy(FoxHud *hud);

/* Lines separated by '\n'; letters show as capitals. Rebuilds the mesh
 * only when the text differs from the last. */
void FoxHudSetText(FoxHud *hud, const char *text);
void FoxHudDraw(FoxHud *hud, const FoxCameraView *camera);

#endif
