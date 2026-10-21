#ifndef FOX_CLIENT_ROADS_VIEW_H
#define FOX_CLIENT_ROADS_VIEW_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"
#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* The visible roads: asphalt, intersections, markings, signs and traffic
 * lights. Roads are gathered into meshes per 128 m patch of space, so a city
 * of thousands of roads costs a few hundred draws and GPU handles, and a
 * change rebuilds only the patches it touched, within a time budget. Each
 * patch has two meshes: the asphalt, always drawn, and the details, drawn
 * only near the eye, where they can be seen and the depth buffer can keep
 * paint apart from the road under it. The lit lamps of traffic lights follow
 * the signals every frame. A planned road shows as a preview until it is
 * built, with the tools' guides and snap marks on the ground beside it. */

typedef struct FoxRoadsView FoxRoadsView;

FoxRoadsView *FoxRoadsViewCreate(LaiueGraphicsDeviceV2 *device);
void FoxRoadsViewDestroy(FoxRoadsView *view);

/* Follows the network's changes. Call before the frame begins. */
void FoxRoadsViewUpdate(FoxRoadsView *view, const FoxNetwork *network, double budgetSeconds);

/* Shows a planned centre line, or nothing for count zero. `valid` picks the
 * colour of a road that can be built or of one that cannot. */
void FoxRoadsViewSetPreview(FoxRoadsView *view, const FoxVec *centre, uint32_t count,
                            FoxRoadType type, bool valid);

/* What the tools draw on the ground besides the planned road. */
typedef struct FoxOverlayLine
{
    const FoxVec *points;
    uint32_t count;
    bool dashed;
    uint32_t color;
} FoxOverlayLine;

typedef struct FoxOverlayRing
{
    FoxVec centre;
    double radius;
    uint32_t color;
} FoxOverlayRing;

/* Replaces the lines and rings shown; zero counts clear them. */
void FoxRoadsViewSetOverlay(FoxRoadsView *view, const FoxOverlayLine *lines, uint32_t lineCount,
                            const FoxOverlayRing *rings, uint32_t ringCount);

void FoxRoadsViewDraw(FoxRoadsView *view, const FoxCameraView *camera, const FoxNetwork *network);

#endif
