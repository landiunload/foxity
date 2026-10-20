#ifndef FOX_CLIENT_ROADS_VIEW_H
#define FOX_CLIENT_ROADS_VIEW_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"
#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* The visible roads: asphalt, markings and the paved areas of
 * intersections. Roads are gathered into meshes per 128 m patch of space, so
 * a city of thousands of roads costs a few hundred draws and GPU handles,
 * and a change rebuilds only the patches it touched, within a time budget. A
 * planned road shows as a preview until it is built. */

typedef struct FoxRoadsView FoxRoadsView;

FoxRoadsView *FoxRoadsViewCreate(LaiueGraphicsDeviceV2 *device);
void FoxRoadsViewDestroy(FoxRoadsView *view);

/* Follows the network's changes. Call before the frame begins. */
void FoxRoadsViewUpdate(FoxRoadsView *view, const FoxNetwork *network, double budgetSeconds);

/* Shows a planned centre line, or nothing for count zero. `valid` picks the
 * colour of a road that can be built or of one that cannot. */
void FoxRoadsViewSetPreview(FoxRoadsView *view, const FoxVec *centre, uint32_t count,
                            FoxRoadType type, bool valid);

void FoxRoadsViewDraw(FoxRoadsView *view, const FoxCameraView *camera);

#endif
