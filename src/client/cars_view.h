#ifndef FOX_CLIENT_CARS_VIEW_H
#define FOX_CLIENT_CARS_VIEW_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"
#include "sim/network.h"
#include "sim/traffic.h"

#include <stdbool.h>
#include <stdint.h>

/* The visible cars: one small mesh per paint colour, drawn as GPU instances
 * placed and turned along their lanes, blended between the last two ticks so
 * motion is smooth at any frame rate. Where the device has no instancing,
 * each car is drawn on its own without turning. */

typedef struct FoxCarsView FoxCarsView;

FoxCarsView *FoxCarsViewCreate(LaiueGraphicsDeviceV2 *device);
void FoxCarsViewDestroy(FoxCarsView *view);

/* `blend` runs from 0 at the previous tick to 1 at the latest. Returns the
 * number of cars submitted. */
uint32_t FoxCarsViewDraw(FoxCarsView *view, const FoxTraffic *traffic, const FoxNetwork *network,
                         const FoxCameraView *camera, float blend);

#endif
