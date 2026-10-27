#ifndef FOX_CLIENT_PLANET_VIEW_H
#define FOX_CLIENT_PLANET_VIEW_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"
#include "mesh/mesher_service.h"
#include "sim/planet.h"

#include <stdbool.h>
#include <stdint.h>

/* The visible ground. Space around the planet is divided into cubes on an
 * octree of detail levels: a cube of level L spans 32 lattice samples spaced
 * 2^L cells apart, so cubes near the eye show half-metre detail and those on
 * the far side span a kilometre. Only cubes the surface may pass through are
 * meshed, from the density field, nearest first and within a time budget per
 * frame, so building never makes a frame late; until a cube is ready its
 * cached children or its coarser ancestor stand in for it, and nothing finer
 * is drawn inside a cube that stands in, so two resolutions never overlap. */

typedef struct FoxPlanetView FoxPlanetView;

typedef struct FoxPlanetViewStats
{
    uint32_t visibleCubes;
    uint32_t drawnCubes;
    uint32_t cachedCubes;
    uint32_t builtThisFrame;
} FoxPlanetViewStats;

FoxPlanetView *FoxPlanetViewCreate(LaiueGraphicsDeviceV2 *device,
                                   const LaiueMesherServiceV1 *mesher, uint32_t mesherServiceSize,
                                   const LaiueSceneMathServiceV1 *math);
/* Releases every GPU resource; call between frames. */
void FoxPlanetViewDestroy(FoxPlanetView *view);

/* Chooses the cubes for this camera and meshes missing ones until
 * `budgetSeconds` have passed. Call before the frame begins. */
void FoxPlanetViewUpdate(FoxPlanetView *view, const FoxPlanet *planet, const FoxCameraView *camera,
                         double budgetSeconds);

/* Marks cached cubes over a changed box for rebuilding; their old meshes
 * stay on screen until the new ones are ready. */
void FoxPlanetViewInvalidate(FoxPlanetView *view, const FoxPlanetBox *box);

void FoxPlanetViewDraw(FoxPlanetView *view, const FoxCameraView *camera);

FoxPlanetViewStats FoxPlanetViewGetStats(const FoxPlanetView *view);

#endif
