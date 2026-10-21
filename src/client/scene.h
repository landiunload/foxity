#ifndef FOX_CLIENT_SCENE_H
#define FOX_CLIENT_SCENE_H

#include "app/modules.h"
#include "client/ui.h"
#include "graphics/graphics_device_v2.h"
#include "mesh/mesher_service.h"
#include "scene/math_service.h"
#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* What the mayor sees and steers: the city, its clock, the camera, the tools
 * and the views of the planet, the roads and the cars. Every engine technology
 * it uses is optional: without camera math nothing is drawn or picked, without
 * the mesher the ground is missing, and the city keeps running either way. */

typedef struct FoxScene FoxScene;

typedef struct FoxSceneDesc
{
    uint64_t seed;
    FoxDriveSide driveSide;
    uint32_t cars;
    bool demo;
    /* Non-zero builds a grid of this many roads each way instead of the
     * demo town. */
    uint32_t gridLines;
    int64_t gridSpacing;
    /* Metres from the focus to the eye; zero keeps the default. */
    double cameraDistance;
    /* Where the camera looks first, metres east and north of the pole. */
    double focusEast;
    double focusNorth;
    /* The tool in hand at the start. */
    uint32_t tool;
    /* Loaded engine modules: routing comes from them. */
    const FoxModules *modules;
    LaiueGraphicsDeviceV2 *device;
    const LaiueSceneMathServiceV1 *math;
    uint32_t mathServiceSize;
    const LaiueMesherServiceV1 *mesher;
    uint32_t mesherServiceSize;
} FoxSceneDesc;

/* One frame of player intent, independent of keyboards and mice. */
typedef struct FoxControls
{
    float zoomSteps;
    bool rotating;
    int32_t rotateX;
    int32_t rotateY;
    float moveForward;
    float moveRight;
    bool moveFast;
    /* The view in pixels. */
    uint32_t viewWidth;
    uint32_t viewHeight;
    /* Where the pointer is, from -1 at the left and bottom of the view to 1
     * at the right and top; `click` is a press this frame, `held` the button
     * being down. */
    bool pointer;
    float pointerX;
    float pointerY;
    bool click;
    bool held;
    /* -1 keeps the tool. */
    int32_t tool;
    bool nextRoadType;
    bool togglePause;
    /* Steps through one, three and ten times real time. */
    bool nextTimeScale;
    /* +1 or -1 asks for more or fewer cars; with moveFast ten times as
     * many. */
    int32_t carsChange;
    bool toggleStatistics;
    /* Whether presenting waits for the display: 1 on, 0 off, -1 the device
     * cannot tell; shown on the statistics panel. */
    int32_t verticalSync;
} FoxControls;

FoxScene *FoxSceneCreate(const FoxSceneDesc *desc);
void FoxSceneDestroy(FoxScene *scene);

/* Applies the controls, advances the city by the elapsed real time and
 * prepares the views; GPU uploads happen here, before the frame begins.
 * outViewProjection receives the camera; false when there is none. */
bool FoxSceneUpdate(FoxScene *scene, const FoxControls *controls, double realSeconds,
                    float aspectRatio, float outViewProjection[16]);
void FoxSceneDraw(FoxScene *scene);

/* Drops a road being placed, or else puts the tool down. False when there
 * was neither, so the caller may treat the key as a request to quit. */
bool FoxSceneCancel(FoxScene *scene);

#endif
