#ifndef FOX_CLIENT_SCENE_H
#define FOX_CLIENT_SCENE_H

#include "app/modules.h"
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

typedef enum FoxTool
{
    /* Click a start and an end; the next road continues from the end. */
    FOX_TOOL_ROAD = 0,
    /* Click a start, a point to bend towards and an end. */
    FOX_TOOL_CURVE,
    FOX_TOOL_BULLDOZE,
    /* Held: dig the ground away or pile it up under the pointer. */
    FOX_TOOL_DIG,
    FOX_TOOL_RAISE,
    /* Click: the district around the point drives on the other side. */
    FOX_TOOL_DRIVE_ZONE,
    /* Click an intersection: traffic lights on or off. */
    FOX_TOOL_SIGNALS,
    /* Click near a road end: the next sign for the traffic arriving there. */
    FOX_TOOL_SIGNS,
    /* Click a road: a higher speed limit; with Shift a lower one. */
    FOX_TOOL_SPEED_LIMIT,
    FOX_TOOL_COUNT,
} FoxTool;

typedef struct FoxScene FoxScene;

typedef struct FoxSceneDesc
{
    uint64_t seed;
    FoxDriveSide driveSide;
    uint32_t cars;
    bool demo;
    /* Metres from the focus to the eye; zero keeps the default. */
    double cameraDistance;
    /* Where the camera looks first, metres east and north of the pole. */
    double focusEast;
    double focusNorth;
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
    /* +1 or -1 asks for more or fewer cars. */
    int32_t carsChange;
} FoxControls;

FoxScene *FoxSceneCreate(const FoxSceneDesc *desc);
void FoxSceneDestroy(FoxScene *scene);

/* Applies the controls, advances the city by the elapsed real time and
 * prepares the views; GPU uploads happen here, before the frame begins.
 * outViewProjection receives the camera; false when there is none. */
bool FoxSceneUpdate(FoxScene *scene, const FoxControls *controls, double realSeconds,
                    float aspectRatio, float outViewProjection[16]);
void FoxSceneDraw(FoxScene *scene);

/* Drops a road being placed. False when nothing was being placed, so the
 * caller may treat the key as a request to quit. */
bool FoxSceneCancel(FoxScene *scene);

#endif
