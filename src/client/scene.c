#include "client/scene.h"

#include "app/city.h"
#include "app/log.h"
#include "client/camera.h"
#include "client/cars_view.h"
#include "client/hud.h"
#include "client/planet_view.h"
#include "client/roads_view.h"
#include "sim/clock.h"

#include "mod/application.h"

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Per frame, the views may spend this long building meshes. */
#define PLANET_BUDGET_SECONDS 0.0025
#define ROADS_BUDGET_SECONDS 0.0015
/* Holding a ground tool works this often. */
#define GROUND_INTERVAL_SECONDS 0.12
#define DRIVE_ZONE_RADIUS FOX_METRES(150)
#define CARS_STEP 50u
#define REPORT_SECONDS 30.0
/* The statistics panel is refreshed this often. */
#define STATISTICS_SECONDS 0.25
#define STATISTICS_TEXT 4096u

static const double speedPresets[] = {1.0, 3.0, 10.0};
static const char *const toolNames[FOX_TOOL_COUNT] = {
    [FOX_TOOL_ROAD] = "road",
    [FOX_TOOL_CURVE] = "curved road",
    [FOX_TOOL_BULLDOZE] = "bulldoze",
    [FOX_TOOL_DIG] = "dig",
    [FOX_TOOL_RAISE] = "raise ground",
    [FOX_TOOL_DRIVE_ZONE] = "drive-side zone",
    [FOX_TOOL_SIGNALS] = "traffic lights",
    [FOX_TOOL_SIGNS] = "junction signs",
    [FOX_TOOL_SPEED_LIMIT] = "speed limits",
};
static const char *const signNames[FOX_SIGN_COUNT] = {
    [FOX_SIGN_AUTO] = "chosen by the roads",
    [FOX_SIGN_NONE] = "no sign",
    [FOX_SIGN_PRIORITY] = "priority road",
    [FOX_SIGN_YIELD] = "give way",
    [FOX_SIGN_STOP] = "stop",
};
/* The limits the speed tool steps through, km/h. */
static const int32_t speedLimits[] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130};
static const char *const roadResults[] = {
    [FOX_ROAD_OK] = "built",
    [FOX_ROAD_INVALID] = "not a road",
    [FOX_ROAD_TOO_SHORT] = "too short, or too close to another junction",
    [FOX_ROAD_TOO_LONG] = "too long",
    [FOX_ROAD_NO_GROUND] = "no ground under it",
    [FOX_ROAD_CROWDED] = "too many roads at one junction",
    [FOX_ROAD_OVERLAP] = "it would lie on another road",
    [FOX_ROAD_OUT_OF_MEMORY] = "out of memory",
};

struct FoxScene
{
    FoxCity city;
    FoxClock clock;
    FoxCamera camera;
    FoxCameraView view;
    const LaiueSceneMathServiceV1 *math;
    FoxPlanetView *planet;
    FoxRoadsView *roads;
    FoxCarsView *cars;
    FoxHud *hud;
    char *statisticsText;
    double resumeScale;
    /* What the panel measures between its refreshes. */
    double windowSeconds;
    double windowSlowestFrame;
    uint64_t windowTicks;
    double windowTickSeconds;
    double windowSlowestTick;

    /* Points of the road being placed: start, then for curves the bend. */
    FoxVec placed[2];
    FoxVec hit;
    /* The preview shown for, to rebuild it only when it changes. */
    FoxVec previewEnd;
    FoxVec *previewPoints;
    double groundCooldown;
    double reportTimer;
    FoxTool tool;
    FoxRoadType roadType;
    uint32_t timeScale;
    uint32_t windowFrames;
    uint32_t placedCount;
    uint32_t previewPlaced;
    bool viewReady;
    bool picking;
    bool statistics;
    bool hasHit;
};

FoxScene *FoxSceneCreate(const FoxSceneDesc *desc)
{
    if (desc == NULL)
    {
        return NULL;
    }
    FoxScene *scene = calloc(1u, sizeof(*scene));
    if (scene == NULL)
    {
        return NULL;
    }
    scene->previewPoints = malloc(FOX_ROAD_MAX_POINTS * sizeof(*scene->previewPoints));
    const FoxCityDesc cityDesc = {
        .seed = desc->seed,
        .driveSide = desc->driveSide,
        .cars = desc->cars,
        .demo = desc->demo,
        .gridLines = desc->gridLines,
        .gridSpacing = desc->gridSpacing,
        .modules = desc->modules,
    };
    scene->statisticsText = malloc(STATISTICS_TEXT);
    scene->statistics = true;
    if (scene->previewPoints == NULL || scene->statisticsText == NULL ||
        !FoxCityCreate(&scene->city, &cityDesc) ||
        !FoxClockInit(&scene->clock, FOX_CLOCK_TICKS_PER_SECOND,
                      FOX_CLOCK_DEFAULT_MAX_ADVANCE_SECONDS))
    {
        FoxSceneDestroy(scene);
        return NULL;
    }
    scene->resumeScale = 1.0;
    scene->tool = FOX_TOOL_ROAD;
    scene->roadType = FOX_ROAD_STREET;

    if (desc->math != NULL && desc->mathServiceSize >= LAIUE_SCENE_MATH_SERVICE_V1_CAMERA_SIZE &&
        desc->math->structSize >= LAIUE_SCENE_MATH_SERVICE_V1_CAMERA_SIZE &&
        desc->math->matrix4LookAt != NULL && desc->math->matrix4Perspective != NULL &&
        desc->math->matrix4Multiply != NULL && desc->math->matrix4ExtractFrustumPlanes != NULL)
    {
        scene->math = desc->math;
        scene->picking = desc->mathServiceSize >= LAIUE_SCENE_MATH_SERVICE_V1_PICKING_SIZE &&
                         desc->math->structSize >= LAIUE_SCENE_MATH_SERVICE_V1_PICKING_SIZE &&
                         desc->math->viewRayThroughScreenPoint != NULL;
        if (!scene->picking)
        {
            FoxLog(FOX_LOG_WARNING, "camera math cannot cast rays; the tools are unavailable");
        }
    }
    else
    {
        FoxLog(FOX_LOG_WARNING, "camera math is unavailable; the city is not drawn");
    }
    if (desc->device != NULL)
    {
        scene->planet = scene->math == NULL
                            ? NULL
                            : FoxPlanetViewCreate(desc->device, desc->mesher,
                                                  desc->mesherServiceSize, scene->math);
        if (scene->planet == NULL)
        {
            FoxLog(FOX_LOG_WARNING, "surface meshing is unavailable; the ground is not drawn");
        }
        scene->roads = FoxRoadsViewCreate(desc->device);
        scene->cars = FoxCarsViewCreate(desc->device);
        scene->hud = FoxHudCreate(desc->device);
        if (scene->hud == NULL)
        {
            FoxLog(FOX_LOG_WARNING, "the device cannot show the statistics panel");
        }
        if (scene->roads == NULL || scene->cars == NULL)
        {
            FoxLog(FOX_LOG_WARNING, "roads or cars cannot be drawn");
        }
    }
    const FoxPlanet *planet = FoxWorldPlanet(scene->city.world);
    FoxVec focus = {(int64_t)llround(desc->focusEast * FOX_UNITS_PER_METRE),
                    (int64_t)llround(desc->focusNorth * FOX_UNITS_PER_METRE),
                    FoxPlanetRadius(planet)};
    (void)FoxPlanetGround(planet, focus, &focus);
    FoxCameraInit(&scene->camera, focus);
    if (desc->cameraDistance > 0.0)
    {
        scene->camera.distance = desc->cameraDistance;
        FoxCameraZoom(&scene->camera, 0.0f);
    }
    FoxCityReport(&scene->city, "city founded");
    FoxLog(FOX_LOG_INFO, "tools: 1 road, 2 curved road, 3 bulldoze, 4 dig, 5 raise, 6 drive "
                         "zone, 7 traffic lights, 8 junction signs, 9 speed limits (Shift "
                         "lowers); T road type; V time speed, Space pause; E/G more or fewer "
                         "cars (Shift: ten times as many); F3 statistics");
    return scene;
}

void FoxSceneDestroy(FoxScene *scene)
{
    if (scene == NULL)
    {
        return;
    }
    FoxHudDestroy(scene->hud);
    free(scene->statisticsText);
    FoxCarsViewDestroy(scene->cars);
    FoxRoadsViewDestroy(scene->roads);
    FoxPlanetViewDestroy(scene->planet);
    FoxCityDestroy(&scene->city);
    free(scene->previewPoints);
    free(scene);
}

bool FoxSceneCancel(FoxScene *scene)
{
    if (scene == NULL || scene->placedCount == 0u)
    {
        return false;
    }
    scene->placedCount = 0u;
    FoxRoadsViewSetPreview(scene->roads, NULL, 0u, scene->roadType, false);
    return true;
}

static void ApplyTimeControls(FoxScene *scene, const FoxControls *controls)
{
    if (controls->nextTimeScale)
    {
        scene->timeScale =
            (scene->timeScale + 1u) % (uint32_t)(sizeof(speedPresets) / sizeof(speedPresets[0]));
        scene->resumeScale = speedPresets[scene->timeScale];
        (void)FoxClockSetTimeScale(&scene->clock, scene->resumeScale);
        FoxLog(FOX_LOG_INFO, "time runs at x%g", scene->resumeScale);
    }
    if (controls->togglePause)
    {
        const bool paused = scene->clock.timeScale == 0.0;
        (void)FoxClockSetTimeScale(&scene->clock, paused ? scene->resumeScale : 0.0);
        FoxLog(FOX_LOG_INFO, "%s", paused ? "time resumes" : "time is paused");
    }
    if (controls->toggleStatistics)
    {
        scene->statistics = !scene->statistics;
    }
    if (controls->carsChange != 0)
    {
        const uint32_t target = FoxTrafficTarget(FoxWorldTraffic(scene->city.world));
        const uint32_t step = controls->moveFast ? CARS_STEP * 10u : CARS_STEP;
        const uint32_t next = controls->carsChange > 0 ? target + step
                                                       : (target > step ? target - step : 0u);
        FoxWorldSetTargetVehicles(scene->city.world, next);
        FoxLog(FOX_LOG_INFO, "the city aims for %u cars", next);
    }
}

static void ApplyCameraControls(FoxScene *scene, const FoxControls *controls, double realSeconds)
{
    if (controls->zoomSteps != 0.0f)
    {
        FoxCameraZoom(&scene->camera, controls->zoomSteps);
    }
    if (controls->rotating)
    {
        FoxCameraRotate(&scene->camera, controls->rotateX, controls->rotateY);
    }
    if (controls->moveForward != 0.0f || controls->moveRight != 0.0f)
    {
        FoxCameraPan(&scene->camera, FoxWorldPlanet(scene->city.world), controls->moveForward,
                     controls->moveRight, realSeconds * (controls->moveFast ? 3.0 : 1.0));
    }
}

/* Where the pointer meets the ground. */
static void Pick(FoxScene *scene, const FoxControls *controls)
{
    scene->hasHit = false;
    if (!controls->pointer || !scene->picking || !scene->viewReady)
    {
        return;
    }
    float origin[3];
    float direction[3];
    if (!scene->math->viewRayThroughScreenPoint(scene->view.viewProjection, controls->pointerX,
                                                controls->pointerY, origin, direction))
    {
        return;
    }
    const FoxVec start = {
        scene->view.renderOrigin.x + (int64_t)llround((double)origin[0] * FOX_UNITS_PER_METRE),
        scene->view.renderOrigin.y + (int64_t)llround((double)origin[1] * FOX_UNITS_PER_METRE),
        scene->view.renderOrigin.z + (int64_t)llround((double)origin[2] * FOX_UNITS_PER_METRE),
    };
    const FoxVec heading = {(int64_t)llround((double)direction[0] * FOX_UNIT),
                            (int64_t)llround((double)direction[1] * FOX_UNIT),
                            (int64_t)llround((double)direction[2] * FOX_UNIT)};
    scene->hasHit = FoxPlanetRaycast(FoxWorldPlanet(scene->city.world), start, heading,
                                     FOX_METRES(20000), &scene->hit);
}

/* A clicked point snaps to a nearby node, so roads join where they meet. */
static FoxVec Snap(const FoxScene *scene, FoxVec point)
{
    const FoxNetwork *network = FoxWorldNetwork(scene->city.world);
    const uint32_t node = FoxNetworkNearestNode(network, point, FOX_ROAD_SNAP_DISTANCE);
    FoxNodeView view;
    return node != FOX_NETWORK_NONE && FoxNetworkNode(network, node, &view) ? view.position : point;
}

static FoxRoadPlan PlanTo(const FoxScene *scene, FoxVec end)
{
    FoxRoadPlan plan = {
        .start = scene->placed[0],
        .end = end,
        .type = scene->roadType,
    };
    if (scene->tool == FOX_TOOL_CURVE && scene->placedCount == 2u)
    {
        plan.curved = true;
        plan.control = scene->placed[1];
    }
    return plan;
}

static void UpdatePreview(FoxScene *scene)
{
    const bool placing = (scene->tool == FOX_TOOL_ROAD || scene->tool == FOX_TOOL_CURVE) &&
                         scene->placedCount != 0u && scene->hasHit;
    if (!placing)
    {
        if (scene->previewPlaced != 0u)
        {
            FoxRoadsViewSetPreview(scene->roads, NULL, 0u, scene->roadType, false);
            scene->previewPlaced = 0u;
        }
        return;
    }
    const FoxVec end = Snap(scene, scene->hit);
    if (scene->previewPlaced == scene->placedCount && FoxVecEqual(end, scene->previewEnd))
    {
        return;
    }
    scene->previewPlaced = scene->placedCount;
    scene->previewEnd = end;
    const FoxRoadPlan plan = PlanTo(scene, end);
    const uint32_t count = FoxNetworkPlanCentre(FoxWorldPlanet(scene->city.world), &plan,
                                                scene->previewPoints, FOX_ROAD_MAX_POINTS);
    int64_t length = 0;
    for (uint32_t index = 1u; index < count; ++index)
    {
        length += FoxVecDistance(scene->previewPoints[index - 1u], scene->previewPoints[index]);
    }
    FoxRoadsViewSetPreview(scene->roads, scene->previewPoints, count, scene->roadType,
                           length >= FOX_ROAD_MIN_LENGTH && length <= FOX_ROAD_MAX_LENGTH);
}

static void PlaceRoad(FoxScene *scene)
{
    const FoxVec point = Snap(scene, scene->hit);
    const uint32_t needed = scene->tool == FOX_TOOL_CURVE ? 2u : 1u;
    if (scene->placedCount < needed)
    {
        scene->placed[scene->placedCount++] = point;
        return;
    }
    const FoxRoadPlan plan = PlanTo(scene, point);
    const FoxRoadResult result = FoxWorldBuildRoad(scene->city.world, &plan);
    FoxLog(result == FOX_ROAD_OK ? FOX_LOG_INFO : FOX_LOG_WARNING, "%s: %s",
           FoxRoadTypeGet(scene->roadType)->name, roadResults[result]);
    if (result == FOX_ROAD_OK)
    {
        /* Keep drawing from the end, as city builders do. */
        scene->placed[0] = point;
        scene->placedCount = 1u;
    }
    scene->previewPlaced = 0u;
}

static void Bulldoze(FoxScene *scene)
{
    const FoxNetwork *network = FoxWorldNetwork(scene->city.world);
    const uint32_t road = FoxNetworkNearestRoad(network, scene->hit, FOX_METRES(8), NULL);
    if (road != FOX_NETWORK_NONE && FoxWorldRemoveRoad(scene->city.world, road))
    {
        FoxLog(FOX_LOG_INFO, "road removed");
    }
}

static void ShapeGround(FoxScene *scene, double realSeconds)
{
    scene->groundCooldown -= realSeconds;
    if (scene->groundCooldown > 0.0)
    {
        return;
    }
    scene->groundCooldown = GROUND_INTERVAL_SECONDS;
    /* The sphere grows with the view, so the hand shows at any zoom. */
    const double metres = fmin(fmax(scene->camera.distance / 50.0, 2.0), 8.0);
    const int64_t radius = (int64_t)(metres * FOX_UNITS_PER_METRE);
    FoxPlanetBox changed;
    const uint32_t samples = scene->tool == FOX_TOOL_DIG
                                 ? FoxWorldDig(scene->city.world, scene->hit, radius, &changed)
                                 : FoxWorldFill(scene->city.world, scene->hit, radius, &changed);
    if (samples != 0u)
    {
        FoxPlanetViewInvalidate(scene->planet, &changed);
    }
}

static void Zone(FoxScene *scene)
{
    const FoxNetwork *network = FoxWorldNetwork(scene->city.world);
    const FoxDriveSide side = FoxNetworkDriveSideAt(network, scene->hit) == FOX_DRIVE_RIGHT
                                  ? FOX_DRIVE_LEFT
                                  : FOX_DRIVE_RIGHT;
    if (FoxWorldAddDriveZone(scene->city.world, scene->hit, DRIVE_ZONE_RADIUS, side))
    {
        FoxLog(FOX_LOG_INFO, "this district now keeps %s", side == FOX_DRIVE_RIGHT ? "right" : "left");
    }
    else
    {
        FoxLog(FOX_LOG_WARNING, "no more drive-side zones can be laid");
    }
}

static void ToggleSignals(FoxScene *scene)
{
    const FoxNetwork *network = FoxWorldNetwork(scene->city.world);
    const uint32_t node = FoxNetworkNearestNode(network, scene->hit, FOX_METRES(15));
    FoxNodeView view;
    if (node == FOX_NETWORK_NONE || !FoxNetworkNode(network, node, &view))
    {
        return;
    }
    if (FoxWorldSetSignals(scene->city.world, node, !view.signals))
    {
        FoxLog(FOX_LOG_INFO, "traffic lights %s", view.signals ? "removed" : "put up");
    }
    else
    {
        FoxLog(FOX_LOG_WARNING, "traffic lights need three roads or more");
    }
}

/* The road under the pointer and which of its ends is nearer. */
static uint32_t PickRoad(const FoxScene *scene, uint32_t *outEnd, FoxRoadView *outRoad)
{
    const FoxNetwork *network = FoxWorldNetwork(scene->city.world);
    int64_t along = 0;
    const uint32_t road = FoxNetworkNearestRoad(network, scene->hit, FOX_METRES(8), &along);
    if (road == FOX_NETWORK_NONE || !FoxNetworkRoad(network, road, outRoad))
    {
        return FOX_NETWORK_NONE;
    }
    *outEnd = along * 2 < outRoad->length ? 0u : 1u;
    return road;
}

static void NextSign(FoxScene *scene)
{
    uint32_t end = 0u;
    FoxRoadView road;
    const uint32_t index = PickRoad(scene, &end, &road);
    if (index == FOX_NETWORK_NONE)
    {
        return;
    }
    static const FoxSign order[FOX_SIGN_COUNT] = {FOX_SIGN_PRIORITY, FOX_SIGN_YIELD, FOX_SIGN_STOP,
                                                  FOX_SIGN_NONE, FOX_SIGN_AUTO};
    FoxSign next = FOX_SIGN_PRIORITY;
    for (uint32_t step = 0u; step < FOX_SIGN_COUNT; ++step)
    {
        if (order[step] == road.signSetting[end])
        {
            next = order[(step + 1u) % FOX_SIGN_COUNT];
        }
    }
    if (FoxWorldSetSign(scene->city.world, index, end, next) &&
        FoxNetworkRoad(FoxWorldNetwork(scene->city.world), index, &road))
    {
        FoxLog(FOX_LOG_INFO, "sign: %s, in force: %s", signNames[next], signNames[road.sign[end]]);
    }
}

static void StepSpeedLimit(FoxScene *scene, bool lower)
{
    uint32_t end = 0u;
    FoxRoadView road;
    const uint32_t index = PickRoad(scene, &end, &road);
    if (index == FOX_NETWORK_NONE)
    {
        return;
    }
    const size_t count = sizeof(speedLimits) / sizeof(speedLimits[0]);
    /* Past either end of the list the road goes back to its type's limit. */
    int32_t next = 0;
    for (size_t step = 0u; step < count; ++step)
    {
        const int32_t speed = FOX_KMH(speedLimits[lower ? count - 1u - step : step]);
        if (lower ? speed < road.speedLimit - 1 : speed > road.speedLimit + 1)
        {
            next = speed;
            break;
        }
    }
    if (FoxWorldSetSpeedLimit(scene->city.world, index, next) &&
        FoxNetworkRoad(FoxWorldNetwork(scene->city.world), index, &road))
    {
        FoxLog(FOX_LOG_INFO, "speed limit %ld km/h%s",
               (long)lround((double)road.speedLimit * 3.6 / FOX_UNITS_PER_METRE),
               road.speedSet ? "" : " (the road's own)");
    }
}

static void ApplyTools(FoxScene *scene, const FoxControls *controls, double realSeconds)
{
    if (controls->tool >= 0 && controls->tool < FOX_TOOL_COUNT && controls->tool != (int32_t)scene->tool)
    {
        scene->tool = (FoxTool)controls->tool;
        scene->placedCount = 0u;
        FoxLog(FOX_LOG_INFO, "tool: %s", toolNames[scene->tool]);
    }
    if (controls->nextRoadType)
    {
        scene->roadType = (FoxRoadType)((scene->roadType + 1) % FOX_ROAD_TYPE_COUNT);
        scene->previewPlaced = 0u;
        FoxLog(FOX_LOG_INFO, "road type: %s", FoxRoadTypeGet(scene->roadType)->name);
    }
    UpdatePreview(scene);
    if (!scene->hasHit)
    {
        return;
    }
    switch (scene->tool)
    {
    case FOX_TOOL_ROAD:
    case FOX_TOOL_CURVE:
        if (controls->click)
        {
            PlaceRoad(scene);
        }
        break;
    case FOX_TOOL_BULLDOZE:
        if (controls->click)
        {
            Bulldoze(scene);
        }
        break;
    case FOX_TOOL_DIG:
    case FOX_TOOL_RAISE:
        if (controls->held)
        {
            ShapeGround(scene, realSeconds);
        }
        else
        {
            scene->groundCooldown = 0.0;
        }
        break;
    case FOX_TOOL_DRIVE_ZONE:
        if (controls->click)
        {
            Zone(scene);
        }
        break;
    case FOX_TOOL_SIGNALS:
        if (controls->click)
        {
            ToggleSignals(scene);
        }
        break;
    case FOX_TOOL_SIGNS:
        if (controls->click)
        {
            NextSign(scene);
        }
        break;
    case FOX_TOOL_SPEED_LIMIT:
        if (controls->click)
        {
            StepSpeedLimit(scene, controls->moveFast);
        }
        break;
    case FOX_TOOL_COUNT:
        break;
    }
}

static double Kmh(int64_t units)
{
    return (double)units * 3.6 / FOX_UNITS_PER_METRE;
}

#if defined(__GNUC__) || defined(__clang__)
#define FOX_PRINTF_LIKE(string, first) __attribute__((format(printf, string, first)))
#else
#define FOX_PRINTF_LIKE(string, first)
#endif

/* Appends to the panel's text, keeping it terminated. */
static void Append(char *text, size_t *used, const char *format, ...) FOX_PRINTF_LIKE(3, 4);

static void Append(char *text, size_t *used, const char *format, ...)
{
    if (*used + 1u >= STATISTICS_TEXT)
    {
        return;
    }
    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(text + *used, STATISTICS_TEXT - *used, format, arguments);
    va_end(arguments);
    if (written > 0)
    {
        *used += (size_t)written < STATISTICS_TEXT - *used ? (size_t)written : STATISTICS_TEXT - *used - 1u;
    }
}

/* Everything about the city and the frame, for the panel. */
static void BuildStatistics(FoxScene *scene)
{
    const FoxWorld *world = scene->city.world;
    const FoxNetwork *network = FoxWorldNetwork(world);
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const FoxNetworkStats roads = FoxNetworkGetStats(network);
    const FoxTrafficStats cars = FoxTrafficGetStats(traffic);
    const uint64_t tick = FoxWorldTick(world);
    const uint64_t seconds = tick / FOX_TRAFFIC_TICKS_PER_SECOND;
    const double frames = (double)scene->windowFrames;
    char *text = scene->statisticsText;
    size_t used = 0u;
    text[0] = '\0';
    Append(text, &used, "-- FOXITY  (F3 hides)\n");
    Append(text, &used, "FPS %.0f  frame %.1f ms  worst %.1f ms\n",
           scene->windowSeconds > 0.0 ? frames / scene->windowSeconds : 0.0,
           frames > 0.0 ? scene->windowSeconds * 1000.0 / frames : 0.0,
           scene->windowSlowestFrame * 1000.0);
    Append(text, &used, "tick %.3f ms  worst %.3f ms  %.0f ticks/s\n",
           scene->windowTicks != 0u ? scene->windowTickSeconds * 1000.0 / (double)scene->windowTicks
                                    : 0.0,
           scene->windowSlowestTick * 1000.0,
           scene->windowSeconds > 0.0 ? (double)scene->windowTicks / scene->windowSeconds : 0.0);
    Append(text, &used, "time x%g%s  city %02u:%02u:%02u\n", scene->resumeScale,
           scene->clock.timeScale == 0.0 ? " PAUSED" : "", (unsigned)(seconds / 3600u),
           (unsigned)(seconds / 60u % 60u), (unsigned)(seconds % 60u));
    Append(text, &used, "tool %s  road %s\n", toolNames[scene->tool],
           FoxRoadTypeGet(scene->roadType)->name);
    Append(text, &used, "-- ROADS\n");
    Append(text, &used, "roads %u  %.2f km  lanes %u  %.2f km\n", roads.roads,
           (double)roads.roadLength / FOX_METRES(1000), roads.lanes,
           (double)roads.laneLength / FOX_METRES(1000));
    Append(text, &used, "street %u  avenue %u  boulevard %u\n",
           roads.roadsByType[FOX_ROAD_STREET], roads.roadsByType[FOX_ROAD_AVENUE],
           roads.roadsByType[FOX_ROAD_BOULEVARD]);
    Append(text, &used, "one-way %u  highway %u  limits set %u\n",
           roads.roadsByType[FOX_ROAD_ONE_WAY], roads.roadsByType[FOX_ROAD_HIGHWAY],
           roads.speedLimitsSet);
    Append(text, &used, "junctions %u  lights %u  bends %u  ends %u\n", roads.junctions,
           roads.signalled, roads.bends, roads.deadEnds);
    Append(text, &used, "signs: priority %u  yield %u  stop %u\n", roads.signs[FOX_SIGN_PRIORITY],
           roads.signs[FOX_SIGN_YIELD], roads.signs[FOX_SIGN_STOP]);
    Append(text, &used, "turns %u  conflicts %u\n", roads.connections, roads.conflicts);
    Append(text, &used, "-- TRAFFIC\n");
    Append(text, &used, "vehicles %u / %u  cars %u  trucks %u\n", cars.vehicles,
           FoxTrafficTarget(traffic), cars.vehicles - cars.trucks, cars.trucks);
    Append(text, &used, "moving %u  standing %u (%.0f%%)  >30s %u\n",
           cars.vehicles - cars.standing, cars.standing,
           cars.vehicles != 0u ? 100.0 * cars.standing / cars.vehicles : 0.0, cars.waitingLong);
    Append(text, &used, "speed mean %.1f  max %.1f km/h\n", Kmh(cars.meanSpeed),
           Kmh(cars.maxSpeed));
    uint32_t bands[8] = {0u};
    for (uint32_t slot = 0u; slot < FoxTrafficCapacity(traffic); ++slot)
    {
        FoxVehicleView vehicle;
        if (FoxTrafficVehicle(traffic, slot, &vehicle))
        {
            const uint32_t band = (uint32_t)(Kmh(vehicle.speed) / 10.0);
            ++bands[band < 7u ? band : 7u];
        }
    }
    for (uint32_t band = 0u; band < 8u; ++band)
    {
        /* A bar of up to thirty cells, scaled to all vehicles. */
        const uint32_t cells =
            cars.vehicles != 0u ? (uint32_t)((uint64_t)bands[band] * 30u / cars.vehicles) : 0u;
        char bar[32];
        memset(bar, '#', cells);
        bar[cells] = '\0';
        if (band < 7u)
        {
            Append(text, &used, "%3u-%-3u %-30s %u\n", band * 10u, band * 10u + 10u, bar,
                   bands[band]);
        }
        else
        {
            Append(text, &used, "%3u+    %-30s %u\n", band * 10u, bar, bands[band]);
        }
    }
    const double minutes = (double)tick / FOX_TRAFFIC_TICKS_PER_SECOND / 60.0;
    Append(text, &used, "arrived %llu (%.0f/min)\n", (unsigned long long)cars.arrived,
           minutes > 0.0 ? (double)cars.arrived / minutes : 0.0);
    Append(text, &used, "spawned %llu  stuck %llu (%.1f/min)\n", (unsigned long long)cars.spawned,
           (unsigned long long)cars.stuck, minutes > 0.0 ? (double)cars.stuck / minutes : 0.0);
    Append(text, &used, "E/G cars +-50  Shift x10   V speed  Space pause");
    FoxHudSetText(scene->hud, text);
    scene->windowSeconds = 0.0;
    scene->windowFrames = 0u;
    scene->windowSlowestFrame = 0.0;
    scene->windowTicks = 0u;
    scene->windowTickSeconds = 0.0;
    scene->windowSlowestTick = 0.0;
}

bool FoxSceneUpdate(FoxScene *scene, const FoxControls *controls, double realSeconds,
                    float aspectRatio, float outViewProjection[16])
{
    if (scene == NULL || controls == NULL)
    {
        return false;
    }
    ApplyTimeControls(scene, controls);
    const uint64_t ticks = FoxClockAdvance(&scene->clock, realSeconds);
    for (uint64_t tick = 0u; tick < ticks; ++tick)
    {
        const double before = LaiueApplicationMonotonicSeconds();
        FoxWorldAdvance(scene->city.world, 1u);
        const double spent = LaiueApplicationMonotonicSeconds() - before;
        scene->windowTickSeconds += spent;
        scene->windowSlowestTick = fmax(scene->windowSlowestTick, spent);
    }
    scene->windowTicks += ticks;
    scene->windowSeconds += realSeconds;
    ++scene->windowFrames;
    scene->windowSlowestFrame = fmax(scene->windowSlowestFrame, realSeconds);
    if (scene->hud != NULL && scene->statistics && scene->windowSeconds >= STATISTICS_SECONDS)
    {
        BuildStatistics(scene);
    }
    scene->reportTimer += realSeconds;
    if (scene->reportTimer >= REPORT_SECONDS)
    {
        scene->reportTimer = 0.0;
        FoxCityReport(&scene->city, "running");
    }
    ApplyCameraControls(scene, controls, realSeconds);
    scene->viewReady = scene->math != NULL &&
                       FoxCameraBuildView(&scene->camera, scene->math, aspectRatio, &scene->view);
    if (!scene->viewReady)
    {
        return false;
    }
    Pick(scene, controls);
    ApplyTools(scene, controls, realSeconds);
    FoxPlanetViewUpdate(scene->planet, FoxWorldPlanet(scene->city.world), &scene->view,
                        PLANET_BUDGET_SECONDS);
    FoxRoadsViewUpdate(scene->roads, FoxWorldNetwork(scene->city.world), ROADS_BUDGET_SECONDS);
    if (outViewProjection != NULL)
    {
        memcpy(outViewProjection, scene->view.viewProjection, sizeof(scene->view.viewProjection));
    }
    return true;
}

void FoxSceneDraw(FoxScene *scene)
{
    if (scene == NULL || !scene->viewReady)
    {
        return;
    }
    FoxPlanetViewDraw(scene->planet, &scene->view);
    FoxRoadsViewDraw(scene->roads, &scene->view, FoxWorldNetwork(scene->city.world));
    (void)FoxCarsViewDraw(scene->cars, FoxWorldTraffic(scene->city.world),
                          FoxWorldNetwork(scene->city.world), &scene->view,
                          FoxClockInterpolation(&scene->clock));
    if (scene->statistics)
    {
        FoxHudDraw(scene->hud, &scene->view);
    }
}
