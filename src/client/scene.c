#include "client/scene.h"

#include "app/city.h"
#include "app/log.h"
#include "client/assist.h"
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
/* The graphs show the last minute. */
#define HISTORY 240u
/* Snaps reach a share of the camera's distance, within bounds, metres. */
#define SNAP_REACH_SHARE 0.03
#define SNAP_REACH_MIN 2.5
#define SNAP_REACH_MAX 30.0
/* Lines the tools show are laid on the ground in steps about this long,
 * metres, at most so many points each. */
#define OVERLAY_STEP 4.0
#define OVERLAY_POINTS 64u
#define OVERLAY_LINES (FOX_ASSIST_MAX_LINES)

#define COLOR_SNAP_NETWORK UINT32_C(0xFFFF8A1A)
#define COLOR_SNAP_GUIDE UINT32_C(0xFF1A9AFF)
#define COLOR_SNAP_PLAIN UINT32_C(0xFFE8E8E8)

static const double speedPresets[] = {1.0, 3.0, 10.0};
static const char *const toolNames[FOX_TOOL_COUNT] = {
    [FOX_TOOL_NONE] = "look around",
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
    /* The arrivals and the tick at each sample, for a rate over the minute. */
    uint64_t arrivedAt[HISTORY];
    uint64_t tickAt[HISTORY];
    float frameHistory[HISTORY];
    float tickHistory[HISTORY];
    float movingHistory[HISTORY];
    float standingHistory[HISTORY];
    float speedHistory[HISTORY];
    float arrivalHistory[HISTORY];

    FoxUi *ui;
    FoxBuildSettings settings;
    /* Points of the road being placed: start, then for curves the bend. */
    FoxVec placed[2];
    FoxVec hit;
    /* Where the pointer snaps to, and what the tools show of it. */
    FoxAssist assist;
    FoxVec overlayPoints[OVERLAY_LINES][OVERLAY_POINTS];
    /* The preview shown for, to rebuild it only when it changes. */
    FoxVec previewEnd;
    FoxVec *previewPoints;
    FoxBuildSettings previewSettings;
    double groundCooldown;
    double reportTimer;
    uint32_t timeScale;
    uint32_t windowFrames;
    uint32_t placedCount;
    uint32_t previewPlaced;
    bool viewReady;
    bool picking;
    bool statistics;
    bool hasHit;
    bool hasAssist;
    /* The pointer is over the interface, or the button went down there. */
    bool overUi;
    bool pressOnUi;
    bool overlayShown;
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
    scene->statistics = false;
    if (scene->previewPoints == NULL || scene->statisticsText == NULL ||
        !FoxCityCreate(&scene->city, &cityDesc) ||
        !FoxClockInit(&scene->clock, FOX_CLOCK_TICKS_PER_SECOND,
                      FOX_CLOCK_DEFAULT_MAX_ADVANCE_SECONDS))
    {
        FoxSceneDestroy(scene);
        return NULL;
    }
    scene->resumeScale = 1.0;
    scene->settings = (FoxBuildSettings){
        .tool = desc->tool < FOX_TOOL_COUNT ? (FoxTool)desc->tool : FOX_TOOL_NONE,
        .roadType = FOX_ROAD_STREET,
        .snaps = FOX_SNAP_NETWORK | FOX_SNAP_ANGLE | FOX_SNAP_GUIDES | FOX_SNAP_TANGENT,
        .angleStep = 1u,
        .lengthStep = 2u,
        .sign = FOX_SIGN_PRIORITY,
        .speedChoice = 4u,
        .brush = 1u,
    };

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
        scene->ui = FoxUiCreate(desc->device);
        if (scene->ui == NULL)
        {
            FoxLog(FOX_LOG_WARNING, "the device has no interface layer; tools are on the keys");
        }
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
    FoxLog(FOX_LOG_INFO, "tools on the dock or the keys: 1 road, 2 curved road, 3 bulldoze, 4 "
                         "dig, 5 raise, 6 drive zone, 7 traffic lights, 8 junction signs, 9 "
                         "speed limits, Esc puts the tool down; T road type; V time speed, "
                         "Space pause; E/G more or fewer cars (Shift: ten times as many); F3 "
                         "statistics");
    return scene;
}

void FoxSceneDestroy(FoxScene *scene)
{
    if (scene == NULL)
    {
        return;
    }
    FoxUiDestroy(scene->ui);
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
    if (scene == NULL)
    {
        return false;
    }
    if (scene->placedCount != 0u)
    {
        scene->placedCount = 0u;
        return true;
    }
    if (scene->settings.tool != FOX_TOOL_NONE)
    {
        scene->settings.tool = FOX_TOOL_NONE;
        return true;
    }
    return false;
}

static void SetTimeScale(FoxScene *scene, uint32_t index)
{
    const uint32_t count = (uint32_t)(sizeof(speedPresets) / sizeof(speedPresets[0]));
    scene->timeScale = index % count;
    scene->resumeScale = speedPresets[scene->timeScale];
    (void)FoxClockSetTimeScale(&scene->clock, scene->resumeScale);
    FoxLog(FOX_LOG_INFO, "time runs at x%g", scene->resumeScale);
}

static void TogglePause(FoxScene *scene)
{
    const bool paused = scene->clock.timeScale == 0.0;
    (void)FoxClockSetTimeScale(&scene->clock, paused ? scene->resumeScale : 0.0);
    FoxLog(FOX_LOG_INFO, "%s", paused ? "time resumes" : "time is paused");
}

static void ApplyTimeControls(FoxScene *scene, const FoxControls *controls,
                              const FoxUiRequests *requests)
{
    if (controls->nextTimeScale)
    {
        SetTimeScale(scene, scene->timeScale + 1u);
    }
    if (requests->timeScale >= 0)
    {
        /* Choosing a speed also lets time run again. */
        if (scene->clock.timeScale == 0.0)
        {
            TogglePause(scene);
        }
        SetTimeScale(scene, (uint32_t)requests->timeScale);
    }
    if (controls->togglePause || requests->togglePause)
    {
        TogglePause(scene);
    }
    if (controls->toggleStatistics || requests->toggleStatistics)
    {
        scene->statistics = !scene->statistics;
    }
    if (controls->carsChange != 0)
    {
        const uint32_t target = FoxTrafficTarget(FoxWorldTraffic(scene->city.world));
        const uint32_t step = controls->moveFast ? CARS_STEP * 10u : CARS_STEP;
        const uint32_t next =
            controls->carsChange > 0 ? target + step : (target > step ? target - step : 0u);
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

static bool Building(const FoxScene *scene)
{
    return scene->settings.tool == FOX_TOOL_ROAD || scene->settings.tool == FOX_TOOL_CURVE;
}

/* How far snapping reaches at the camera's distance, metres. */
static double SnapReach(const FoxScene *scene)
{
    return fmin(fmax(scene->camera.distance * SNAP_REACH_SHARE, SNAP_REACH_MIN), SNAP_REACH_MAX);
}

/* A straight line between two points laid on the ground; the number of
 * points written. */
static uint32_t Drape(const FoxScene *scene, FoxVec from, FoxVec to, FoxVec *points)
{
    const FoxPlanet *planet = FoxWorldPlanet(scene->city.world);
    const double metres = (double)FoxVecDistance(from, to) / FOX_UNITS_PER_METRE;
    const uint32_t steps =
        (uint32_t)fmin(fmax(ceil(metres / OVERLAY_STEP), 1.0), (double)(OVERLAY_POINTS - 1u));
    uint32_t count = 0u;
    for (uint32_t step = 0u; step <= steps; ++step)
    {
        const FoxVec point = FoxVecLerp(from, to, (int64_t)step * FOX_UNIT / (int64_t)steps);
        count += FoxPlanetGround(planet, point, &points[count]) ? 1u : 0u;
    }
    return count;
}

static uint32_t SnapColor(FoxSnapKind kind)
{
    switch (kind)
    {
    case FOX_SNAPPED_NODE:
    case FOX_SNAPPED_ROAD:
        return COLOR_SNAP_NETWORK;
    case FOX_SNAPPED_GUIDE:
        return COLOR_SNAP_GUIDE;
    default:
        return COLOR_SNAP_PLAIN;
    }
}

/* Where the pointer snaps to while a road is drawn, shown on the ground: a
 * ring at the point and the lines it follows. */
static void UpdateAssist(FoxScene *scene)
{
    const FoxAssist before = scene->assist;
    const bool had = scene->hasAssist;
    scene->hasAssist = false;
    if (Building(scene) && scene->hasHit && !scene->overUi)
    {
        const FoxBuildSettings *settings = &scene->settings;
        const bool started = scene->placedCount != 0u;
        const FoxAssistRequest request = {
            .raw = scene->hit,
            .hasStart = started,
            .start = started ? scene->placed[scene->placedCount - 1u] : scene->hit,
            .curveControl = settings->tool == FOX_TOOL_CURVE && scene->placedCount == 1u,
            .snaps = settings->snaps,
            .angleStep = FoxSnapAngleDegrees(settings->angleStep),
            .lengthStep = FoxSnapLengthMetres(settings->lengthStep),
            .reach = SnapReach(scene),
        };
        scene->hasAssist =
            FoxAssistPoint(FoxWorldNetwork(scene->city.world), FoxWorldPlanet(scene->city.world),
                           &request, &scene->assist);
    }
    if (!scene->hasAssist)
    {
        if (scene->overlayShown)
        {
            FoxRoadsViewSetOverlay(scene->roads, NULL, 0u, NULL, 0u);
            scene->overlayShown = false;
        }
        return;
    }
    /* Rebuilt only when the snap moves. */
    if (had && scene->overlayShown && FoxVecEqual(before.point, scene->assist.point) &&
        before.kind == scene->assist.kind && before.lineCount == scene->assist.lineCount)
    {
        return;
    }
    FoxOverlayLine lines[OVERLAY_LINES];
    uint32_t lineCount = 0u;
    for (uint32_t index = 0u; index < scene->assist.lineCount; ++index)
    {
        const FoxAssistLine *line = &scene->assist.lines[index];
        const uint32_t count = Drape(scene, line->from, line->to, scene->overlayPoints[index]);
        if (count >= 2u)
        {
            lines[lineCount++] = (FoxOverlayLine){
                .points = scene->overlayPoints[index],
                .count = count,
                .dashed = true,
                .color = line->guide ? COLOR_SNAP_GUIDE : COLOR_SNAP_PLAIN,
            };
        }
    }
    const FoxOverlayRing ring = {
        .centre = scene->assist.point,
        .radius = (double)FoxRoadHalfWidth(scene->settings.roadType) / FOX_UNITS_PER_METRE + 0.6,
        .color = SnapColor(scene->assist.kind),
    };
    FoxRoadsViewSetOverlay(scene->roads, lines, lineCount, &ring,
                           scene->assist.kind != FOX_SNAPPED_FREE ? 1u : 0u);
    scene->overlayShown = true;
}

static FoxRoadPlan PlanTo(const FoxScene *scene, FoxVec end)
{
    FoxRoadPlan plan = {
        .start = scene->placed[0],
        .end = end,
        .type = scene->settings.roadType,
    };
    if (scene->settings.tool == FOX_TOOL_CURVE && scene->placedCount == 2u)
    {
        plan.curved = true;
        plan.control = scene->placed[1];
    }
    return plan;
}

/* The road as it would be built, in the accent colour, or red where the
 * network would refuse it; its length and angle beside the pointer. */
static void UpdatePreview(FoxScene *scene)
{
    const bool placing = Building(scene) && scene->placedCount != 0u && scene->hasAssist;
    if (!placing)
    {
        if (scene->previewPlaced != 0u)
        {
            FoxRoadsViewSetPreview(scene->roads, NULL, 0u, scene->settings.roadType, false);
            scene->previewPlaced = 0u;
        }
        FoxUiSetReadout(scene->ui, false, 0.0, 0.0);
        return;
    }
    const FoxVec end = scene->assist.point;
    if (scene->previewPlaced == scene->placedCount && FoxVecEqual(end, scene->previewEnd))
    {
        return;
    }
    scene->previewPlaced = scene->placedCount;
    scene->previewEnd = end;
    const FoxRoadPlan plan = PlanTo(scene, end);
    const FoxPlanet *planet = FoxWorldPlanet(scene->city.world);
    const uint32_t count =
        FoxNetworkPlanCentre(planet, &plan, scene->previewPoints, FOX_ROAD_MAX_POINTS);
    int64_t length = 0;
    for (uint32_t index = 1u; index < count; ++index)
    {
        length += FoxVecDistance(scene->previewPoints[index - 1u], scene->previewPoints[index]);
    }
    const bool valid =
        FoxNetworkCheckRoad(FoxWorldNetwork(scene->city.world), planet, &plan) == FOX_ROAD_OK;
    FoxRoadsViewSetPreview(scene->roads, scene->previewPoints, count, scene->settings.roadType,
                           valid);
    FoxUiSetReadout(scene->ui, true, (double)length / FOX_UNITS_PER_METRE, scene->assist.degrees);
}

static void PlaceRoad(FoxScene *scene)
{
    if (!scene->hasAssist)
    {
        return;
    }
    const FoxVec point = scene->assist.point;
    const uint32_t needed = scene->settings.tool == FOX_TOOL_CURVE ? 2u : 1u;
    scene->previewPlaced = 0u;
    if (scene->placedCount < needed)
    {
        scene->placed[scene->placedCount++] = point;
        return;
    }
    const FoxRoadPlan plan = PlanTo(scene, point);
    const FoxRoadResult result = FoxWorldBuildRoad(scene->city.world, &plan);
    FoxLog(result == FOX_ROAD_OK ? FOX_LOG_INFO : FOX_LOG_WARNING, "%s: %s",
           FoxRoadTypeGet(scene->settings.roadType)->name, roadResults[result]);
    if (result == FOX_ROAD_OK)
    {
        /* Keep drawing from the end, as city builders do. */
        scene->placed[0] = point;
        scene->placedCount = 1u;
    }
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
    /* The sphere grows with the view, so the hand shows at any zoom, and
     * with the brush chosen. */
    const double metres =
        fmin(fmax(scene->camera.distance / 50.0, 2.0), 8.0) * FoxBrushScale(scene->settings.brush);
    const int64_t radius = (int64_t)(metres * FOX_UNITS_PER_METRE);
    FoxPlanetBox changed;
    const uint32_t samples = scene->settings.tool == FOX_TOOL_DIG
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
        FoxLog(FOX_LOG_INFO, "this district now keeps %s",
               side == FOX_DRIVE_RIGHT ? "right" : "left");
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

static void PlaceSign(FoxScene *scene)
{
    uint32_t end = 0u;
    FoxRoadView road;
    const uint32_t index = PickRoad(scene, &end, &road);
    const FoxSign sign = scene->settings.sign;
    if (index != FOX_NETWORK_NONE && FoxWorldSetSign(scene->city.world, index, end, sign) &&
        FoxNetworkRoad(FoxWorldNetwork(scene->city.world), index, &road))
    {
        FoxLog(FOX_LOG_INFO, "sign: %s, in force: %s", signNames[sign], signNames[road.sign[end]]);
    }
}

static void PlaceSpeedLimit(FoxScene *scene)
{
    uint32_t end = 0u;
    FoxRoadView road;
    const uint32_t index = PickRoad(scene, &end, &road);
    const int32_t kmh = FoxSpeedChoiceKmh(scene->settings.speedChoice);
    if (index != FOX_NETWORK_NONE &&
        FoxWorldSetSpeedLimit(scene->city.world, index, kmh == 0 ? 0 : FOX_KMH(kmh)) &&
        FoxNetworkRoad(FoxWorldNetwork(scene->city.world), index, &road))
    {
        FoxLog(FOX_LOG_INFO, "speed limit %ld km/h%s",
               (long)lround((double)road.speedLimit * 3.6 / FOX_UNITS_PER_METRE),
               road.speedSet ? "" : " (the road's own)");
    }
}

static void ApplyTools(FoxScene *scene, const FoxControls *controls, double realSeconds)
{
    FoxBuildSettings *settings = &scene->settings;
    if (controls->tool >= 0 && controls->tool < FOX_TOOL_COUNT)
    {
        settings->tool = (FoxTool)controls->tool;
    }
    if (controls->nextRoadType)
    {
        settings->roadType = (FoxRoadType)((settings->roadType + 1) % FOX_ROAD_TYPE_COUNT);
    }
    /* Whatever changed the tool, the dock or a key, starts it afresh. */
    if (settings->tool != scene->previewSettings.tool)
    {
        scene->placedCount = 0u;
        FoxLog(FOX_LOG_INFO, "tool: %s", toolNames[settings->tool]);
    }
    if (settings->roadType != scene->previewSettings.roadType)
    {
        FoxLog(FOX_LOG_INFO, "road type: %s", FoxRoadTypeGet(settings->roadType)->name);
        /* The preview is stale, whatever it showed. */
        scene->previewPlaced = UINT32_MAX;
    }
    scene->previewSettings = *settings;
    UpdateAssist(scene);
    UpdatePreview(scene);
    if (!scene->hasHit || scene->overUi)
    {
        return;
    }
    switch (settings->tool)
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
        if (controls->held && !scene->pressOnUi)
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
            PlaceSign(scene);
        }
        break;
    case FOX_TOOL_SPEED_LIMIT:
        if (controls->click)
        {
            PlaceSpeedLimit(scene);
        }
        break;
    case FOX_TOOL_NONE:
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
        *used += (size_t)written < STATISTICS_TEXT - *used ? (size_t)written
                                                           : STATISTICS_TEXT - *used - 1u;
    }
}

/* Shifts a history left by one and puts the newest value at the end. */
static void Push(float *history, float value)
{
    memmove(history, history + 1, (HISTORY - 1u) * sizeof(*history));
    history[HISTORY - 1u] = value;
}

static float Highest(const float *history, const float *stacked, float floor)
{
    float highest = floor;
    for (uint32_t index = 0u; index < HISTORY; ++index)
    {
        highest = fmaxf(highest, history[index] + (stacked != NULL ? stacked[index] : 0.0f));
    }
    return highest;
}

static const char *const toolNamesRu[FOX_TOOL_COUNT] = {
    [FOX_TOOL_NONE] = "осмотр",
    [FOX_TOOL_ROAD] = "дорога",
    [FOX_TOOL_CURVE] = "кривая дорога",
    [FOX_TOOL_BULLDOZE] = "снос",
    [FOX_TOOL_DIG] = "копать",
    [FOX_TOOL_RAISE] = "насыпать",
    [FOX_TOOL_DRIVE_ZONE] = "зона движения",
    [FOX_TOOL_SIGNALS] = "светофоры",
    [FOX_TOOL_SIGNS] = "знаки",
    [FOX_TOOL_SPEED_LIMIT] = "ограничения скорости",
};
static const char *const roadNamesRu[FOX_ROAD_TYPE_COUNT] = {
    [FOX_ROAD_STREET] = "улица",      [FOX_ROAD_AVENUE] = "проспект",
    [FOX_ROAD_BOULEVARD] = "бульвар", [FOX_ROAD_ONE_WAY] = "односторонняя",
    [FOX_ROAD_HIGHWAY] = "шоссе",
};

/* Everything about the city and the frame, for the panel, and the graphs of
 * the last minute. */
static void BuildStatistics(FoxScene *scene, int32_t verticalSync)
{
    const FoxWorld *world = scene->city.world;
    const FoxNetwork *network = FoxWorldNetwork(world);
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const FoxNetworkStats roads = FoxNetworkGetStats(network);
    const FoxTrafficStats cars = FoxTrafficGetStats(traffic);
    const uint64_t tick = FoxWorldTick(world);
    const uint64_t seconds = tick / FOX_TRAFFIC_TICKS_PER_SECOND;
    const double frames = (double)scene->windowFrames;
    const double fps = scene->windowSeconds > 0.0 ? frames / scene->windowSeconds : 0.0;
    const double frameMs = frames > 0.0 ? scene->windowSeconds * 1000.0 / frames : 0.0;
    const double tickMs = scene->windowTicks != 0u
                              ? scene->windowTickSeconds * 1000.0 / (double)scene->windowTicks
                              : 0.0;
    const double minutes = (double)tick / FOX_TRAFFIC_TICKS_PER_SECOND / 60.0;
    /* Arrivals per city minute over the minute the graphs show. */
    memmove(scene->arrivedAt, scene->arrivedAt + 1, (HISTORY - 1u) * sizeof(*scene->arrivedAt));
    memmove(scene->tickAt, scene->tickAt + 1, (HISTORY - 1u) * sizeof(*scene->tickAt));
    scene->arrivedAt[HISTORY - 1u] = cars.arrived;
    scene->tickAt[HISTORY - 1u] = tick;
    uint32_t oldest = 0u;
    while (oldest + 1u < HISTORY && scene->tickAt[oldest] == 0u)
    {
        ++oldest;
    }
    const double spanMinutes =
        (double)(tick - scene->tickAt[oldest]) / FOX_TRAFFIC_TICKS_PER_SECOND / 60.0;
    const double arrivalRate =
        spanMinutes > 0.0 ? (double)(cars.arrived - scene->arrivedAt[oldest]) / spanMinutes : 0.0;

    Push(scene->frameHistory, (float)frameMs);
    Push(scene->tickHistory, (float)tickMs);
    Push(scene->movingHistory, (float)(cars.vehicles - cars.standing));
    Push(scene->standingHistory, (float)cars.standing);
    Push(scene->speedHistory, (float)Kmh(cars.meanSpeed));
    Push(scene->arrivalHistory, (float)arrivalRate);

    char *text = scene->statisticsText;
    size_t used = 0u;
    text[0] = '\0';
    Append(text, &used, "-- FOXITY: СТАТИСТИКА (F3 скрыть)\n");
    Append(text, &used, "Кадр: %.0f к/с, %.1f мс, худший %.1f мс\n", fps, frameMs,
           scene->windowSlowestFrame * 1000.0);
    Append(text, &used, "Вертикальная синхронизация (F7): %s\n",
           verticalSync < 0 ? "недоступна" : (verticalSync != 0 ? "вкл" : "выкл"));
    Append(text, &used, "Тик: %.3f мс, худший %.3f мс, %.0f тиков/с\n", tickMs,
           scene->windowSlowestTick * 1000.0,
           scene->windowSeconds > 0.0 ? (double)scene->windowTicks / scene->windowSeconds : 0.0);
    Append(text, &used, "Тик занимает %.2f %% своих 33,3 мс\n",
           tickMs / (1000.0 / FOX_TRAFFIC_TICKS_PER_SECOND) * 100.0);
    Append(text, &used, "Время: x%g%s, в городе %02u:%02u:%02u\n", scene->resumeScale,
           scene->clock.timeScale == 0.0 ? ", ПАУЗА" : "", (unsigned)(seconds / 3600u),
           (unsigned)(seconds / 60u % 60u), (unsigned)(seconds % 60u));
    Append(text, &used, "Инструмент: %s, тип: %s\n", toolNamesRu[scene->settings.tool],
           roadNamesRu[scene->settings.roadType]);
    Append(text, &used, "-- ДОРОГИ\n");
    Append(text, &used, "Дорог %u, %.2f км; полос %u, %.2f км\n", roads.roads,
           (double)roads.roadLength / FOX_METRES(1000), roads.lanes,
           (double)roads.laneLength / FOX_METRES(1000));
    Append(text, &used, "Улиц %u, проспектов %u, бульваров %u\n",
           roads.roadsByType[FOX_ROAD_STREET], roads.roadsByType[FOX_ROAD_AVENUE],
           roads.roadsByType[FOX_ROAD_BOULEVARD]);
    Append(text, &used, "Односторонних %u, шоссе %u\n", roads.roadsByType[FOX_ROAD_ONE_WAY],
           roads.roadsByType[FOX_ROAD_HIGHWAY]);
    Append(text, &used, "Ограничений скорости вручную: %u\n", roads.speedLimitsSet);
    Append(text, &used, "Узлов %u: перекрёстков %u, изгибов %u, тупиков %u\n", roads.nodes,
           roads.junctions, roads.bends, roads.deadEnds);
    Append(text, &used, "Светофоров: %u\n", roads.signalled);
    Append(text, &used, "Знаки: главная %u, уступи %u, стоп %u, нет %u\n",
           roads.signs[FOX_SIGN_PRIORITY], roads.signs[FOX_SIGN_YIELD], roads.signs[FOX_SIGN_STOP],
           roads.signs[FOX_SIGN_NONE]);
    Append(text, &used, "Поворотов %u, пересечений между ними %u\n", roads.connections,
           roads.conflicts);
    Append(text, &used, "-- ТРАНСПОРТ\n");
    Append(text, &used, "Машин %u из %u: легковых %u, грузовиков %u\n", cars.vehicles,
           FoxTrafficTarget(traffic), cars.vehicles - cars.trucks, cars.trucks);
    Append(text, &used, "Едут %u, стоят %u (%.0f %%), дольше 30 с: %u\n",
           cars.vehicles - cars.standing, cars.standing,
           cars.vehicles != 0u ? 100.0 * cars.standing / cars.vehicles : 0.0, cars.waitingLong);
    Append(text, &used, "Скорость: средняя %.1f, наибольшая %.1f км/ч\n", Kmh(cars.meanSpeed),
           Kmh(cars.maxSpeed));
    Append(text, &used, "Перестроений %llu, обгонов по встречной %llu\n",
           (unsigned long long)cars.laneChanges, (unsigned long long)cars.overtakes);
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
        /* A bar of up to twenty-four cells, scaled to all vehicles. */
        const uint32_t cells =
            cars.vehicles != 0u ? (uint32_t)((uint64_t)bands[band] * 24u / cars.vehicles) : 0u;
        char bar[32];
        memset(bar, '#', cells);
        bar[cells] = '\0';
        if (band < 7u)
        {
            Append(text, &used, "%3u-%-3u км/ч %-24s %u\n", band * 10u, band * 10u + 10u, bar,
                   bands[band]);
        }
        else
        {
            Append(text, &used, "%3u+    км/ч %-24s %u\n", band * 10u, bar, bands[band]);
        }
    }
    Append(text, &used, "Появилось %llu, доехало %llu (%.0f в мин.)\n",
           (unsigned long long)cars.spawned, (unsigned long long)cars.arrived,
           minutes > 0.0 ? (double)cars.arrived / minutes : 0.0);
    Append(text, &used, "Стоят полминуты и дольше %u\n", cars.waitingLong);
    Append(text, &used, "-- КЛАВИШИ\n");
    Append(text, &used, "1-9 инструменты, ESC убрать, T тип дороги\n");
    Append(text, &used, "V скорость времени\n");
    Append(text, &used, "Пробел пауза, E/G ±50 машин, с Shift ×10\n");
    Append(text, &used, "F3 панель, F7 синхронизация");

    char titles[5][96];
    (void)snprintf(titles[0], sizeof(titles[0]), "Кадр, мс: %.1f", frameMs);
    (void)snprintf(titles[1], sizeof(titles[1]), "Тик, мс: %.3f", tickMs);
    (void)snprintf(titles[2], sizeof(titles[2]), "Едут (зел.) и стоят (красн.): %u / %u",
                   cars.vehicles - cars.standing, cars.standing);
    (void)snprintf(titles[3], sizeof(titles[3]), "Средняя скорость, км/ч: %.1f",
                   Kmh(cars.meanSpeed));
    (void)snprintf(titles[4], sizeof(titles[4]), "Доезжают в минуту: %.0f", arrivalRate);
    const FoxHudGraph graphs[] = {
        {titles[0], scene->frameHistory, NULL, HISTORY, Highest(scene->frameHistory, NULL, 33.4f),
         UINT32_C(0xFF50C8F0), 0u},
        {titles[1], scene->tickHistory, NULL, HISTORY, Highest(scene->tickHistory, NULL, 1.0f),
         UINT32_C(0xFFE0A050), 0u},
        {titles[2], scene->movingHistory, scene->standingHistory, HISTORY,
         Highest(scene->movingHistory, scene->standingHistory, 10.0f), UINT32_C(0xFF50D050),
         UINT32_C(0xFF4040E0)},
        {titles[3], scene->speedHistory, NULL, HISTORY, Highest(scene->speedHistory, NULL, 10.0f),
         UINT32_C(0xFFF0F0F0), 0u},
        {titles[4], scene->arrivalHistory, NULL, HISTORY,
         Highest(scene->arrivalHistory, NULL, 10.0f), UINT32_C(0xFFD080D0), 0u},
    };
    FoxHudSetContent(scene->hud, text, graphs, (uint32_t)(sizeof(graphs) / sizeof(graphs[0])));
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
    /* The interface takes the pointer first; what it covers the world does
     * not see. */
    FoxUiRequests requests = {.timeScale = -1};
    const FoxUiInput input = {
        .width = controls->viewWidth,
        .height = controls->viewHeight,
        .pointer = controls->pointer,
        .x = (controls->pointerX + 1.0f) / 2.0f * (float)controls->viewWidth,
        .y = (1.0f - controls->pointerY) / 2.0f * (float)controls->viewHeight,
        .click = controls->click,
    };
    const FoxUiStatus status = {
        .paused = scene->clock.timeScale == 0.0,
        .timeScale = scene->timeScale,
        .statistics = scene->statistics,
    };
    scene->overUi = FoxUiUpdate(scene->ui, &input, &status, &scene->settings, &requests);
    if (controls->click)
    {
        scene->pressOnUi = scene->overUi;
    }
    else if (!controls->held)
    {
        scene->pressOnUi = false;
    }
    ApplyTimeControls(scene, controls, &requests);
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
        BuildStatistics(scene, controls->verticalSync);
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
    FoxUiDraw(scene->ui);
}
