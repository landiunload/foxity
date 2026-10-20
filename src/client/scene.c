#include "client/scene.h"

#include "app/city.h"
#include "app/log.h"
#include "client/camera.h"
#include "client/cars_view.h"
#include "client/planet_view.h"
#include "client/roads_view.h"
#include "sim/clock.h"

#include <math.h>
#include <stddef.h>
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

static const double speedPresets[] = {1.0, 3.0, 10.0};
static const char *const toolNames[FOX_TOOL_COUNT] = {
    [FOX_TOOL_ROAD] = "road",
    [FOX_TOOL_CURVE] = "curved road",
    [FOX_TOOL_BULLDOZE] = "bulldoze",
    [FOX_TOOL_DIG] = "dig",
    [FOX_TOOL_RAISE] = "raise ground",
    [FOX_TOOL_DRIVE_ZONE] = "drive-side zone",
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
    double resumeScale;
    FoxCamera camera;
    FoxCameraView view;
    bool viewReady;
    const LaiueSceneMathServiceV1 *math;
    bool picking;
    FoxPlanetView *planet;
    FoxRoadsView *roads;
    FoxCarsView *cars;

    FoxTool tool;
    FoxRoadType roadType;
    /* Points of the road being placed: start, then for curves the bend. */
    FoxVec placed[2];
    uint32_t placedCount;
    bool hasHit;
    FoxVec hit;
    double groundCooldown;
    double reportTimer;
    /* The preview shown for, to rebuild it only when it changes. */
    FoxVec previewEnd;
    uint32_t previewPlaced;
    FoxVec *previewPoints;
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
        .modules = desc->modules,
    };
    if (scene->previewPoints == NULL || !FoxCityCreate(&scene->city, &cityDesc) ||
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
        if (scene->roads == NULL || scene->cars == NULL)
        {
            FoxLog(FOX_LOG_WARNING, "roads or cars cannot be drawn");
        }
    }
    const FoxPlanet *planet = FoxWorldPlanet(scene->city.world);
    FoxVec focus = {0, 0, FoxPlanetRadius(planet)};
    (void)FoxPlanetGround(planet, focus, &focus);
    FoxCameraInit(&scene->camera, focus);
    if (desc->cameraDistance > 0.0)
    {
        scene->camera.distance = desc->cameraDistance;
        FoxCameraZoom(&scene->camera, 0.0f);
    }
    FoxCityReport(&scene->city, "city founded");
    FoxLog(FOX_LOG_INFO, "tools: 1 road, 2 curved road, 3 bulldoze, 4 dig, 5 raise, 6 drive "
                         "zone; T road type; 7/8/9 speed, Space pause; E/G more or fewer cars");
    return scene;
}

void FoxSceneDestroy(FoxScene *scene)
{
    if (scene == NULL)
    {
        return;
    }
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
    if (controls->speedPreset >= 0 &&
        (size_t)controls->speedPreset < sizeof(speedPresets) / sizeof(speedPresets[0]))
    {
        scene->resumeScale = speedPresets[controls->speedPreset];
        (void)FoxClockSetTimeScale(&scene->clock, scene->resumeScale);
        FoxLog(FOX_LOG_INFO, "time runs at x%g", scene->resumeScale);
    }
    if (controls->togglePause)
    {
        const bool paused = scene->clock.timeScale == 0.0;
        (void)FoxClockSetTimeScale(&scene->clock, paused ? scene->resumeScale : 0.0);
        FoxLog(FOX_LOG_INFO, "%s", paused ? "time resumes" : "time is paused");
    }
    if (controls->carsChange != 0)
    {
        const uint32_t target = FoxTrafficTarget(FoxWorldTraffic(scene->city.world));
        const uint32_t next = controls->carsChange > 0 ? target + CARS_STEP
                                                       : (target > CARS_STEP ? target - CARS_STEP : 0u);
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
    case FOX_TOOL_COUNT:
        break;
    }
}

bool FoxSceneUpdate(FoxScene *scene, const FoxControls *controls, double realSeconds,
                    float aspectRatio, float outViewProjection[16])
{
    if (scene == NULL || controls == NULL)
    {
        return false;
    }
    ApplyTimeControls(scene, controls);
    FoxWorldAdvance(scene->city.world, FoxClockAdvance(&scene->clock, realSeconds));
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
    FoxRoadsViewDraw(scene->roads, &scene->view);
    (void)FoxCarsViewDraw(scene->cars, FoxWorldTraffic(scene->city.world),
                          FoxWorldNetwork(scene->city.world), &scene->view,
                          FoxClockInterpolation(&scene->clock));
}
