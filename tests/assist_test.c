#include "client/assist.h"
#include "expect.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>

/* Drawing assistance over a small network near the pole: an east-west
 * street from -50 m to 50 m and a north-south one at 100 m east. */

#define RADIUS FOX_PLANET_DEFAULT_RADIUS
#define CLOSE 0.3

static FoxVec Ground(const FoxPlanet *planet, double east, double north)
{
    FoxVec point = {llround(east * FOX_UNITS_PER_METRE), llround(north * FOX_UNITS_PER_METRE),
                    RADIUS};
    EXPECT(FoxPlanetGround(planet, point, &point), "there is ground near the pole");
    return point;
}

/* Whether a point lies on the ground close to where east and north from the
 * pole meet it. */
static bool Near(const FoxPlanet *planet, FoxVec point, double east, double north)
{
    return (double)FoxVecDistance(point, Ground(planet, east, north)) < CLOSE * FOX_UNITS_PER_METRE;
}

static FoxAssist Snap(const FoxNetwork *network, const FoxPlanet *planet, FoxAssistRequest request)
{
    FoxAssist assist;
    EXPECT(FoxAssistPoint(network, planet, &request, &assist), "the point lies on the ground");
    return assist;
}

int main(void)
{
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    const FoxNetworkDesc desc = {.driveSide = FOX_DRIVE_RIGHT};
    FoxNetwork *network = FoxNetworkCreate(&desc);
    EXPECT(planet != NULL && network != NULL, "the planet and the network are created");
    FoxRoadPlan plan = {.start = Ground(planet, -50.0, 0.0),
                        .end = Ground(planet, 50.0, 0.0),
                        .type = FOX_ROAD_STREET};
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the street is built");
    plan.start = Ground(planet, 100.0, -50.0);
    plan.end = Ground(planet, 100.0, 50.0);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OK,
           "the second street could be built");
    EXPECT(FoxNetworkRoadCount(network) == 1u, "checking builds nothing");
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the second street");
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) != FOX_ROAD_OK,
           "the same street again cannot be built on it");

    const FoxVec end = Ground(planet, 50.0, 0.0);
    const FoxAssistRequest base = {.reach = 5.0, .angleStep = 15.0, .lengthStep = 8.0};

    FoxAssistRequest request = base;
    request.raw = Ground(planet, 48.5, 2.0);
    request.snaps = FOX_SNAP_NETWORK;
    FoxAssist assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_NODE && Near(planet, assist.point, 50.0, 0.0),
           "a point near a road's end snaps onto its node");

    request = base;
    request.raw = Ground(planet, 80.0, 3.0);
    request.hasStart = true;
    request.start = end;
    request.snaps = FOX_SNAP_ANGLE;
    assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_ANGLE && Near(planet, assist.point, 80.0, 0.0) &&
               assist.degrees < 1.0,
           "a road drawn on from a dead end snaps straight on");
    request.raw = Ground(planet, 52.0, 30.0);
    assist = Snap(network, planet, request);
    EXPECT(Near(planet, assist.point, 50.0, 30.0) && fabs(assist.degrees - 90.0) < 1.0,
           "or square to the road it continues");

    request.raw = Ground(planet, 80.3, 0.5);
    request.snaps = FOX_SNAP_ANGLE | FOX_SNAP_LENGTH;
    assist = Snap(network, planet, request);
    EXPECT(Near(planet, assist.point, 82.0, 0.0) && fabs(assist.metres - 32.0) < CLOSE,
           "the length rounds to whole steps");

    request.raw = Ground(planet, 97.0, 1.0);
    request.snaps = FOX_SNAP_ANGLE | FOX_SNAP_NETWORK;
    assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_ROAD && Near(planet, assist.point, 100.0, 0.0),
           "along the angle the road ahead is met where it crosses");

    request = base;
    request.raw = Ground(planet, 13.2, 22.9);
    request.snaps = FOX_SNAP_GRID;
    assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_GRID && Near(planet, assist.point, 16.0, 24.0),
           "the grid catches the nearest corner");

    request.raw = Ground(planet, 130.0, 2.0);
    request.snaps = FOX_SNAP_GUIDES;
    assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_GUIDE && Near(planet, assist.point, 130.0, 0.0) &&
               assist.lineCount != 0u,
           "a guide carries a road's line on past its end");
    request.raw = Ground(planet, 101.5, 31.0);
    assist = Snap(network, planet, request);
    EXPECT(assist.kind == FOX_SNAPPED_GUIDE && Near(planet, assist.point, 100.0, 31.0),
           "and lies along the road itself");

    request = base;
    request.raw = Ground(planet, 70.0, 10.0);
    request.hasStart = true;
    request.start = end;
    request.curveControl = true;
    request.snaps = FOX_SNAP_TANGENT;
    assist = Snap(network, planet, request);
    EXPECT(Near(planet, assist.point, 70.0, 0.0), "a curve leaves a dead end the way the road went");

    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
    FoxPlanetDestroy(planet);
    puts("assist: ok");
    return 0;
}
