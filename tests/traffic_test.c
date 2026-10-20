#include "expect.h"
#include "sim/world.h"
#include "test_router.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define SECONDS(seconds) ((uint64_t)(seconds) * FOX_TRAFFIC_TICKS_PER_SECOND)

static TestRouter routerA;
static TestRouter routerB;

static FoxWorld *Town(FoxDriveSide side, const FoxRouter *router, uint32_t cars)
{
    const FoxWorldDesc desc = {
        .seed = 7u, .driveSide = side, .targetVehicles = cars, .router = router};
    FoxWorld *world = FoxWorldCreate(&desc);
    EXPECT(world != NULL, "the world is created");
    EXPECT(FoxWorldBuildDemoTown(world), "the demo town is built");
    return world;
}

/* No two cars on one path overlap. */
static void ExpectNoOverlap(const FoxWorld *world)
{
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const uint32_t capacity = FoxTrafficCapacity(traffic);
    for (uint32_t a = 0u; a < capacity; ++a)
    {
        FoxVehicleView first;
        if (!FoxTrafficVehicle(traffic, a, &first))
        {
            continue;
        }
        for (uint32_t b = a + 1u; b < capacity; ++b)
        {
            FoxVehicleView second;
            if (FoxTrafficVehicle(traffic, b, &second) && second.path == first.path)
            {
                EXPECT(FoxAbs(second.distance - first.distance) >= FOX_VEHICLE_LENGTH,
                       "cars on a path keep apart");
            }
        }
    }
}

static void Report(const char *label, const FoxWorld *world)
{
    const FoxTrafficStats stats = FoxTrafficGetStats(FoxWorldTraffic(world));
    printf("%s: %u cars, %u standing, %llu spawned, %llu arrived, %llu removed (%llu stuck), "
           "mean %.1f m/s\n",
           label, stats.vehicles, stats.standing, (unsigned long long)stats.spawned,
           (unsigned long long)stats.arrived, (unsigned long long)stats.removed,
           (unsigned long long)stats.stuck,
           (double)stats.meanSpeed / FOX_UNITS_PER_METRE);
}

static void ExpectFlowing(const FoxWorld *world, const char *label)
{
    const FoxTrafficStats stats = FoxTrafficGetStats(FoxWorldTraffic(world));
    Report(label, world);
    EXPECT(stats.vehicles > 0u, "cars are on the road");
    EXPECT(stats.standing * 10u < stats.vehicles * 8u, "the town does not lock up");
    EXPECT(stats.meanSpeed > FOX_METRES(2), "traffic moves");
}

static void TestRandomTurns(FoxDriveSide side)
{
    FoxWorld *world = Town(side, NULL, 160u);
    for (int minute = 0; minute < 4; ++minute)
    {
        FoxWorldAdvance(world, SECONDS(60));
        ExpectNoOverlap(world);
    }
    ExpectFlowing(world, side == FOX_DRIVE_RIGHT ? "keep right, random turns"
                                                : "keep left, random turns");
    FoxWorldDestroy(world);
}

static void TestRouted(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, 200u);
    for (int minute = 0; minute < 4; ++minute)
    {
        FoxWorldAdvance(world, SECONDS(60));
        ExpectNoOverlap(world);
    }
    ExpectFlowing(world, "routed");
    EXPECT(routerA.calls > 0u, "cars ask for routes");
    EXPECT(FoxTrafficGetStats(FoxWorldTraffic(world)).arrived > 20u, "cars arrive");
    FoxWorldDestroy(world);
}

static void TestDeterminism(void)
{
    TestRouterInit(&routerA);
    TestRouterInit(&routerB);
    FoxWorld *first = Town(FOX_DRIVE_RIGHT, &routerA.router, 120u);
    FoxWorld *second = Town(FOX_DRIVE_RIGHT, &routerB.router, 120u);
    for (int step = 0; step < 6; ++step)
    {
        FoxWorldAdvance(first, SECONDS(10));
        FoxWorldAdvance(second, SECONDS(10));
        EXPECT(FoxWorldHash(first) == FoxWorldHash(second), "the same commands give the same city");
    }
    FoxWorldDestroy(first);
    FoxWorldDestroy(second);
}

static void TestEditsUnderTraffic(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, 200u);
    FoxWorldAdvance(world, SECONDS(60));
    const int64_t radius = FoxPlanetRadius(FoxWorldPlanet(world));
    /* A diagonal crossing several roads while cars drive on them. */
    const FoxRoadPlan diagonal = {
        .start = {FOX_METRES(-190), FOX_METRES(-130), radius},
        .end = {FOX_METRES(130), FOX_METRES(190), radius},
        .type = FOX_ROAD_STREET,
    };
    EXPECT(FoxWorldBuildRoad(world, &diagonal) == FOX_ROAD_OK, "a diagonal through the town");
    ExpectNoOverlap(world);
    FoxWorldAdvance(world, SECONDS(30));
    ExpectNoOverlap(world);
    /* Take out a road in the middle and turn half the town to keep left. */
    const FoxNetwork *network = FoxWorldNetwork(world);
    const uint32_t road =
        FoxNetworkNearestRoad(network, (FoxVec){FOX_METRES(40), 0, radius}, FOX_METRES(5), NULL);
    EXPECT(road != FOX_NETWORK_NONE && FoxWorldRemoveRoad(world, road), "a road is removed");
    EXPECT(FoxWorldAddDriveZone(world, (FoxVec){FOX_METRES(-120), FOX_METRES(-120), radius},
                                FOX_METRES(130), FOX_DRIVE_LEFT),
           "a keep-left district");
    ExpectNoOverlap(world);
    for (int minute = 0; minute < 3; ++minute)
    {
        FoxWorldAdvance(world, SECONDS(60));
        ExpectNoOverlap(world);
    }
    ExpectFlowing(world, "after edits");
    FoxWorldDestroy(world);
}

int main(void)
{
    TestRandomTurns(FOX_DRIVE_RIGHT);
    TestRandomTurns(FOX_DRIVE_LEFT);
    TestRouted();
    TestDeterminism();
    TestEditsUnderTraffic();
    return 0;
}
