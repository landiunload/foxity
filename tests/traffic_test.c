#include "expect.h"
#include "sim/network_internal.h"
#include "sim/world.h"
#include "test_router.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define SECONDS(seconds) ((uint64_t)(seconds) * FOX_TRAFFIC_TICKS_PER_SECOND)

/* The town as loaded as the game starts it, with every driver giving way
 * as the rules ask. */
#define LOAD 150u

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
                const FoxVehicleView *front = second.distance > first.distance ? &second : &first;
                EXPECT(FoxAbs(second.distance - first.distance) >= front->length,
                       "vehicles on a path keep apart");
            }
        }
    }
}

static void Report(const char *label, const FoxWorld *world)
{
    const FoxTrafficStats stats = FoxTrafficGetStats(FoxWorldTraffic(world));
    printf("%s: %u cars, %u standing (%u half a minute), %llu spawned, %llu arrived, %llu "
           "removed, mean %.1f m/s\n",
           label, stats.vehicles, stats.standing, stats.waitingLong,
           (unsigned long long)stats.spawned, (unsigned long long)stats.arrived,
           (unsigned long long)stats.removed, (double)stats.meanSpeed / FOX_UNITS_PER_METRE);
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
    ExpectFlowing(world,
                  side == FOX_DRIVE_RIGHT ? "keep right, random turns" : "keep left, random turns");
    FoxWorldDestroy(world);
}

static void TestRouted(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, LOAD);
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

/* Whether a car stands on a pedestrian crossing somebody walks on. */
static bool CarOnWalkedCrossing(const FoxWorld *world)
{
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const FoxNetwork *network = FoxWorldNetwork(world);
    for (uint32_t slot = 0u; slot < FoxTrafficCapacity(traffic); ++slot)
    {
        FoxVehicleView view;
        if (!FoxTrafficVehicle(traffic, slot, &view))
        {
            continue;
        }
        const FoxPath *lane = FoxNetworkPathRecord(network, view.path);
        for (uint32_t index = 0u;
             lane != NULL && lane->kind == PATH_LANE && index < lane->crossingCount; ++index)
        {
            const uint32_t walk = lane->crossingWalk[index];
            const int64_t at = lane->crossingAt[index];
            if (walk != FOX_NETWORK_NONE && network->paths[walk].claims != 0u &&
                view.distance > at - FOX_METRES(1) &&
                view.distance - view.length < at + FOX_METRES(1))
            {
                return true;
            }
        }
    }
    return false;
}

/* People walk the town's footways and over its crossings; no car is ever on
 * a crossing while somebody walks it, and the same seed walks the same. */
static void TestWalkers(void)
{
    uint64_t hashes[2] = {0u, 0u};
    for (uint32_t run = 0u; run < 2u; ++run)
    {
        TestRouterInit(&routerA);
        const FoxWorldDesc desc = {.seed = 11u,
                                   .driveSide = FOX_DRIVE_RIGHT,
                                   .targetVehicles = LOAD,
                                   .targetWalkers = 300u,
                                   .router = &routerA.router};
        FoxWorld *world = FoxWorldCreate(&desc);
        EXPECT(world != NULL && FoxWorldBuildDemoTown(world), "the town");
        EXPECT(FoxNetworkGetStats(FoxWorldNetwork(world)).crossings != 0u, "it has crossings");
        bool unsafe = false;
        for (uint64_t tick = 0u; tick < SECONDS(240); ++tick)
        {
            FoxWorldAdvance(world, 1u);
            unsafe = unsafe || CarOnWalkedCrossing(world);
        }
        const FoxWalkersStats stats = FoxWalkersGetStats(FoxWorldWalkers(world));
        printf("walkers: %u out, %u waiting, %u crossing, %llu crossed\n", stats.walkers,
               stats.waiting, stats.crossing, (unsigned long long)stats.crossed);
        EXPECT(stats.walkers == 300u, "everybody is out walking");
        EXPECT(stats.crossed != 0u, "people walk over the crossings");
        EXPECT(!unsafe, "no car is on a crossing while somebody walks it");
        FoxWorldSetTargetWalkers(world, 100u);
        FoxWorldAdvance(world, SECONDS(120));
        EXPECT(FoxWalkersGetStats(FoxWorldWalkers(world)).walkers < 300u,
               "fewer wanted, some go home");
        hashes[run] = FoxWorldHash(world);
        FoxWorldDestroy(world);
    }
    EXPECT(hashes[0] == hashes[1], "the same seed walks the same");
}

/* Digging or filling right on a road leaves the ground it rests on and the
 * air over it; beside the road the ground changes. */
static void TestGroundUnderRoads(void)
{
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, NULL, 0u);
    const FoxPlanet *planet = FoxWorldPlanet(world);
    const int64_t radius = FoxPlanetRadius(planet);
    /* A road of the town runs along y = 0; the middle of a block lies 40 m
     * off it. */
    const FoxVec onRoad = {FOX_METRES(40), 0, radius};
    const FoxVec offRoad = {FOX_METRES(40), FOX_METRES(40), radius};
    FoxVec before;
    EXPECT(FoxPlanetGround(planet, onRoad, &before), "ground under the road");
    FoxPlanetBox changed;
    (void)FoxWorldDig(world, onRoad, FOX_METRES(6), &changed);
    (void)FoxWorldFill(world, (FoxVec){onRoad.x, onRoad.y, radius + FOX_METRES(3)}, FOX_METRES(5),
                       &changed);
    FoxVec after;
    EXPECT(FoxPlanetGround(planet, onRoad, &after), "ground still under the road");
    EXPECT(FoxVecDistance(before, after) < FOX_METRES(1) / 10, "the road keeps its ground");
    EXPECT(FoxWorldDig(world, offRoad, FOX_METRES(6), &changed) > 0u, "a block can be dug");
    FoxVec dug;
    EXPECT(FoxPlanetGround(planet, offRoad, &dug) && FoxVecLength(dug) < radius - FOX_METRES(3),
           "the block is dug down");
    FoxWorldDestroy(world);
}

static void TestEditsUnderTraffic(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, LOAD);
    FoxWorldAdvance(world, SECONDS(60));
    const int64_t radius = FoxPlanetRadius(FoxWorldPlanet(world));
    /* A diagonal crossing several roads while cars drive on them. */
    const FoxRoadPlan diagonal = {
        .start = {FOX_METRES(-190), FOX_METRES(-130), radius},
        .end = {FOX_METRES(130), FOX_METRES(190), radius},
        .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET),
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

static FoxVec Pole(const FoxWorld *world, int64_t east, int64_t north)
{
    const FoxPlanet *planet = FoxWorldPlanet(world);
    FoxVec ground = {FOX_METRES(east), FOX_METRES(north), FoxPlanetRadius(planet)};
    EXPECT(FoxPlanetGround(planet, ground, &ground), "the town stands on ground");
    return ground;
}

/* The sign in force for the traffic arriving at a node along a road. */
static FoxSign SignAt(const FoxNetwork *network, uint32_t road, uint32_t node)
{
    FoxRoadView view;
    EXPECT(FoxNetworkRoad(network, road, &view), "the road exists");
    return view.sign[view.nodes[0] == node ? 0u : 1u];
}

static void TestSigns(void)
{
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, NULL, 0u);
    const FoxNetwork *network = FoxWorldNetwork(world);
    /* The north-south avenue crosses a street: the avenue has priority. */
    const uint32_t mixed = FoxNetworkNearestNode(network, Pole(world, 0, 80), FOX_METRES(12));
    FoxNodeView node;
    EXPECT(mixed != FOX_NETWORK_NONE && FoxNetworkNode(network, mixed, &node), "a junction");
    uint32_t priority = 0u;
    uint32_t yield = 0u;
    for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
    {
        FoxRoadView road;
        EXPECT(FoxNetworkRoad(network, node.roads[slot], &road), "a road of the junction");
        const FoxSign sign = SignAt(network, node.roads[slot], mixed);
        EXPECT(sign == (road.layout.lanes[0] == 2u ? FOX_SIGN_PRIORITY : FOX_SIGN_YIELD),
               "the bigger road has priority and the smaller gives way");
        priority += sign == FOX_SIGN_PRIORITY ? 1u : 0u;
        yield += sign == FOX_SIGN_YIELD ? 1u : 0u;
    }
    EXPECT(priority == 2u && yield == 2u, "two of each");
    /* Equal streets need no signs; the demo corner has stop signs. */
    const uint32_t equal = FoxNetworkNearestNode(network, Pole(world, 80, 80), FOX_METRES(12));
    EXPECT(FoxNetworkNode(network, equal, &node), "a junction of streets");
    EXPECT(SignAt(network, node.roads[0], equal) == FOX_SIGN_NONE, "equal roads are unsigned");
    const uint32_t corner = FoxNetworkNearestNode(network, Pole(world, -80, -80), FOX_METRES(12));
    EXPECT(FoxNetworkNode(network, corner, &node), "the corner");
    EXPECT(SignAt(network, node.roads[1], corner) == FOX_SIGN_STOP, "the corner has stop signs");
    /* The mayor's sign wins, and taking it back restores the network's. */
    FoxRoadView road;
    EXPECT(FoxNetworkNode(network, equal, &node) && FoxNetworkRoad(network, node.roads[0], &road),
           "a street");
    const uint32_t end = road.nodes[0] == equal ? 0u : 1u;
    EXPECT(FoxWorldSetSign(world, node.roads[0], end, FOX_SIGN_PRIORITY), "a priority sign");
    EXPECT(SignAt(network, node.roads[0], equal) == FOX_SIGN_PRIORITY, "it is in force");
    EXPECT(SignAt(network, node.roads[1], equal) == FOX_SIGN_YIELD, "the others give way");
    EXPECT(FoxWorldSetSign(world, node.roads[0], end, FOX_SIGN_AUTO), "back to the network's");
    EXPECT(SignAt(network, node.roads[1], equal) == FOX_SIGN_NONE, "unsigned again");
    EXPECT(!FoxWorldSetSign(world, node.roads[0], 2u, FOX_SIGN_STOP), "a road has two ends");
    FoxWorldDestroy(world);
}

static void TestSpeedLimits(void)
{
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, NULL, 0u);
    const FoxNetwork *network = FoxWorldNetwork(world);
    const uint32_t road =
        FoxNetworkNearestRoad(network, Pole(world, 160, 40), FOX_METRES(12), NULL);
    FoxRoadView view;
    EXPECT(road != FOX_NETWORK_NONE && FoxNetworkRoad(network, road, &view), "a street");
    EXPECT(view.speedLimit == FOX_KMH(50) && !view.speedSet, "streets start at 50 km/h");
    EXPECT(!FoxWorldSetSpeedLimit(world, road, FOX_KMH(5)), "too slow for a sign");
    EXPECT(!FoxWorldSetSpeedLimit(world, road, FOX_KMH(200)), "too fast for a sign");
    EXPECT(FoxWorldSetSpeedLimit(world, road, FOX_KMH(30)), "a 30 zone");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.speedLimit == FOX_KMH(30) && view.speedSet,
           "the limit is in force");
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < view.laneCount[direction]; ++lane)
        {
            FoxPathView path;
            EXPECT(FoxNetworkPath(network, view.lanes[direction][lane], &path) &&
                       path.speedLimit <= FOX_KMH(30),
                   "its lanes follow it");
        }
    }
    EXPECT(FoxWorldSetSpeedLimit(world, road, 0), "back to the type's limit");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.speedLimit == FOX_KMH(50), "50 again");
    FoxWorldDestroy(world);
}

static void TestSignals(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, LOAD);
    const FoxNetwork *network = FoxWorldNetwork(world);
    const uint32_t centre = FoxNetworkNearestNode(network, Pole(world, 0, 0), FOX_METRES(12));
    FoxNodeView node;
    EXPECT(FoxNetworkNode(network, centre, &node) && node.signals, "lights at the centre");
    EXPECT(node.roadCount == 4u, "four roads meet there");
    FoxVec away[FOX_NODE_MAX_ROADS];
    uint32_t ends[FOX_NODE_MAX_ROADS];
    for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
    {
        FoxRoadView road;
        EXPECT(FoxNetworkRoad(network, node.roads[slot], &road), "a road at the lights");
        ends[slot] = road.nodes[0] == centre ? 0u : 1u;
        const FoxVec other = road.centre[ends[slot] == 0u ? road.centreCount - 1u : 0u];
        away[slot] = FoxVecDirection(FoxVecSub(other, node.position));
    }
    uint32_t greens[FOX_NODE_MAX_ROADS] = {0u};
    for (uint32_t second = 0u; second < 240u; ++second)
    {
        FoxWorldAdvance(world, SECONDS(1));
        for (uint32_t a = 0u; a < node.roadCount; ++a)
        {
            const FoxLight light = FoxNetworkLight(network, node.roads[a], ends[a]);
            EXPECT(light != FOX_LIGHT_NONE, "every approach has a light");
            if (light != FOX_LIGHT_GREEN && light != FOX_LIGHT_YELLOW)
            {
                continue;
            }
            ++greens[a];
            for (uint32_t b = a + 1u; b < node.roadCount; ++b)
            {
                const FoxLight other = FoxNetworkLight(network, node.roads[b], ends[b]);
                EXPECT((other != FOX_LIGHT_GREEN && other != FOX_LIGHT_YELLOW) ||
                           FoxVecDot(away[a], away[b]) < 0,
                       "only opposite approaches go together");
            }
        }
    }
    for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
    {
        EXPECT(greens[slot] > 10u, "every approach gets green");
    }
    ExpectNoOverlap(world);
    ExpectFlowing(world, "with lights");
    EXPECT(FoxWorldSetSignals(world, centre, false), "lights come down");
    EXPECT(FoxNetworkLight(network, node.roads[0], ends[0]) == FOX_LIGHT_NONE, "no lights");
    const uint32_t bend = FoxNetworkNearestNode(network, Pole(world, 200, 160), FOX_METRES(12));
    EXPECT(bend == FOX_NETWORK_NONE || !FoxWorldSetSignals(world, bend, true) ||
               (FoxNetworkNode(network, bend, &node) && node.roadCount >= 3u),
           "no lights where fewer than three roads meet");
    FoxWorldDestroy(world);
}

/* Trucks drive among the cars, slower; nobody speeds. */
static void TestVehicles(void)
{
    TestRouterInit(&routerA);
    FoxWorld *world = Town(FOX_DRIVE_RIGHT, &routerA.router, LOAD);
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const FoxNetwork *network = FoxWorldNetwork(world);
    uint32_t trucks = 0u;
    int32_t fastest = 0;
    for (uint32_t second = 0u; second < 180u; ++second)
    {
        FoxWorldAdvance(world, SECONDS(1));
        for (uint32_t slot = 0u; slot < FoxTrafficCapacity(traffic); ++slot)
        {
            FoxVehicleView vehicle;
            FoxPathView path;
            if (!FoxTrafficVehicle(traffic, slot, &vehicle) ||
                !FoxNetworkPath(network, vehicle.path, &path))
            {
                continue;
            }
            trucks += vehicle.kind == FOX_VEHICLE_TRUCK && second == 179u ? 1u : 0u;
            EXPECT(vehicle.length ==
                       (vehicle.kind == FOX_VEHICLE_TRUCK ? FOX_TRUCK_LENGTH : FOX_CAR_LENGTH),
                   "a vehicle is as long as its kind");
            /* Within a fifth and a walking pace: braking into a turn takes a moment. */
            EXPECT((int64_t)vehicle.speed * 100 <= (int64_t)path.speedLimit * 120 + FOX_METRES(50),
                   "nobody drives much above the limit");
            EXPECT(vehicle.kind != FOX_VEHICLE_TRUCK || vehicle.speed <= FOX_KMH(81),
                   "trucks keep to their top speed");
            fastest = vehicle.speed > fastest ? vehicle.speed : fastest;
            FoxVec position;
            FoxVec forward;
            EXPECT(FoxTrafficVehiclePose(traffic, network, slot, FOX_UNIT / 2, &position, &forward),
                   "every vehicle has a pose");
            EXPECT(FoxVecLength(forward) > FOX_UNIT * 9 / 10, "and faces somewhere");
        }
    }
    EXPECT(trucks > 0u, "trucks are on the road");
    EXPECT(fastest > FOX_KMH(45), "cars drive at town speeds");
    printf("fastest %.0f km/h, %u trucks\n", (double)fastest * 3.6 / FOX_UNITS_PER_METRE, trucks);
    FoxWorldDestroy(world);
}

int main(void)
{
    TestSigns();
    TestSpeedLimits();
    TestSignals();
    TestVehicles();
    TestRandomTurns(FOX_DRIVE_RIGHT);
    TestRandomTurns(FOX_DRIVE_LEFT);
    TestRouted();
    TestDeterminism();
    TestEditsUnderTraffic();
    TestGroundUnderRoads();
    TestWalkers();
    return 0;
}
