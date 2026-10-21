#include "expect.h"
#include "sim/network.h"
#include "sim/network_internal.h"

#include <stdint.h>

#define RADIUS FOX_PLANET_DEFAULT_RADIUS

static FoxVec Pole(int64_t eastMetres, int64_t northMetres)
{
    return (FoxVec){FOX_METRES(eastMetres), FOX_METRES(northMetres), RADIUS};
}

static FoxRoadPlan Plan(FoxVec start, FoxVec end, FoxRoadType type)
{
    return (FoxRoadPlan){.start = start, .end = end, .type = type};
}

static FoxNetwork *NewNetwork(FoxDriveSide side)
{
    const FoxNetworkDesc desc = {.driveSide = side};
    FoxNetwork *network = FoxNetworkCreate(&desc);
    EXPECT(network != NULL, "the network is created");
    return network;
}

static uint32_t FirstRoad(const FoxNetwork *network)
{
    for (uint32_t index = 0u; index < FoxNetworkRoadCapacity(network); ++index)
    {
        FoxRoadView view;
        if (FoxNetworkRoad(network, index, &view))
        {
            return index;
        }
    }
    return FOX_NETWORK_NONE;
}

static void TestStraightStreet(const FoxPlanet *planet, FoxDriveSide side)
{
    FoxNetwork *network = NewNetwork(side);
    const FoxRoadPlan plan = Plan(Pole(-50, 0), Pole(50, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a street is built");
    EXPECT(FoxNetworkRoadCount(network) == 1u && FoxNetworkNodeCount(network) == 2u,
           "one road between two nodes");
    FoxRoadView road;
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &road), "the road is visible");
    EXPECT(road.laneCount[0] == 1u && road.laneCount[1] == 1u, "a street has a lane each way");
    EXPECT(FoxAbs(road.length - FOX_METRES(100)) < FOX_METRES(1), "the street is 100 m long");

    /* Travelling east from nodes[0], the lane keeps to the drive side: south
     * of the centre where traffic keeps right, north where it keeps left. */
    FoxPathView lane;
    EXPECT(FoxNetworkPath(network, road.lanes[0][0], &lane) && lane.count >= 2u, "a lane path");
    const FoxVec middle = lane.points[lane.count / 2u];
    const bool eastward = road.centre[road.centreCount - 1u].x > road.centre[0].x;
    const int64_t north = eastward ? middle.y : -middle.y;
    if (side == FOX_DRIVE_RIGHT)
    {
        EXPECT(north < -FOX_METRES(1), "keep right puts the forward lane on the right");
    }
    else
    {
        EXPECT(north > FOX_METRES(1), "keep left puts the forward lane on the left");
    }
    EXPECT(FoxAbs(FoxAbs(middle.y) - FOX_LANE_WIDTH / 2) < FOX_METRES(1) / 10,
           "the lane centre is half a lane from the road centre");

    /* Dead ends turn every lane around. */
    const FoxPath *record = FoxNetworkPathRecord(network, road.lanes[0][0]);
    EXPECT(record != NULL && record->outgoingCount == 1u, "the lane turns around at the end");
    const FoxPath *turn = FoxNetworkPathRecord(network, record->outgoing[0]);
    EXPECT(turn != NULL && turn->movement == MOVE_U_TURN && turn->to == road.lanes[1][0],
           "the turn leads into the opposite lane");
    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
}

static uint32_t BusiestNode(const FoxNetwork *network)
{
    uint32_t best = FOX_NETWORK_NONE;
    uint32_t roads = 0u;
    for (uint32_t index = 0u; index < FoxNetworkNodeCapacity(network); ++index)
    {
        FoxNodeView view;
        if (FoxNetworkNode(network, index, &view) && view.roadCount > roads)
        {
            best = index;
            roads = view.roadCount;
        }
    }
    return best;
}

static void TestCrossing(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    FoxRoadPlan plan = Plan(Pole(-60, 0), Pole(60, 0), FOX_ROAD_AVENUE);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the avenue is built");
    plan = Plan(Pole(0, -60), Pole(0, 60), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the crossing street");
    EXPECT(FoxNetworkRoadCount(network) == 4u && FoxNetworkNodeCount(network) == 5u,
           "crossing splits both roads at a new intersection");
    const uint32_t centre = BusiestNode(network);
    FoxNodeView node;
    EXPECT(FoxNetworkNode(network, centre, &node) && node.roadCount == 4u,
           "the intersection joins four roads");
    EXPECT(FoxVecLength((FoxVec){node.position.x, node.position.y, 0}) < FOX_METRES(1),
           "the intersection lies where the roads cross");

    const FoxNode *record = &network->nodes[centre];
    EXPECT(record->connectionCount > 8u, "the intersection has turns");
    uint32_t conflicts = 0u;
    for (uint32_t index = 0u; index < record->connectionCount; ++index)
    {
        const FoxPath *a = FoxNetworkPathRecord(network, record->connections[index]);
        EXPECT(a != NULL && a->kind == PATH_CONNECTION && !a->retired, "connections are live");
        for (uint32_t c = 0u; c < a->conflictCount; ++c)
        {
            const FoxPath *b = FoxNetworkPathRecord(network, a->conflicts[c].path);
            bool mirrored = false;
            for (uint32_t d = 0u; d < b->conflictCount; ++d)
            {
                if (b->conflicts[d].path == record->connections[index])
                {
                    /* Two turning across each other's way head on both
                     * wait, and whoever came first goes first. */
                    mirrored = b->conflicts[d].yields != a->conflicts[c].yields ||
                               (a->movement == MOVE_FAR && b->movement == MOVE_FAR &&
                                a->conflicts[c].yields && b->conflicts[d].yields);
                }
            }
            EXPECT(mirrored, "of two conflicting turns exactly one yields, or both turning across");
            ++conflicts;
        }
        /* The avenue is the bigger road: going straight along it never
         * yields to the street. */
        const FoxPath *in = FoxNetworkPathRecord(network, a->from);
        if (in->road != FOX_NETWORK_NONE && network->roads[in->road].type == FOX_ROAD_AVENUE &&
            a->movement == MOVE_STRAIGHT)
        {
            for (uint32_t c = 0u; c < a->conflictCount; ++c)
            {
                const FoxPath *b = FoxNetworkPathRecord(network, a->conflicts[c].path);
                const FoxPath *bIn = FoxNetworkPathRecord(network, b->from);
                if (network->roads[bIn->road].type == FOX_ROAD_STREET)
                {
                    EXPECT(!a->conflicts[c].yields, "the avenue has priority over the street");
                }
            }
        }
    }
    EXPECT(conflicts > 0u, "crossing turns conflict");

    /* Every lane into the intersection leads somewhere. */
    for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
    {
        FoxRoadView road;
        EXPECT(FoxNetworkRoad(network, node.roads[slot], &road), "roads at the intersection");
        const uint32_t direction = road.nodes[1] == centre ? 0u : 1u;
        for (uint32_t lane = 0u; lane < road.laneCount[direction]; ++lane)
        {
            const FoxPath *path = FoxNetworkPathRecord(network, road.lanes[direction][lane]);
            EXPECT(path->outgoingCount > 0u, "each incoming lane has a way on");
        }
    }

    plan = Plan(Pole(0, -60), Pole(0, 60), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OVERLAP,
           "a road over a road is refused");
    plan = Plan(Pole(30, 30), Pole(33, 30), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_TOO_SHORT, "a stub is refused");
    plan = Plan(Pole(4, 3), Pole(1, 50), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OVERLAP ||
               FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_TOO_SHORT,
           "a road along a road is refused");

    /* A T-junction: a road ending on the avenue splits it. */
    const uint64_t before = FoxNetworkRevision(network);
    plan = Plan(Pole(40, 40), Pole(40, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the T-junction");
    EXPECT(FoxNetworkRoadCount(network) == 6u && FoxNetworkNodeCount(network) == 7u,
           "the avenue is split where the street joins");
    EXPECT(FoxNetworkRevision(network) > before, "the revision moves");

    FoxNetworkReleaseRetired(network);
    const uint64_t hash = FoxNetworkHash(network);
    EXPECT(FoxNetworkRemoveRoad(network, FirstRoad(network)), "a road is removed");
    EXPECT(FoxNetworkRoadCount(network) == 5u, "one road fewer");
    EXPECT(FoxNetworkHash(network) != hash, "the hash sees the removal");
    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
}

static void TestZones(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    const FoxRoadPlan plan = Plan(Pole(-50, 0), Pole(50, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a street is built");
    FoxRoadView road;
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &road) && road.driveSide == FOX_DRIVE_RIGHT,
           "the city keeps right");
    EXPECT(FoxNetworkAddDriveZone(network, Pole(0, 0), FOX_METRES(200), FOX_DRIVE_LEFT),
           "a keep-left zone");
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &road) && road.driveSide == FOX_DRIVE_LEFT,
           "the street inside the zone keeps left");
    EXPECT(FoxNetworkDriveSideAt(network, Pole(0, 0)) == FOX_DRIVE_LEFT &&
               FoxNetworkDriveSideAt(network, (FoxVec){RADIUS, 0, 0}) == FOX_DRIVE_RIGHT,
           "zones are local");
    FoxPathView lane;
    EXPECT(FoxNetworkPath(network, road.lanes[0][0], &lane), "the new lane");
    const bool eastward = road.centre[road.centreCount - 1u].x > road.centre[0].x;
    const int64_t north =
        eastward ? lane.points[lane.count / 2u].y : -lane.points[lane.count / 2u].y;
    EXPECT(north > 0, "lanes move to the other side");
    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
}

static void TestCurve(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    FoxRoadPlan plan = Plan(Pole(-80, 0), Pole(80, 0), FOX_ROAD_BOULEVARD);
    plan.curved = true;
    plan.control = Pole(0, 80);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a curved boulevard");
    FoxRoadView road;
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &road), "the curve exists");
    EXPECT(road.length > FOX_METRES(180) && road.centreCount > 9u,
           "a 40 m bow over a 160 m chord is about 187 m long");
    int64_t highest = 0;
    for (uint32_t point = 0u; point < road.centreCount; ++point)
    {
        highest = FoxMax(highest, road.centre[point].y);
        EXPECT(FoxAbs(FoxVecLength(road.centre[point]) - RADIUS) < FOX_METRES(1) / 20,
               "the road lies on the ground");
    }
    EXPECT(FoxAbs(highest - FOX_METRES(40)) < FOX_METRES(2), "the curve bends towards its control");
    EXPECT(road.laneCount[0] == 3u && road.laneCount[1] == 3u, "three lanes each way");
    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
}

/* A road laid across the dead end of another joins it there. */
static void TestThroughDeadEnd(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    EXPECT(FoxNetworkBuildRoad(
               network, planet,
               &(FoxRoadPlan){.start = Pole(0, 0), .end = Pole(0, 60), .type = FOX_ROAD_STREET}) ==
               FOX_ROAD_OK,
           "a street ending in the open");
    EXPECT(FoxNetworkBuildRoad(network, planet,
                               &(FoxRoadPlan){.start = Pole(-60, 60),
                                              .end = Pole(60, 60),
                                              .type = FOX_ROAD_STREET}) == FOX_ROAD_OK,
           "a street across its end");
    EXPECT(FoxNetworkRoadCount(network) == 3u, "the crossing street is cut at the end");
    EXPECT(FoxNetworkNodeCount(network) == 4u, "the end became a junction");
    const FoxNetworkStats stats = FoxNetworkGetStats(network);
    EXPECT(stats.junctions == 1u && stats.deadEnds == 3u, "one junction, three dead ends");
    FoxNetworkDestroy(network);
}

/* A street laid across a curve right through one of the points the curve
 * bends at crosses it once, though the curve's pieces on either side of
 * that point both meet it there. */
static void TestThroughBend(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    FoxRoadPlan plan = Plan(Pole(-80, 0), Pole(80, 0), FOX_ROAD_STREET);
    plan.curved = true;
    plan.control = Pole(0, 80);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a curved street");
    FoxRoadView road;
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &road), "the curve exists");
    const FoxVec bend = road.centre[road.centreCount / 2u];
    const FoxRoadPlan across = {
        .start = {bend.x, bend.y - FOX_METRES(60), RADIUS},
        .end = {bend.x, bend.y + FOX_METRES(60), RADIUS},
        .type = FOX_ROAD_STREET,
    };
    EXPECT(FoxNetworkBuildRoad(network, planet, &across) == FOX_ROAD_OK,
           "a street across the bend");
    EXPECT(FoxNetworkRoadCount(network) == 4u && FoxNetworkNodeCount(network) == 5u,
           "both roads are cut once where they cross");
    FoxNetworkDestroy(network);
}

int main(void)
{
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    EXPECT(planet != NULL, "the planet");
    TestStraightStreet(planet, FOX_DRIVE_RIGHT);
    TestStraightStreet(planet, FOX_DRIVE_LEFT);
    TestCrossing(planet);
    TestZones(planet);
    TestCurve(planet);
    TestThroughDeadEnd(planet);
    TestThroughBend(planet);
    FoxPlanetDestroy(planet);
    return 0;
}
