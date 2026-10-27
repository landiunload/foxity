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
    return (FoxRoadPlan){.start = start,
                         .end = end,
                         .layout = FoxRoadLayoutPreset(type),
                         .speedLimit = FoxRoadPresetSpeed(type)};
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
        if (in->road != FOX_NETWORK_NONE && network->roads[in->road].layout.lanes[0] == 2u &&
            a->movement == MOVE_STRAIGHT)
        {
            for (uint32_t c = 0u; c < a->conflictCount; ++c)
            {
                const FoxPath *b = FoxNetworkPathRecord(network, a->conflicts[c].path);
                const FoxPath *bIn = FoxNetworkPathRecord(network, b->from);
                if (network->roads[bIn->road].layout.lanes[0] == 1u)
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
    EXPECT(FoxNetworkBuildRoad(network, planet,
                               &(FoxRoadPlan){.start = Pole(0, 0),
                                              .end = Pole(0, 60),
                                              .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET)}) ==
               FOX_ROAD_OK,
           "a street ending in the open");
    EXPECT(FoxNetworkBuildRoad(network, planet,
                               &(FoxRoadPlan){.start = Pole(-60, 60),
                                              .end = Pole(60, 60),
                                              .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET)}) ==
               FOX_ROAD_OK,
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
        .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET),
    };
    EXPECT(FoxNetworkBuildRoad(network, planet, &across) == FOX_ROAD_OK,
           "a street across the bend");
    EXPECT(FoxNetworkRoadCount(network) == 4u && FoxNetworkNodeCount(network) == 5u,
           "both roads are cut once where they cross");
    FoxNetworkDestroy(network);
}

/* A road drawn a strip at a time: one lane, then one back beside it and a
 * footway; a lone lane turned one way, the other and both ways. */
static void TestStrips(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    const FoxRoadPlan lane = {
        .start = Pole(-50, 0), .end = Pole(50, 0), .layout = FoxRoadLayoutSingle(FOX_LANE_ASPHALT)};
    EXPECT(FoxNetworkBuildRoad(network, planet, &lane) == FOX_ROAD_OK, "a single lane");
    const uint32_t road = FirstRoad(network);
    FoxRoadView view;
    EXPECT(FoxNetworkRoad(network, road, &view) && view.layout.lanes[0] == 1u &&
               view.layout.lanes[1] == 0u,
           "one lane the way it was drawn");
    EXPECT(view.speedLimit == FOX_KMH(50), "a lone lane keeps 50 km/h");

    EXPECT(FoxNetworkTurnStrip(network, road, 0), "the lane turns");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.layout.lanes[0] == 0u &&
               view.layout.lanes[1] == 1u,
           "now it runs back");
    EXPECT(FoxNetworkTurnStrip(network, road, 0), "the lane turns again");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.layout.shared,
           "now it is driven both ways");
    EXPECT(view.speedLimit == FOX_KMH(30), "a shared lane is slow");
    EXPECT(FoxNetworkTurnStrip(network, road, 0) && FoxNetworkRoad(network, road, &view) &&
               !view.layout.shared && view.layout.lanes[0] == 1u,
           "and back to one way");

    EXPECT(FoxNetworkWidenRoad(network, road, 0u, FOX_LANE_CONCRETE, false) == FOX_ROAD_OK,
           "a concrete lane back beside it");
    EXPECT(FoxNetworkWidenRoad(network, road, 1u, FOX_LANE_FOOT, true) == FOX_ROAD_OK,
           "a footway on the right");
    EXPECT(FoxNetworkWidenRoad(network, road, 1u, FOX_LANE_FOOT, true) == FOX_ROAD_INVALID,
           "only one footway a side");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.laneCount[0] == 1u &&
               view.laneCount[1] == 1u && view.layout.foot[1] && !view.layout.foot[0],
           "a street with a footway");
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t count = FoxRoadLayoutStrips(&view.layout, FOX_DRIVE_RIGHT, strips);
    EXPECT(count == 3u && strips[0].kind == FOX_LANE_CONCRETE &&
               strips[0].way == FOX_WAY_BACKWARD && strips[1].kind == FOX_LANE_ASPHALT &&
               strips[1].way == FOX_WAY_FORWARD && strips[2].kind == FOX_LANE_FOOT,
           "the lanes keep right and the footway runs outside");
    EXPECT(strips[2].right - strips[2].left == FOX_FOOT_WIDTH, "a footway is half a lane");
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        const FoxPath *path = FoxNetworkPathRecord(network, view.lanes[direction][0]);
        FoxVec point;
        EXPECT(
            FoxNetworkPathPoint(network, view.lanes[direction][0], path->length / 2, &point, NULL),
            "a lane point");
        /* Forward, going east, keeps to the south of the centre line. */
        EXPECT(direction == 0u ? point.y < 0 : point.y > 0, "each way on its side");
    }

    EXPECT(FoxNetworkToggleCrossing(network, road, view.length / 2), "a crossing goes on");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.crossingCount == 1u, "one crossing");
    EXPECT(FoxNetworkToggleCrossing(network, road, view.length / 2 + FOX_METRES(2)) &&
               FoxNetworkRoad(network, road, &view) && view.crossingCount == 0u,
           "a click near it takes it off");

    /* Taken away a strip at a time, the last lane with the road. */
    EXPECT(FoxNetworkNarrowRoad(network, road, strips[2].left + FOX_FOOT_WIDTH / 2) ==
                   FOX_ROAD_OK &&
               FoxNetworkRoad(network, road, &view) && !view.layout.foot[1],
           "the footway goes");
    EXPECT(FoxNetworkNarrowRoad(network, road, -FOX_LANE_WIDTH / 2) == FOX_ROAD_OK &&
               FoxNetworkRoad(network, road, &view) && view.layout.lanes[0] == 1u &&
               view.layout.lanes[1] == 0u,
           "the lane back goes, the one forward stays");
    EXPECT(FoxNetworkNarrowRoad(network, road, 0) == FOX_ROAD_OK &&
               FoxNetworkRoadCount(network) == 0u,
           "the last lane takes the road");
    FoxNetworkDestroy(network);
}

/* A road running one way into a dead end leaves its cars a slow way back. */
static void TestWayBack(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    FoxRoadPlan plan = Plan(Pole(-50, 0), Pole(50, 0), FOX_ROAD_ONE_WAY);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a one-way road");
    FoxRoadView view;
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &view) && view.laneCount[0] == 1u &&
               view.laneCount[1] == 1u && view.layout.lanes[1] == 0u,
           "its inner lane runs back out of the dead end");
    const FoxPath *back = FoxNetworkPathRecord(network, view.lanes[1][0]);
    EXPECT(back->speedLimit <= FOX_KMH(20), "slowly");
    const FoxPath *in = FoxNetworkPathRecord(network, view.lanes[0][0]);
    EXPECT(in->outgoingCount != 0u, "the dead end lets the cars turn round");
    FoxNetworkDestroy(network);

    network = NewNetwork(FOX_DRIVE_RIGHT);
    plan = (FoxRoadPlan){
        .start = Pole(-50, 0), .end = Pole(50, 0), .layout = FoxRoadLayoutSingle(FOX_LANE_ASPHALT)};
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a lone lane");
    EXPECT(FoxNetworkRoad(network, FirstRoad(network), &view) && view.laneCount[0] == 1u &&
               view.laneCount[1] == 1u && !view.layout.shared,
           "driven both ways out of its dead end");
    /* Joined on beyond, the dead end is gone and so is the way back. */
    plan = (FoxRoadPlan){
        .start = Pole(50, 0), .end = Pole(150, 0), .layout = FoxRoadLayoutSingle(FOX_LANE_ASPHALT)};
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "the lane goes on");
    uint32_t oneWay = 0u;
    for (uint32_t index = 0u; index < FoxNetworkRoadCapacity(network); ++index)
    {
        oneWay += FoxNetworkRoad(network, index, &view) && view.laneCount[1] == 0u ? 1u : 0u;
    }
    EXPECT(oneWay == 1u, "only the last piece, into the new dead end, keeps a way back");
    FoxNetworkDestroy(network);
}

static FoxRoadPlan PathPlan(FoxVec start, FoxVec end, FoxLaneKind kind)
{
    return (FoxRoadPlan){.start = start, .end = end, .layout = FoxRoadLayoutSingle(kind)};
}

/* The node nearest a point, and its view. */
static uint32_t NodeAt(const FoxNetwork *network, FoxVec point, FoxNodeView *outView,
                       const char *what)
{
    /* Poles stand off the ground the further out they are. */
    const uint32_t node = FoxNetworkNearestNode(network, point, FOX_METRES(10));
    EXPECT(node != FOX_NETWORK_NONE && FoxNetworkNode(network, node, outView), what);
    return node;
}

/* The walk of a path road, whichever side it is on. */
static uint32_t PathWalk(const FoxNetwork *network, uint32_t road)
{
    const FoxRoad *record = &network->roads[road];
    return record->walkCount[0] != 0u ? record->walks[0][0] : record->walks[1][0];
}

/* Whether people can walk from one walk to another along the joins. */
static bool Walkable(const FoxNetwork *network, uint32_t from, uint32_t to)
{
    static uint32_t queue[1u << 14u];
    static bool seen[1u << 14u];
    const uint32_t capacity = network->pathCapacity;
    EXPECT(capacity <= (1u << 14u), "few paths");
    for (uint32_t index = 0u; index < capacity; ++index)
    {
        seen[index] = false;
    }
    uint32_t head = 0u;
    uint32_t tail = 0u;
    queue[tail++] = from;
    seen[from] = true;
    while (head < tail)
    {
        const uint32_t walk = queue[head++];
        if (walk == to)
        {
            return true;
        }
        const FoxPath *path = &network->paths[walk];
        for (uint32_t end = 0u; end < 2u; ++end)
        {
            for (uint32_t join = 0u; join < path->joinCount[end]; ++join)
            {
                const uint32_t next = path->joins[end][join] / 2u;
                if (!seen[next] && !network->paths[next].retired)
                {
                    seen[next] = true;
                    queue[tail++] = next;
                }
            }
        }
    }
    return false;
}

/* The road of a node other than `except`. */
static uint32_t RoadOf(const FoxNetwork *network, uint32_t node, uint32_t except)
{
    FoxNodeView view;
    EXPECT(FoxNetworkNode(network, node, &view), "the node");
    for (uint32_t slot = 0u; slot < view.roadCount; ++slot)
    {
        if (view.roads[slot] != except)
        {
            return view.roads[slot];
        }
    }
    return FOX_NETWORK_NONE;
}

/* Footways and bike lanes alone meet roads for cars from the side: across
 * one they leave a crossing, ending at one they join its footway, and never
 * lie along it or end in an intersection. */
static void TestPaths(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    const FoxRoadPlan street = Plan(Pole(-60, 0), Pole(60, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &street) == FOX_ROAD_OK, "a street");
    const uint32_t road = FirstRoad(network);

    FoxRoadPlan plan = PathPlan(Pole(0, -30), Pole(0, 30), FOX_LANE_FOOT);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a footway across");
    FoxRoadView view;
    EXPECT(FoxNetworkRoad(network, road, &view) && view.crossingCount == 1u &&
               FoxAbs(view.crossings[0] - FOX_METRES(60)) < FOX_METRES(1),
           "the street goes on uncut, with a crossing where the footway meets it");
    FoxNodeView tap;
    const uint32_t tapNode = NodeAt(network, Pole(0, 0), &tap, "a node where the footway crosses");
    EXPECT(tap.tap && tap.roadCount == 2u && tap.carRoadCount == 0u,
           "the two halves of the footway meet on the street's centre line");
    const FoxRoad *record = &network->roads[road];
    EXPECT(record->tapCount == 1u && record->tapWalks[0] != FOX_NETWORK_NONE,
           "people walk over it");
    for (uint32_t slot = 0u; slot < tap.roadCount; ++slot)
    {
        FoxRoadView half;
        EXPECT(FoxNetworkRoad(network, tap.roads[slot], &half), "a half");
        const uint32_t end = half.nodes[0] == tapNode ? 0u : 1u;
        EXPECT(half.trim[end] >= FOX_LANE_WIDTH + FOX_FOOT_WIDTH,
               "each half stops at the street's edge");
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, view.lanes[0][0]);
    EXPECT(lane->crossingCount == 1u && lane->crossingWalk[0] == record->tapWalks[0],
           "cars know where people cross");
    const uint32_t south = PathWalk(network, tap.roads[0]);
    const uint32_t north = PathWalk(network, tap.roads[1]);
    EXPECT(Walkable(network, south, north), "from one half over the street to the other");
    EXPECT(Walkable(network, south, record->walks[0][0]) &&
               Walkable(network, south, record->walks[1][1]),
           "and onto the street's footways either way");

    plan = PathPlan(Pole(30, -30), Pole(30, -2), FOX_LANE_FOOT);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a footway up to it");
    EXPECT(FoxNetworkRoad(network, road, &view) && view.crossingCount == 1u,
           "ending there it leaves no crossing");
    FoxNodeView end;
    const uint32_t endNode = NodeAt(network, Pole(30, 0), &end, "a node where the footway ends");
    EXPECT(end.tap && end.roadCount == 1u, "it ends on the street");
    EXPECT(Walkable(network, PathWalk(network, end.roads[0]), north),
           "and walks on along the footway and over the crossing");

    plan = PathPlan(Pole(-45, 9), Pole(45, -9), FOX_LANE_BIKE);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OVERLAP,
           "a bike lane cannot cross at a glancing angle, over the carriageway");
    plan = PathPlan(Pole(-40, 4), Pole(40, 4), FOX_LANE_BIKE);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) != FOX_ROAD_OK, "nor run along it");
    plan = PathPlan(Pole(60, -20), Pole(60, -1), FOX_LANE_FOOT);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OVERLAP,
           "a footway does not end in the street's dead end");
    plan = PathPlan(Pole(-30, 3), Pole(-30, 30), FOX_LANE_FOOT);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK,
           "a footway from the north footway");

    /* A road for cars over a footway: the footway meets it, the road goes on. */
    plan = PathPlan(Pole(120, -30), Pole(120, 30), FOX_LANE_FOOT);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a footway alone");
    const uint32_t roads = FoxNetworkRoadCount(network);
    FoxRoadPlan lone = {
        .start = Pole(90, 0), .end = Pole(150, 0), .layout = FoxRoadLayoutSingle(FOX_LANE_ASPHALT)};
    EXPECT(FoxNetworkBuildRoad(network, planet, &lone) == FOX_ROAD_OK, "a lane over it");
    EXPECT(FoxNetworkRoadCount(network) == roads + 2u, "the lane is one road, the footway two");
    FoxNodeView over;
    (void)NodeAt(network, Pole(120, 0), &over, "a node where the lane runs over the footway");
    EXPECT(over.tap && over.roadCount == 2u, "the footway meets the lane from both sides");
    const uint32_t laneRoad =
        network->nodes[FoxNetworkNearestNode(network, Pole(120, 0), FOX_METRES(10))].tap;
    EXPECT(FoxNetworkRoad(network, laneRoad, &view) && view.crossingCount == 1u,
           "with a crossing, edge to edge without footways");
    EXPECT(Walkable(network, PathWalk(network, over.roads[0]), PathWalk(network, over.roads[1])),
           "people walk over the lane");
    plan = PathPlan(Pole(110, -20), Pole(110, -1), FOX_LANE_FOOT);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OVERLAP,
           "a footway cannot end at a lane with no edge to walk on");
    lone = (FoxRoadPlan){.start = Pole(150, 30),
                         .end = Pole(120, 15),
                         .layout = FoxRoadLayoutSingle(FOX_LANE_ASPHALT)};
    EXPECT(FoxNetworkCheckRoad(network, planet, &lone) == FOX_ROAD_OVERLAP,
           "nor a lane end on a footway");

    /* Wider, the street keeps its paths at its new edge. */
    EXPECT(FoxNetworkWidenRoad(network, road, 0u, FOX_LANE_BIKE, false) == FOX_ROAD_OK,
           "a bike lane along the street");
    FoxRoadView half;
    const uint32_t south0 = RoadOf(network, tapNode, FOX_NETWORK_NONE);
    EXPECT(FoxNetworkRoad(network, south0, &half), "a half of the footway");
    const uint32_t at = half.nodes[0] == tapNode ? 0u : 1u;
    EXPECT(half.trim[at] >= FOX_LANE_WIDTH + FOX_FOOT_WIDTH, "still at the edge");

    /* Gone, the street leaves its paths as they were. */
    EXPECT(FoxNetworkRemoveRoad(network, road), "the street goes");
    FoxNodeView left;
    EXPECT(FoxNetworkNode(network, endNode, &left) && !left.tap && left.roadCount == 1u,
           "the footway that ended on it ends on its own");
    EXPECT(FoxNetworkNode(network, tapNode, &left) && !left.tap && left.roadCount == 2u,
           "the one across it runs on");
    FoxNetworkReleaseRetired(network);
    FoxNetworkDestroy(network);
}

/* A curve tighter than a road's width would fold its inner edge over. */
static void TestSharpCurve(const FoxPlanet *planet)
{
    FoxNetwork *network = NewNetwork(FOX_DRIVE_RIGHT);
    FoxRoadPlan plan = Plan(Pole(200, 160), Pole(160, 200), FOX_ROAD_BOULEVARD);
    plan.curved = true;
    plan.control = Pole(250, 250);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_TOO_SHARP,
           "a boulevard cannot turn back that tightly");
    plan.layout = FoxRoadLayoutSingle(FOX_LANE_FOOT);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OK, "a footway can");
    plan = Plan(Pole(200, 160), Pole(160, 200), FOX_ROAD_BOULEVARD);
    plan.curved = true;
    plan.control = Pole(212, 212);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OK,
           "and a boulevard round a wider curve");

    /* Nor do two roads meet almost along each other. */
    plan = Plan(Pole(-60, 0), Pole(60, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a street");
    plan = Plan(Pole(-80, -12), Pole(0, 0), FOX_ROAD_STREET);
    const FoxRoadResult glancing = FoxNetworkCheckRoad(network, planet, &plan);
    EXPECT(glancing == FOX_ROAD_TOO_SHARP || glancing == FOX_ROAD_OVERLAP,
           "a street cannot join it at under twenty degrees");
    plan = Plan(Pole(-40, -40), Pole(0, 0), FOX_ROAD_STREET);
    EXPECT(FoxNetworkCheckRoad(network, planet, &plan) == FOX_ROAD_OK, "at forty-five it can");
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
    TestStrips(planet);
    TestWayBack(planet);
    TestPaths(planet);
    TestSharpCurve(planet);
    FoxPlanetDestroy(planet);
    return 0;
}
