#include "sim/world.h"

#include "sim/random.h"

#include <stddef.h>
#include <stdlib.h>

/* A road or drive zone laid this close to a road still closing takes that
 * one out first: the intersections it makes may reach this far. */
#define EDIT_REACH FOX_METRES(40)

struct FoxWorld
{
    uint64_t tick;
    FoxPlanet *planet;
    FoxNetwork *network;
    FoxTraffic *traffic;
};

FoxWorld *FoxWorldCreate(const FoxWorldDesc *desc)
{
    if (desc == NULL)
    {
        return NULL;
    }
    FoxWorld *world = calloc(1u, sizeof(*world));
    if (world == NULL)
    {
        return NULL;
    }
    const FoxNetworkDesc networkDesc = {.driveSide = desc->driveSide, .router = desc->router};
    const FoxTrafficDesc trafficDesc = {.seed = desc->seed, .targetVehicles = desc->targetVehicles};
    world->planet =
        FoxPlanetCreate(desc->planetRadius != 0 ? desc->planetRadius : FOX_PLANET_DEFAULT_RADIUS);
    world->network = FoxNetworkCreate(&networkDesc);
    world->traffic = FoxTrafficCreate(&trafficDesc);
    if (world->planet == NULL || world->network == NULL || world->traffic == NULL)
    {
        FoxWorldDestroy(world);
        return NULL;
    }
    return world;
}

void FoxWorldDestroy(FoxWorld *world)
{
    if (world == NULL)
    {
        return;
    }
    FoxTrafficDestroy(world->traffic);
    FoxNetworkDestroy(world->network);
    FoxPlanetDestroy(world->planet);
    free(world);
}

uint64_t FoxWorldTick(const FoxWorld *world)
{
    return world == NULL ? 0u : world->tick;
}

/* Cars leave retired paths before the network frees them. */
static void Settle(FoxWorld *world)
{
    FoxTrafficRemap(world->traffic, world->network);
    FoxNetworkReleaseRetired(world->network);
}

void FoxWorldAdvance(FoxWorld *world, uint64_t ticks)
{
    if (world == NULL)
    {
        return;
    }
    for (uint64_t step = 0u; step < ticks; ++step)
    {
        FoxTrafficStep(world->traffic, world->network);
        ++world->tick;
        /* A road removed under cars goes for good once they are off it. */
        if (FoxNetworkSettleClosing(world->network, (FoxVec){0, 0, 0}, 0))
        {
            Settle(world);
        }
    }
}

const FoxPlanet *FoxWorldPlanet(const FoxWorld *world)
{
    return world == NULL ? NULL : world->planet;
}

const FoxNetwork *FoxWorldNetwork(const FoxWorld *world)
{
    return world == NULL ? NULL : world->network;
}

const FoxTraffic *FoxWorldTraffic(const FoxWorld *world)
{
    return world == NULL ? NULL : world->traffic;
}

static bool GuardTouches(const void *context, FoxVec min, FoxVec max)
{
    return FoxGroundGuardTouches((const FoxGroundGuard *)context, min, max);
}

static bool GuardKeeps(const void *context, FoxVec point)
{
    return FoxGroundGuardKeeps((const FoxGroundGuard *)context, point);
}

/* Digs or fills around the roads: the ground they rest on and the air over
 * them stay as they are. */
static uint32_t EditGround(FoxWorld *world, bool dig, FoxVec center, int64_t radius,
                           FoxPlanetBox *outChanged)
{
    if (world == NULL)
    {
        return 0u;
    }
    FoxGroundGuard guard;
    FoxNetworkGuardGround(world->network, center, radius, &guard);
    const FoxPlanetKeep keep = {.context = &guard, .touches = GuardTouches, .keeps = GuardKeeps};
    return dig ? FoxPlanetDig(world->planet, center, radius, &keep, outChanged)
               : FoxPlanetFill(world->planet, center, radius, &keep, outChanged);
}

uint32_t FoxWorldDig(FoxWorld *world, FoxVec center, int64_t radius, FoxPlanetBox *outChanged)
{
    return EditGround(world, true, center, radius, outChanged);
}

uint32_t FoxWorldFill(FoxWorld *world, FoxVec center, int64_t radius, FoxPlanetBox *outChanged)
{
    return EditGround(world, false, center, radius, outChanged);
}

FoxRoadResult FoxWorldBuildRoad(FoxWorld *world, const FoxRoadPlan *plan)
{
    if (world == NULL || plan == NULL)
    {
        return FOX_ROAD_INVALID;
    }
    /* Roads still closing where the new one goes go for good first. */
    const FoxVec middle = FoxVecScale(FoxVecAdd(plan->start, plan->end), 1, 2);
    const int64_t reach = FoxVecDistance(plan->start, plan->end) / 2 +
                          (plan->curved ? FoxVecDistance(plan->control, middle) : 0) + EDIT_REACH;
    if (FoxNetworkSettleClosing(world->network, middle, reach))
    {
        Settle(world);
    }
    const FoxRoadResult result = FoxNetworkBuildRoad(world->network, world->planet, plan);
    Settle(world);
    return result;
}

bool FoxWorldRemoveRoad(FoxWorld *world, uint32_t road)
{
    if (world == NULL)
    {
        return false;
    }
    const bool removed = FoxNetworkRemoveRoad(world->network, road);
    Settle(world);
    return removed;
}

bool FoxWorldAddDriveZone(FoxWorld *world, FoxVec center, int64_t radius, FoxDriveSide side)
{
    if (world == NULL)
    {
        return false;
    }
    if (FoxNetworkSettleClosing(world->network, center, radius + EDIT_REACH))
    {
        Settle(world);
    }
    const bool added = FoxNetworkAddDriveZone(world->network, center, radius, side);
    Settle(world);
    return added;
}

bool FoxWorldSetSpeedLimit(FoxWorld *world, uint32_t road, int32_t speed)
{
    return world != NULL && FoxNetworkSetSpeedLimit(world->network, road, speed);
}

bool FoxWorldSetSign(FoxWorld *world, uint32_t road, uint32_t end, FoxSign sign)
{
    return world != NULL && FoxNetworkSetSign(world->network, road, end, sign);
}

bool FoxWorldSetSignals(FoxWorld *world, uint32_t node, bool on)
{
    return world != NULL && FoxNetworkSetSignals(world->network, node, on);
}

void FoxWorldSetTargetVehicles(FoxWorld *world, uint32_t vehicles)
{
    if (world != NULL)
    {
        FoxTrafficSetTarget(world->traffic, vehicles);
    }
}

static FoxVec PoleOffset(const FoxWorld *world, int64_t east, int64_t north)
{
    return (FoxVec){FOX_METRES(east), FOX_METRES(north), FoxPlanetRadius(world->planet)};
}

/* The ground under a point of the town. */
static FoxVec Ground(const FoxWorld *world, int64_t east, int64_t north)
{
    FoxVec point = PoleOffset(world, east, north);
    (void)FoxPlanetGround(world->planet, point, &point);
    return point;
}

bool FoxWorldBuildDemoTown(FoxWorld *world)
{
    if (world == NULL)
    {
        return false;
    }
    bool ok = true;
    /* Five streets each way 80 m apart; the middle ones are avenues. */
    for (int64_t line = -2; line <= 2; ++line)
    {
        const FoxRoadType type = line == 0 ? FOX_ROAD_AVENUE : FOX_ROAD_STREET;
        const FoxRoadPlan northSouth = {
            .start = PoleOffset(world, line * 80, -200),
            .end = PoleOffset(world, line * 80, 200),
            .type = type,
        };
        ok = FoxWorldBuildRoad(world, &northSouth) == FOX_ROAD_OK && ok;
    }
    for (int64_t line = -2; line <= 2; ++line)
    {
        const FoxRoadType type = line == 0 ? FOX_ROAD_AVENUE : FOX_ROAD_STREET;
        const FoxRoadPlan eastWest = {
            .start = PoleOffset(world, -200, line * 80),
            .end = PoleOffset(world, 200, line * 80),
            .type = type,
        };
        ok = FoxWorldBuildRoad(world, &eastWest) == FOX_ROAD_OK && ok;
    }
    /* A curved boulevard round the north-east corner. */
    const FoxRoadPlan bend = {
        .start = PoleOffset(world, 200, 160),
        .end = PoleOffset(world, 160, 200),
        .control = PoleOffset(world, 250, 250),
        .curved = true,
        .type = FOX_ROAD_BOULEVARD,
    };
    ok = FoxWorldBuildRoad(world, &bend) == FOX_ROAD_OK && ok;
    /* A short street out of town that ends in a turning circle. */
    const FoxRoadPlan spur = {
        .start = PoleOffset(world, -120, -200),
        .end = PoleOffset(world, -120, -260),
        .type = FOX_ROAD_STREET,
    };
    ok = FoxWorldBuildRoad(world, &spur) == FOX_ROAD_OK && ok;
    /* East of town the avenue runs on into a square where five roads meet at
     * uneven angles; a boulevard sweeps round behind it, one street ends in a
     * turning circle and another doubles back sharply. */
    static const int16_t district[][7] = {
        /* east, north of the start and end, of the point to bend towards,
         * and the type. */
        {200, 0, 330, 0, 0, 0, FOX_ROAD_AVENUE},
        {330, 0, 330, 90, 0, 0, FOX_ROAD_STREET},
        {330, 0, 330, -90, 0, 0, FOX_ROAD_STREET},
        {330, -90, 260, -140, 0, 0, FOX_ROAD_STREET},
        {330, 0, 430, 50, 0, 0, FOX_ROAD_STREET},
        {330, 0, 430, -30, 0, 0, FOX_ROAD_STREET},
        {430, 50, 430, -30, 510, 10, FOX_ROAD_BOULEVARD},
    };
    for (size_t index = 0u; index < sizeof(district) / sizeof(district[0]); ++index)
    {
        const int16_t *road = district[index];
        const FoxRoadPlan plan = {
            .start = PoleOffset(world, road[0], road[1]),
            .end = PoleOffset(world, road[2], road[3]),
            .control = PoleOffset(world, road[4], road[5]),
            .curved = road[4] != 0,
            .type = (FoxRoadType)road[6],
        };
        ok = FoxWorldBuildRoad(world, &plan) == FOX_ROAD_OK && ok;
    }
    /* Lights where the avenues cross, stop signs where two streets meet. */
    const FoxNetwork *network = world->network;
    const uint32_t centre = FoxNetworkNearestNode(network, Ground(world, 0, 0), FOX_METRES(4));
    ok = centre != FOX_NETWORK_NONE && FoxWorldSetSignals(world, centre, true) && ok;
    const uint32_t corner = FoxNetworkNearestNode(network, Ground(world, -80, -80), FOX_METRES(4));
    FoxNodeView view;
    if (corner != FOX_NETWORK_NONE && FoxNetworkNode(network, corner, &view))
    {
        for (uint32_t slot = 0u; slot < view.roadCount; ++slot)
        {
            FoxRoadView road;
            if (FoxNetworkRoad(network, view.roads[slot], &road))
            {
                const uint32_t end = road.nodes[0] == corner ? 0u : 1u;
                ok = FoxWorldSetSign(world, view.roads[slot], end, FOX_SIGN_STOP) && ok;
            }
        }
    }
    else
    {
        ok = false;
    }
    /* A slow street past the corner. */
    const uint32_t slow =
        FoxNetworkNearestRoad(network, Ground(world, -80, -120), FOX_METRES(4), NULL);
    ok = slow != FOX_NETWORK_NONE && FoxWorldSetSpeedLimit(world, slow, FOX_KMH(30)) && ok;
    return ok;
}

/* sin(degree) in FOX_UNIT for 0..90 degrees, every 3 degrees. */
static const int32_t sines[31] = {
    0,     3430,  6850,  10252, 13626, 16962, 20252, 23486, 26656, 29753, 32768,
    35693, 38521, 41243, 43852, 46341, 48703, 50931, 53020, 54963, 56756, 58393,
    59870, 61183, 62328, 63303, 64104, 64729, 65177, 65446, 65536,
};

/* sin of an angle given in tenths of a degree, a multiple of 3 degrees. */
static int64_t Sine(int32_t tenths)
{
    const int32_t angle = ((tenths % 3600) + 3600) % 3600 / 30;
    if (angle <= 30)
    {
        return sines[angle];
    }
    if (angle <= 60)
    {
        return sines[60 - angle];
    }
    return angle <= 90 ? -sines[angle - 60] : -sines[120 - angle];
}

static int64_t Cosine(int32_t tenths)
{
    return Sine(tenths + 900);
}

/* A point of the planet's surface at a latitude and longitude in tenths of
 * a degree, multiples of 3 degrees: the town's pole is at 90 degrees, and
 * longitude runs from the town's east towards its north. */
static FoxVec OnPlanet(const FoxWorld *world, int32_t latitude, int32_t longitude)
{
    const int64_t radius = FoxPlanetRadius(world->planet);
    const int64_t across = radius * Cosine(latitude) / FOX_UNIT;
    return (FoxVec){across * Cosine(longitude) / FOX_UNIT, across * Sine(longitude) / FOX_UNIT,
                    radius * Sine(latitude) / FOX_UNIT};
}

/* A road all the way round the planet at a latitude, in `pieces` pieces of
 * a multiple of 6 degrees each, the first from the longitude `first`. Each
 * bends towards where the ring's tangents at its ends meet, so the ring
 * runs on through its corners without a kink. */
static bool Ring(FoxWorld *world, int32_t latitude, int32_t first, uint32_t pieces,
                 FoxRoadType type)
{
    bool ok = true;
    const int32_t step = 3600 / (int32_t)pieces;
    const int64_t radius = FoxPlanetRadius(world->planet);
    const int64_t across = radius * Cosine(latitude) / Cosine(step / 2);
    for (uint32_t piece = 0u; piece < pieces; ++piece)
    {
        const int32_t from = first + (int32_t)piece * step;
        const int32_t middle = from + step / 2;
        const FoxRoadPlan plan = {
            .start = OnPlanet(world, latitude, from),
            .end = OnPlanet(world, latitude, from + step),
            .control = {across * Cosine(middle) / FOX_UNIT, across * Sine(middle) / FOX_UNIT,
                        radius * Sine(latitude) / FOX_UNIT},
            .curved = true,
            .type = type,
        };
        ok = FoxWorldBuildRoad(world, &plan) == FOX_ROAD_OK && ok;
    }
    return ok;
}

bool FoxWorldBuildPlanetRoads(FoxWorld *world)
{
    if (world == NULL)
    {
        return false;
    }
    /* Rings of every kind of road at different latitudes: a street, an
     * avenue, a boulevard and a one-way highway, the bigger ones nearer the
     * equator. The street turns its corners where the meridians below cross
     * it, leaving long stretches to overtake on; the corners of the others
     * keep off the meridians. */
    bool ok = Ring(world, 510, 0, 4u, FOX_ROAD_STREET);
    ok = Ring(world, 210, 90, 15u, FOX_ROAD_AVENUE) && ok;
    ok = Ring(world, -90, 90, 15u, FOX_ROAD_BOULEVARD) && ok;
    ok = Ring(world, -390, 90, 15u, FOX_ROAD_HIGHWAY) && ok;
    /* Meridians from the town's avenues down across every ring to the
     * south pole, where they meet. */
    static const struct
    {
        int16_t east;
        int16_t north;
        int16_t longitude;
        uint8_t type;
    } meridians[] = {
        {0, -200, 2700, FOX_ROAD_AVENUE},
        {-200, 0, 1800, FOX_ROAD_STREET},
        {0, 200, 900, FOX_ROAD_BOULEVARD},
    };
    static const int16_t latitudes[] = {660, 360, 60, -240, -540, -900};
    for (size_t index = 0u; index < sizeof(meridians) / sizeof(meridians[0]); ++index)
    {
        FoxVec from = PoleOffset(world, meridians[index].east, meridians[index].north);
        for (size_t step = 0u; step < sizeof(latitudes) / sizeof(latitudes[0]); ++step)
        {
            const FoxVec to = OnPlanet(world, latitudes[step], meridians[index].longitude);
            const FoxRoadPlan plan = {
                .start = from, .end = to, .type = (FoxRoadType)meridians[index].type};
            ok = FoxWorldBuildRoad(world, &plan) == FOX_ROAD_OK && ok;
            from = to;
        }
    }
    /* Lights where roads cross, but for two streets: nobody could get across
     * the traffic of a big road, let alone a highway, without. */
    for (uint32_t index = 0u; index < FoxNetworkNodeCapacity(world->network); ++index)
    {
        FoxNodeView node;
        if (!FoxNetworkNode(world->network, index, &node) || node.roadCount < 4u || node.signals)
        {
            continue;
        }
        uint32_t big = 0u;
        for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
        {
            FoxRoadView road;
            big += FoxNetworkRoad(world->network, node.roads[slot], &road) &&
                           road.type != FOX_ROAD_STREET
                       ? 1u
                       : 0u;
        }
        /* Away from the town, which has its own. */
        if (big != 0u && FoxPlanetUp(node.position).z < FOX_UNIT * 9 / 10)
        {
            ok = FoxWorldSetSignals(world, index, true) && ok;
        }
    }
    return ok;
}

bool FoxWorldBuildGrid(FoxWorld *world, uint32_t lines, int64_t spacing)
{
    if (world == NULL || lines < 2u || spacing < FOX_METRES(20) ||
        (int64_t)(lines - 1u) * spacing > FOX_ROAD_MAX_LENGTH)
    {
        return false;
    }
    const int64_t half = (int64_t)(lines - 1u) * spacing / 2;
    const uint32_t middle = lines / 2u;
    bool ok = true;
    for (uint32_t pass = 0u; pass < 2u; ++pass)
    {
        for (uint32_t line = 0u; line < lines; ++line)
        {
            const int64_t at = (int64_t)line * spacing - half;
            const bool boulevard = line == middle;
            const bool avenue = line % 4u == 0u || line + 1u == lines;
            const FoxRoadPlan plan = {
                .start = pass == 0u ? (FoxVec){at, -half, FoxPlanetRadius(world->planet)}
                                    : (FoxVec){-half, at, FoxPlanetRadius(world->planet)},
                .end = pass == 0u ? (FoxVec){at, half, FoxPlanetRadius(world->planet)}
                                  : (FoxVec){half, at, FoxPlanetRadius(world->planet)},
                .type =
                    boulevard ? FOX_ROAD_BOULEVARD : (avenue ? FOX_ROAD_AVENUE : FOX_ROAD_STREET),
            };
            ok = FoxWorldBuildRoad(world, &plan) == FOX_ROAD_OK && ok;
        }
    }
    /* Lights where two big roads cross. */
    for (uint32_t index = 0u; index < FoxNetworkNodeCapacity(world->network); ++index)
    {
        FoxNodeView node;
        if (!FoxNetworkNode(world->network, index, &node) || node.roadCount < 4u)
        {
            continue;
        }
        uint32_t big = 0u;
        for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
        {
            FoxRoadView road;
            big += FoxNetworkRoad(world->network, node.roads[slot], &road) &&
                           road.type != FOX_ROAD_STREET
                       ? 1u
                       : 0u;
        }
        if (big == node.roadCount)
        {
            ok = FoxWorldSetSignals(world, index, true) && ok;
        }
    }
    return ok;
}

uint64_t FoxWorldHash(const FoxWorld *world)
{
    if (world == NULL)
    {
        return 0u;
    }
    uint64_t hash = FoxHashCombine(world->tick, FoxPlanetHash(world->planet));
    hash = FoxHashCombine(hash, FoxNetworkHash(world->network));
    return FoxHashCombine(hash, FoxTrafficHash(world->traffic));
}
