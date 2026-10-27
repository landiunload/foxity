#include "sim/network_internal.h"

#include "sim/random.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Centre lines are sampled about this often; curves at least this finely. */
#define SAMPLE_SPACING FOX_METRES(4)
#define CURVE_MIN_SAMPLES 9u
#define MAX_CUTS 64u
#define MAX_DIRTY (MAX_CUTS * 4u + 8u)
/* Crossings this close along both roads are the same one. */
#define SAME_CUT FOX_METRES(1)

/* ---- Layouts ---------------------------------------------------------------- */

/* The widest any road reaches from its centre line: six car lanes, a bike
 * lane and a footway. */
#define MAX_HALF_WIDTH (6 * FOX_LANE_WIDTH / 2 + FOX_LANE_WIDTH + FOX_FOOT_WIDTH)
/* Crossings keep this far from a road's ends and from each other. */
#define CROSSING_ROOM FOX_METRES(6)

FoxRoadLayout FoxRoadLayoutPreset(FoxRoadType type)
{
    FoxRoadLayout layout = {.lanes = {1u, 1u}, .foot = {true, true}};
    switch (type)
    {
    case FOX_ROAD_AVENUE:
        layout.lanes[0] = 2u;
        layout.lanes[1] = 2u;
        break;
    case FOX_ROAD_BOULEVARD:
        layout.lanes[0] = 3u;
        layout.lanes[1] = 3u;
        break;
    case FOX_ROAD_ONE_WAY:
        layout = (FoxRoadLayout){.lanes = {2u, 0u}};
        break;
    case FOX_ROAD_HIGHWAY:
        layout = (FoxRoadLayout){.lanes = {3u, 0u}};
        break;
    default:
        break;
    }
    return layout;
}

int32_t FoxRoadPresetSpeed(FoxRoadType type)
{
    static const int32_t speeds[FOX_ROAD_TYPE_COUNT] = {
        [FOX_ROAD_STREET] = FOX_KMH(50),    [FOX_ROAD_AVENUE] = FOX_KMH(60),
        [FOX_ROAD_BOULEVARD] = FOX_KMH(70), [FOX_ROAD_ONE_WAY] = FOX_KMH(50),
        [FOX_ROAD_HIGHWAY] = FOX_KMH(110),
    };
    return (unsigned)type < FOX_ROAD_TYPE_COUNT ? speeds[type] : FOX_KMH(50);
}

FoxRoadLayout FoxRoadLayoutSingle(FoxLaneKind kind)
{
    FoxRoadLayout layout = {0};
    switch (kind)
    {
    case FOX_LANE_BIKE:
        layout.bike[1] = FOX_WAY_FORWARD;
        break;
    case FOX_LANE_FOOT:
        layout.foot[1] = true;
        break;
    default:
        layout.lanes[0] = 1u;
        layout.surface[0][0] =
            (uint8_t)(kind == FOX_LANE_CONCRETE ? FOX_LANE_CONCRETE : FOX_LANE_ASPHALT);
        break;
    }
    return layout;
}

bool FoxRoadLayoutValid(const FoxRoadLayout *layout)
{
    if (layout == NULL || layout->lanes[0] > FOX_ROAD_MAX_LANES ||
        layout->lanes[1] > FOX_ROAD_MAX_LANES ||
        (layout->shared && (layout->lanes[0] != 1u || layout->lanes[1] != 1u)))
    {
        return false;
    }
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < layout->lanes[direction]; ++lane)
        {
            if (layout->surface[direction][lane] > FOX_LANE_CONCRETE)
            {
                return false;
            }
        }
    }
    bool any = layout->lanes[0] + layout->lanes[1] != 0u;
    for (uint32_t side = 0u; side < 2u; ++side)
    {
        if (layout->bike[side] > FOX_WAY_BOTH)
        {
            return false;
        }
        any = any || layout->bike[side] != FOX_WAY_NONE || layout->foot[side];
    }
    return any;
}

uint32_t FoxRoadLayoutCarLanes(const FoxRoadLayout *layout)
{
    return layout->shared ? 1u : (uint32_t)layout->lanes[0] + layout->lanes[1];
}

int64_t FoxRoadLayoutCarHalfWidth(const FoxRoadLayout *layout)
{
    return (int64_t)FoxRoadLayoutCarLanes(layout) * FOX_LANE_WIDTH / 2;
}

/* The width of what runs along one edge. */
static int64_t EdgeWidth(const FoxRoadLayout *layout, uint32_t side)
{
    return (layout->bike[side] != FOX_WAY_NONE ? FOX_LANE_WIDTH : 0) +
           (layout->foot[side] ? FOX_FOOT_WIDTH : 0);
}

void FoxRoadLayoutExtent(const FoxRoadLayout *layout, int64_t outExtent[2])
{
    const int64_t half = FoxRoadLayoutCarHalfWidth(layout);
    if (half == 0)
    {
        /* Paths alone: centred on the centre line. */
        const int64_t total = EdgeWidth(layout, 0u) + EdgeWidth(layout, 1u);
        outExtent[0] = total / 2;
        outExtent[1] = total - total / 2;
        return;
    }
    outExtent[0] = half + EdgeWidth(layout, 0u);
    outExtent[1] = half + EdgeWidth(layout, 1u);
}

int64_t FoxRoadLaneOffset(const FoxRoadLayout *layout, FoxDriveSide side, uint32_t direction,
                          uint32_t lane)
{
    /* Forward lanes keep to the right where traffic keeps right. */
    const bool forwardRight = side == FOX_DRIVE_RIGHT;
    if (layout->shared)
    {
        const bool right = (direction == 0u) == forwardRight;
        return right ? FOX_LANE_WIDTH / 4 : -FOX_LANE_WIDTH / 4;
    }
    const uint32_t count = (uint32_t)layout->lanes[0] + layout->lanes[1];
    /* Slots from the left; the curb lane of each way lies at its edge. */
    const bool onRight = (direction == 0u) == forwardRight;
    const uint32_t slot = onRight ? count - 1u - lane : lane;
    return -(int64_t)count * FOX_LANE_WIDTH / 2 + (int64_t)slot * FOX_LANE_WIDTH +
           FOX_LANE_WIDTH / 2;
}

uint32_t FoxRoadLayoutStrips(const FoxRoadLayout *layout, FoxDriveSide side,
                             FoxRoadStrip outStrips[FOX_ROAD_MAX_STRIPS])
{
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    int64_t x = -extent[0];
    uint32_t count = 0u;
    if (layout->foot[0])
    {
        outStrips[count++] = (FoxRoadStrip){FOX_LANE_FOOT, FOX_WAY_BOTH, x, x + FOX_FOOT_WIDTH};
        x += FOX_FOOT_WIDTH;
    }
    if (layout->bike[0] != FOX_WAY_NONE)
    {
        outStrips[count++] = (FoxRoadStrip){FOX_LANE_BIKE, layout->bike[0], x, x + FOX_LANE_WIDTH};
        x += FOX_LANE_WIDTH;
    }
    if (layout->shared)
    {
        outStrips[count++] =
            (FoxRoadStrip){layout->surface[0][0], FOX_WAY_BOTH, x, x + FOX_LANE_WIDTH};
        x += FOX_LANE_WIDTH;
    }
    else
    {
        const uint32_t lanes = (uint32_t)layout->lanes[0] + layout->lanes[1];
        for (uint32_t slot = 0u; slot < lanes; ++slot)
        {
            /* Which way and which lane of it lies in this slot. */
            uint32_t direction = 0u;
            uint32_t lane = 0u;
            for (uint32_t way = 0u; way < 2u; ++way)
            {
                for (uint32_t index = 0u; index < layout->lanes[way]; ++index)
                {
                    const int64_t centre = FoxRoadLaneOffset(layout, side, way, index);
                    if (centre > x && centre < x + FOX_LANE_WIDTH)
                    {
                        direction = way;
                        lane = index;
                    }
                }
            }
            outStrips[count++] =
                (FoxRoadStrip){layout->surface[direction][lane],
                               (uint8_t)(direction == 0u ? FOX_WAY_FORWARD : FOX_WAY_BACKWARD), x,
                               x + FOX_LANE_WIDTH};
            x += FOX_LANE_WIDTH;
        }
    }
    if (layout->bike[1] != FOX_WAY_NONE)
    {
        outStrips[count++] = (FoxRoadStrip){FOX_LANE_BIKE, layout->bike[1], x, x + FOX_LANE_WIDTH};
        x += FOX_LANE_WIDTH;
    }
    if (layout->foot[1])
    {
        outStrips[count++] = (FoxRoadStrip){FOX_LANE_FOOT, FOX_WAY_BOTH, x, x + FOX_FOOT_WIDTH};
    }
    return count;
}

int32_t FoxRoadLayoutSpeed(const FoxRoadLayout *layout)
{
    const uint32_t lanes = FoxRoadLayoutCarLanes(layout);
    if (lanes == 0u)
    {
        return FOX_KMH(20);
    }
    if (layout->shared)
    {
        return FOX_KMH(30);
    }
    return lanes <= 2u ? FOX_KMH(50) : (lanes <= 4u ? FOX_KMH(60) : FOX_KMH(70));
}

/* Folds every field of a layout into one value. */
static uint64_t LayoutHash(const FoxRoadLayout *layout)
{
    uint64_t hash = FoxHashCombine(layout->lanes[0], layout->lanes[1]);
    hash = FoxHashCombine(hash, layout->shared ? 1u : 0u);
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < FOX_ROAD_MAX_LANES; ++lane)
        {
            hash = FoxHashCombine(hash, layout->surface[direction][lane]);
        }
        hash = FoxHashCombine(hash, layout->bike[direction]);
        hash = FoxHashCombine(hash, layout->foot[direction] ? 1u : 0u);
    }
    return hash;
}

uint32_t FoxNodeCarRoads(const FoxNetwork *network, uint32_t node)
{
    const FoxNode *record = &network->nodes[node];
    uint32_t count = 0u;
    for (uint32_t slot = 0u; slot < record->roadCount; ++slot)
    {
        count += FoxRoadLayoutCarLanes(&network->roads[record->roads[slot]].layout) != 0u ? 1u : 0u;
    }
    return count;
}

int32_t FoxRoadOwnSpeed(const FoxRoad *road)
{
    return road->builtSpeed != 0 ? road->builtSpeed : FoxRoadLayoutSpeed(&road->layout);
}

/* Half the width roads meet by at a junction: the car lanes, or a path's
 * strips where it has none. Footways and bike lanes beside the car lanes run
 * round the corners. */
static int64_t CoreHalf(const FoxRoadLayout *layout)
{
    const int64_t cars = FoxRoadLayoutCarHalfWidth(layout);
    if (cars != 0)
    {
        return cars;
    }
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    return FoxMax(extent[0], extent[1]);
}

int64_t FoxRoadReach(const FoxRoad *road)
{
    int64_t extent[2];
    FoxRoadLayoutExtent(&road->driven, extent);
    return FoxMax(extent[0], extent[1]) + FOX_ROAD_SHOULDER;
}

/* ---- Polylines -------------------------------------------------------------- */

int64_t FoxPolylineMeasure(const FoxVec *points, int64_t *lengths, uint32_t count)
{
    int64_t total = 0;
    for (uint32_t index = 0u; index < count; ++index)
    {
        if (index != 0u)
        {
            total += FoxVecDistance(points[index - 1u], points[index]);
        }
        lengths[index] = total;
    }
    return total;
}

FoxVec FoxPolylinePoint(const FoxVec *points, const int64_t *lengths, uint32_t count,
                        int64_t distance, uint32_t *outIndex)
{
    if (count == 0u)
    {
        if (outIndex != NULL)
        {
            *outIndex = 0u;
        }
        return (FoxVec){0, 0, 0};
    }
    if (count == 1u || distance <= 0)
    {
        if (outIndex != NULL)
        {
            *outIndex = 0u;
        }
        return points[0];
    }
    if (distance >= lengths[count - 1u])
    {
        if (outIndex != NULL)
        {
            *outIndex = count - 2u;
        }
        return points[count - 1u];
    }
    uint32_t low = 0u;
    uint32_t high = count - 1u;
    while (high - low > 1u)
    {
        const uint32_t middle = low + (high - low) / 2u;
        if (lengths[middle] <= distance)
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    if (outIndex != NULL)
    {
        *outIndex = low;
    }
    const int64_t span = lengths[high] - lengths[low];
    if (span <= 0)
    {
        return points[low];
    }
    const int64_t t = (distance - lengths[low]) * FOX_UNIT / span;
    return FoxVecLerp(points[low], points[high], t);
}

/* ---- Storage ---------------------------------------------------------------- */

FoxNetwork *FoxNetworkCreate(const FoxNetworkDesc *desc)
{
    if (desc == NULL || (desc->driveSide != FOX_DRIVE_RIGHT && desc->driveSide != FOX_DRIVE_LEFT))
    {
        return NULL;
    }
    FoxNetwork *network = calloc(1u, sizeof(*network));
    if (network == NULL)
    {
        return NULL;
    }
    network->defaultSide = (uint8_t)desc->driveSide;
    network->router = desc->router;
    return network;
}

static void FreePathArrays(FoxPath *path)
{
    free(path->points);
    free(path->lengths);
    free(path->centre);
    free(path->outgoing);
    free(path->conflicts);
    free(path->siblings);
    path->points = NULL;
    path->lengths = NULL;
    path->centre = NULL;
    path->outgoing = NULL;
    path->conflicts = NULL;
    path->siblings = NULL;
}

void FoxNetworkDestroy(FoxNetwork *network)
{
    if (network == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        FreePathArrays(&network->paths[index]);
    }
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        free(network->roads[index].centre);
        free(network->roads[index].centreLengths);
    }
    for (uint32_t index = 0u; index < network->nodeCapacity; ++index)
    {
        free(network->nodes[index].connections);
        free(network->nodes[index].corners);
    }
    free(network->paths);
    free(network->roads);
    free(network->nodes);
    free(network->retired);
    free(network->lineage);
    free(network);
}

static bool Grow(void **array, uint32_t *capacity, uint32_t needed, size_t elementSize)
{
    if (needed <= *capacity)
    {
        return true;
    }
    uint32_t grown = *capacity < 16u ? 16u : *capacity;
    while (grown < needed)
    {
        if (grown > UINT32_MAX / 2u)
        {
            return false;
        }
        grown *= 2u;
    }
    void *bigger = realloc(*array, (size_t)grown * elementSize);
    if (bigger == NULL)
    {
        return false;
    }
    memset((uint8_t *)bigger + (size_t)*capacity * elementSize, 0,
           (size_t)(grown - *capacity) * elementSize);
    *array = bigger;
    *capacity = grown;
    return true;
}

uint32_t FoxNetworkAllocatePath(FoxNetwork *network)
{
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        const FoxPath *path = &network->paths[index];
        if (path->kind == 0u && !path->retired)
        {
            return index;
        }
    }
    const uint32_t index = network->pathCapacity;
    if (!Grow((void **)&network->paths, &network->pathCapacity, index + 1u,
              sizeof(*network->paths)))
    {
        return FOX_NETWORK_NONE;
    }
    return index;
}

bool FoxNetworkRetirePath(FoxNetwork *network, uint32_t path)
{
    if (path >= network->pathCapacity || network->paths[path].kind == 0u ||
        network->paths[path].retired)
    {
        return true;
    }
    if (!Grow((void **)&network->retired, &network->retiredCapacity, network->retiredCount + 1u,
              sizeof(*network->retired)))
    {
        return false;
    }
    network->paths[path].retired = true;
    network->retired[network->retiredCount++] = path;
    return true;
}

void FoxNetworkReleaseRetired(FoxNetwork *network)
{
    if (network == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < network->retiredCount; ++index)
    {
        FoxPath *path = &network->paths[network->retired[index]];
        const uint32_t generation = path->generation + 1u;
        FreePathArrays(path);
        memset(path, 0, sizeof(*path));
        path->generation = generation;
    }
    network->retiredCount = 0u;
    network->lineageCount = 0u;
}

const FoxPath *FoxNetworkPathRecord(const FoxNetwork *network, uint32_t path)
{
    return network != NULL && path < network->pathCapacity && network->paths[path].kind != 0u
               ? &network->paths[path]
               : NULL;
}

FoxPath *FoxNetworkPathRecordMutable(FoxNetwork *network, uint32_t path)
{
    return network != NULL && path < network->pathCapacity && network->paths[path].kind != 0u
               ? &network->paths[path]
               : NULL;
}

bool FoxNetworkPathLive(const FoxNetwork *network, uint32_t path)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    return record != NULL && !record->retired;
}

/* ---- Views ------------------------------------------------------------------ */

uint64_t FoxNetworkRevision(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->revision;
}

FoxDriveSide FoxNetworkDefaultDriveSide(const FoxNetwork *network)
{
    return network == NULL ? FOX_DRIVE_RIGHT : (FoxDriveSide)network->defaultSide;
}

FoxDriveSide FoxNetworkDriveSideAt(const FoxNetwork *network, FoxVec point)
{
    if (network == NULL)
    {
        return FOX_DRIVE_RIGHT;
    }
    for (uint32_t index = network->zoneCount; index > 0u; --index)
    {
        const FoxDriveZone *zone = &network->zones[index - 1u];
        if (FoxVecDistance(point, zone->center) <= zone->radius)
        {
            return (FoxDriveSide)zone->side;
        }
    }
    return (FoxDriveSide)network->defaultSide;
}

uint32_t FoxNetworkDriveZoneCount(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->zoneCount;
}

uint32_t FoxNetworkRoadCapacity(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->roadCapacity;
}

uint32_t FoxNetworkNodeCapacity(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->nodeCapacity;
}

uint32_t FoxNetworkRoadCount(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->roadCount - network->closingCount;
}

uint32_t FoxNetworkNodeCount(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->nodeCount;
}

bool FoxNetworkRoad(const FoxNetwork *network, uint32_t road, FoxRoadView *outView)
{
    if (network == NULL || outView == NULL || road >= network->roadCapacity ||
        !network->roads[road].alive || network->roads[road].closing)
    {
        return false;
    }
    const FoxRoad *record = &network->roads[road];
    *outView = (FoxRoadView){
        .nodes = {record->nodes[0], record->nodes[1]},
        .layout = record->layout,
        .driveSide = (FoxDriveSide)record->driveSide,
        .centre = record->centre,
        .centreCount = record->centreCount,
        .length = record->length,
        .trim = {record->trim[0], record->trim[1]},
        .laneCount = {record->laneCount[0], record->laneCount[1]},
        .speedLimit = record->speedLimit,
        .speedSet = record->speedSetting != 0,
        .sign = {(FoxSign)record->sign[0], (FoxSign)record->sign[1]},
        .signSetting = {(FoxSign)record->signSetting[0], (FoxSign)record->signSetting[1]},
        .crossingCount = record->crossingCount,
        .revision = record->revision,
    };
    memcpy(outView->lanes, record->lanes, sizeof(outView->lanes));
    memcpy(outView->crossings, record->crossings, sizeof(outView->crossings));
    return true;
}

bool FoxNetworkNode(const FoxNetwork *network, uint32_t node, FoxNodeView *outView)
{
    if (network == NULL || outView == NULL || node >= network->nodeCapacity ||
        !network->nodes[node].alive)
    {
        return false;
    }
    const FoxNode *record = &network->nodes[node];
    *outView = (FoxNodeView){
        .position = record->position,
        .radius = record->radius,
        .signals = record->signals,
        .revision = record->revision,
    };
    /* A closing road is no longer there to see. */
    for (uint32_t slot = 0u; slot < record->roadCount; ++slot)
    {
        const FoxRoad *road = &network->roads[record->roads[slot]];
        if (!road->closing)
        {
            outView->roads[outView->roadCount++] = record->roads[slot];
            outView->carRoadCount += FoxRoadLayoutCarLanes(&road->layout) != 0u ? 1u : 0u;
        }
    }
    return outView->roadCount != 0u || record->roadCount == 0u;
}

bool FoxNetworkPath(const FoxNetwork *network, uint32_t path, FoxPathView *outView)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    if (record == NULL || outView == NULL)
    {
        return false;
    }
    *outView = (FoxPathView){
        .connection = record->kind == PATH_CONNECTION,
        .points = record->points,
        .lengths = record->lengths,
        .count = record->count,
        .length = record->length,
        .speedLimit = record->speedLimit,
    };
    return true;
}

bool FoxNetworkPathPoint(const FoxNetwork *network, uint32_t path, int64_t distance,
                         FoxVec *outPoint, FoxVec *outDirection)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    if (record == NULL || record->count < 2u)
    {
        return false;
    }
    uint32_t index = 0u;
    const FoxVec point =
        FoxPolylinePoint(record->points, record->lengths, record->count, distance, &index);
    if (outPoint != NULL)
    {
        *outPoint = point;
    }
    if (outDirection != NULL)
    {
        FoxVec direction =
            FoxVecDirection(FoxVecSub(record->points[index + 1u], record->points[index]));
        if (FoxVecIsZero(direction))
        {
            direction =
                FoxVecDirection(FoxVecSub(record->points[record->count - 1u], record->points[0]));
        }
        *outDirection = direction;
    }
    return true;
}

/* ---- Picking ---------------------------------------------------------------- */

uint32_t FoxNetworkNearestNode(const FoxNetwork *network, FoxVec point, int64_t maxDistance)
{
    uint32_t best = FOX_NETWORK_NONE;
    int64_t bestDistance = maxDistance;
    for (uint32_t index = 0u; network != NULL && index < network->nodeCapacity; ++index)
    {
        const FoxNode *node = &network->nodes[index];
        if (!node->alive)
        {
            continue;
        }
        const int64_t distance = FoxVecDistance(point, node->position);
        if (distance <= bestDistance)
        {
            best = index;
            bestDistance = distance;
        }
    }
    return best;
}

/* Distance along a segment to the point nearest `point`, and that distance. */
static int64_t ClosestOnSegment(FoxVec a, FoxVec b, FoxVec point, int64_t *outDistance)
{
    const FoxVec ab = FoxVecSub(b, a);
    const int64_t lengthSquared = FoxVecDot(ab, ab);
    int64_t t = 0;
    if (lengthSquared > 0)
    {
        t = FoxClamp(FoxVecDot(FoxVecSub(point, a), ab) * 1024 / lengthSquared, 0, 1024);
    }
    const FoxVec closest = FoxVecAdd(a, FoxVecScale(ab, t, 1024));
    *outDistance = FoxVecDistance(closest, point);
    return FoxVecDistance(a, closest);
}

uint32_t FoxNetworkNearestRoad(const FoxNetwork *network, FoxVec point, int64_t maxDistance,
                               int64_t *outAlong)
{
    uint32_t best = FOX_NETWORK_NONE;
    int64_t bestDistance = maxDistance;
    for (uint32_t index = 0u; network != NULL && index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (!road->alive || road->closing ||
            FoxVecDistance(point, road->middle) > road->boundRadius + maxDistance)
        {
            continue;
        }
        for (uint32_t segment = 0u; segment + 1u < road->centreCount; ++segment)
        {
            int64_t distance = 0;
            const int64_t along = ClosestOnSegment(road->centre[segment],
                                                   road->centre[segment + 1u], point, &distance);
            if (distance <= bestDistance)
            {
                best = index;
                bestDistance = distance;
                if (outAlong != NULL)
                {
                    *outAlong = road->centreLengths[segment] + along;
                }
            }
        }
    }
    return best;
}

/* ---- Ground under the roads ------------------------------------------------- */

/* Kept across, beyond the paved edge, and how far under and over the road:
 * over it as high as any edit reaches, so nothing can bury it. */
#define GUARD_MARGIN (FOX_METRES(3) / 2)
#define GUARD_BELOW FOX_METRES(4)
#define GUARD_ABOVE (2 * FOX_PLANET_EDIT_MAX_RADIUS)
/* No kept point lies further than this from the paved centre line or edge. */
#define GUARD_REACH (GUARD_MARGIN + GUARD_ABOVE + GUARD_BELOW)

/* Whether `offset` from a point on the ground lies within `across` of it
 * sideways and within the guarded band up and down. */
static bool WithinBand(FoxVec offset, FoxVec ground, int64_t across)
{
    const FoxVec up = FoxVecDirection(ground);
    const int64_t height = FoxVecDot(offset, up) / FOX_UNIT;
    if (height < -GUARD_BELOW || height > GUARD_ABOVE)
    {
        return false;
    }
    const int64_t sideways = FoxVecDot(offset, offset) - height * height;
    return sideways <= across * across;
}

void FoxNetworkGuardGround(const FoxNetwork *network, FoxVec center, int64_t radius,
                           FoxGroundGuard *outGuard)
{
    if (outGuard == NULL)
    {
        return;
    }
    *outGuard = (FoxGroundGuard){.network = network};
    for (uint32_t index = 0u; network != NULL && index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (road->alive && outGuard->roadCount < FOX_GROUND_GUARD_MAX &&
            FoxVecDistance(center, road->middle) <=
                radius + road->boundRadius + FoxRoadReach(road) + GUARD_REACH)
        {
            outGuard->roads[outGuard->roadCount++] = index;
        }
    }
    for (uint32_t index = 0u; network != NULL && index < network->nodeCapacity; ++index)
    {
        const FoxNode *node = &network->nodes[index];
        if (node->alive && outGuard->nodeCount < FOX_GROUND_GUARD_MAX &&
            FoxVecDistance(center, node->position) <= radius + node->radius + GUARD_REACH)
        {
            outGuard->nodes[outGuard->nodeCount++] = index;
        }
    }
}

bool FoxGroundGuardKeeps(const FoxGroundGuard *guard, FoxVec point)
{
    if (guard == NULL || guard->network == NULL)
    {
        return false;
    }
    const FoxNetwork *network = guard->network;
    for (uint32_t index = 0u; index < guard->nodeCount; ++index)
    {
        const FoxNode *node = &network->nodes[guard->nodes[index]];
        if (WithinBand(FoxVecSub(point, node->position), node->position,
                       node->radius + GUARD_MARGIN))
        {
            return true;
        }
    }
    for (uint32_t index = 0u; index < guard->roadCount; ++index)
    {
        const FoxRoad *road = &network->roads[guard->roads[index]];
        const int64_t across = FoxRoadReach(road) + GUARD_MARGIN;
        if (FoxVecDistance(point, road->middle) > road->boundRadius + across + GUARD_REACH)
        {
            continue;
        }
        for (uint32_t segment = 0u; segment + 1u < road->centreCount; ++segment)
        {
            const FoxVec a = road->centre[segment];
            const FoxVec ab = FoxVecSub(road->centre[segment + 1u], a);
            const int64_t lengthSquared = FoxVecDot(ab, ab);
            const int64_t t =
                lengthSquared > 0
                    ? FoxClamp(FoxVecDot(FoxVecSub(point, a), ab) * 1024 / lengthSquared, 0, 1024)
                    : 0;
            const FoxVec closest = FoxVecAdd(a, FoxVecScale(ab, t, 1024));
            if (WithinBand(FoxVecSub(point, closest), closest, across))
            {
                return true;
            }
        }
    }
    return false;
}

bool FoxGroundGuardTouches(const FoxGroundGuard *guard, FoxVec min, FoxVec max)
{
    if (guard == NULL || guard->network == NULL)
    {
        return false;
    }
    const FoxNetwork *network = guard->network;
    const FoxVec middle = {min.x / 2 + max.x / 2, min.y / 2 + max.y / 2, min.z / 2 + max.z / 2};
    const int64_t half = FoxVecDistance(min, max) / 2 + 1;
    for (uint32_t index = 0u; index < guard->nodeCount; ++index)
    {
        const FoxNode *node = &network->nodes[guard->nodes[index]];
        if (FoxVecDistance(middle, node->position) <= half + node->radius + GUARD_REACH)
        {
            return true;
        }
    }
    for (uint32_t index = 0u; index < guard->roadCount; ++index)
    {
        const FoxRoad *road = &network->roads[guard->roads[index]];
        if (FoxVecDistance(middle, road->middle) <=
            half + road->boundRadius + FoxRoadReach(road) + GUARD_REACH)
        {
            return true;
        }
    }
    return false;
}

/* ---- Planning --------------------------------------------------------------- */

uint32_t FoxNetworkPlanCentre(const FoxPlanet *planet, const FoxRoadPlan *plan, FoxVec *points,
                              uint32_t capacity)
{
    if (planet == NULL || plan == NULL || points == NULL || capacity < 2u ||
        !FoxRoadLayoutValid(&plan->layout) || FoxVecEqual(plan->start, plan->end))
    {
        return 0u;
    }
    const int64_t limit = INT64_C(1) << 31;
    const FoxVec ends[3] = {plan->start, plan->end, plan->control};
    for (uint32_t index = 0u; index < 3u; ++index)
    {
        if (FoxAbs(ends[index].x) >= limit || FoxAbs(ends[index].y) >= limit ||
            FoxAbs(ends[index].z) >= limit)
        {
            return 0u;
        }
    }
    const FoxVec control =
        plan->curved ? plan->control : FoxVecLerp(plan->start, plan->end, FOX_UNIT / 2);
    const int64_t estimate =
        FoxVecDistance(plan->start, control) + FoxVecDistance(control, plan->end);
    if (estimate > 4 * FOX_ROAD_MAX_LENGTH)
    {
        return 0u;
    }
    uint64_t count = (uint64_t)(estimate / SAMPLE_SPACING) + 2u;
    if (plan->curved && count < CURVE_MIN_SAMPLES)
    {
        count = CURVE_MIN_SAMPLES;
    }
    if (count > capacity)
    {
        count = capacity;
    }
    for (uint32_t index = 0u; index < (uint32_t)count; ++index)
    {
        const int64_t t = (int64_t)index * FOX_UNIT / (int64_t)(count - 1u);
        const FoxVec first = FoxVecLerp(plan->start, control, t);
        const FoxVec second = FoxVecLerp(control, plan->end, t);
        const FoxVec onCurve = FoxVecLerp(first, second, t);
        if (!FoxPlanetGround(planet, onCurve, &points[index]))
        {
            return 0u;
        }
    }
    return (uint32_t)count;
}

/* ---- Roads and nodes ------------------------------------------------------- */

static void Touch(FoxNetwork *network)
{
    ++network->revision;
}

static uint32_t NewNode(FoxNetwork *network, FoxVec position)
{
    uint32_t index = 0u;
    while (index < network->nodeCapacity && network->nodes[index].alive)
    {
        ++index;
    }
    if (index == network->nodeCapacity && !Grow((void **)&network->nodes, &network->nodeCapacity,
                                                index + 1u, sizeof(*network->nodes)))
    {
        return FOX_NETWORK_NONE;
    }
    FoxNode *node = &network->nodes[index];
    const uint32_t generation = node->generation;
    free(node->connections);
    *node = (FoxNode){
        .alive = true,
        .generation = generation,
        .position = position,
        .up = FoxPlanetUp(position),
        .driveSide = (uint8_t)FoxNetworkDriveSideAt(network, position),
        .revision = network->revision + 1u,
    };
    ++network->nodeCount;
    if (network->router != NULL)
    {
        network->router->setNode(network->router->context, index, position, true);
    }
    return index;
}

static uint32_t TravelCost(int64_t length, int32_t speed)
{
    if (speed <= 0)
    {
        return UINT32_MAX - 1u;
    }
    const int64_t cost = length * 1000 / speed;
    return (uint32_t)FoxClamp(cost, 1, UINT32_MAX - 1u);
}

static void RouteRoad(FoxNetwork *network, uint32_t index, bool alive)
{
    if (network->router == NULL)
    {
        return;
    }
    FoxRoad *road = &network->roads[index];
    /* Only the lanes it was built with: a way back out of a dead end takes
     * nobody in. */
    const uint8_t *lanes = road->layout.lanes;
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        if (lanes[direction] == 0u)
        {
            continue;
        }
        const uint32_t from = road->nodes[direction];
        const uint32_t to = road->nodes[1u - direction];
        const uint32_t cost = alive ? TravelCost(road->length, road->speedLimit) : 0u;
        road->routedCost[direction] = cost;
        network->router->setEdge(network->router->context, index * 2u + direction, from, to, cost);
    }
}

static bool AttachRoad(FoxNode *node, uint32_t road)
{
    if (node->roadCount >= FOX_NODE_MAX_ROADS)
    {
        return false;
    }
    node->roads[node->roadCount++] = road;
    return true;
}

static void DetachRoad(FoxNode *node, uint32_t road)
{
    for (uint32_t index = 0u; index < node->roadCount; ++index)
    {
        if (node->roads[index] == road)
        {
            node->roads[index] = node->roads[--node->roadCount];
            return;
        }
    }
}

/* Creates a road over a copy of the points; its lanes come later. */
static uint32_t NewRoad(FoxNetwork *network, uint32_t from, uint32_t to,
                        const FoxRoadLayout *layout, int32_t speed, const FoxVec *points,
                        uint32_t count)
{
    uint32_t index = 0u;
    while (index < network->roadCapacity && network->roads[index].alive)
    {
        ++index;
    }
    if (index == network->roadCapacity && !Grow((void **)&network->roads, &network->roadCapacity,
                                                index + 1u, sizeof(*network->roads)))
    {
        return FOX_NETWORK_NONE;
    }
    FoxVec *centre = malloc((size_t)count * sizeof(*centre));
    int64_t *lengths = malloc((size_t)count * sizeof(*lengths));
    if (centre == NULL || lengths == NULL)
    {
        free(centre);
        free(lengths);
        return FOX_NETWORK_NONE;
    }
    memcpy(centre, points, (size_t)count * sizeof(*centre));
    FoxRoad *road = &network->roads[index];
    const uint32_t generation = road->generation;
    free(road->centre);
    free(road->centreLengths);
    *road = (FoxRoad){
        .alive = true,
        .generation = generation,
        .nodes = {from, to},
        .layout = *layout,
        .driven = *layout,
        .builtSpeed = speed,
        .centre = centre,
        .centreLengths = lengths,
        .centreCount = count,
        .revision = network->revision + 1u,
    };
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < FOX_ROAD_MAX_LANES; ++lane)
        {
            road->lanes[direction][lane] = FOX_NETWORK_NONE;
        }
    }
    for (uint32_t crossing = 0u; crossing < FOX_ROAD_MAX_CROSSINGS; ++crossing)
    {
        road->crossingWalks[crossing] = FOX_NETWORK_NONE;
    }
    road->length = FoxPolylineMeasure(centre, lengths, count);
    road->middle = FoxPolylinePoint(centre, lengths, count, road->length / 2, NULL);
    for (uint32_t point = 0u; point < count; ++point)
    {
        road->boundRadius = FoxMax(road->boundRadius, FoxVecDistance(road->middle, centre[point]));
    }
    road->driveSide = (uint8_t)FoxNetworkDriveSideAt(network, road->middle);
    road->speedLimit = FoxRoadOwnSpeed(road);
    road->averageSpeed[0] = road->speedLimit;
    road->averageSpeed[1] = road->speedLimit;
    (void)AttachRoad(&network->nodes[from], index);
    (void)AttachRoad(&network->nodes[to], index);
    ++network->roadCount;
    RouteRoad(network, index, true);
    return index;
}

static void RetireRoadLanes(FoxNetwork *network, FoxRoad *road)
{
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
        {
            (void)FoxNetworkRetirePath(network, road->lanes[direction][lane]);
            road->lanes[direction][lane] = FOX_NETWORK_NONE;
        }
        road->laneCount[direction] = 0u;
    }
    FoxLanesRetireWalks(network, road);
}

static void DeleteRoad(FoxNetwork *network, uint32_t index)
{
    FoxRoad *road = &network->roads[index];
    RouteRoad(network, index, false);
    RetireRoadLanes(network, road);
    DetachRoad(&network->nodes[road->nodes[0]], index);
    DetachRoad(&network->nodes[road->nodes[1]], index);
    free(road->centre);
    free(road->centreLengths);
    road->centre = NULL;
    road->centreLengths = NULL;
    road->alive = false;
    ++road->generation;
    --network->roadCount;
}

static void DeleteNode(FoxNetwork *network, uint32_t index)
{
    FoxNode *node = &network->nodes[index];
    for (uint32_t connection = 0u; connection < node->connectionCount; ++connection)
    {
        (void)FoxNetworkRetirePath(network, node->connections[connection]);
    }
    free(node->connections);
    node->connections = NULL;
    node->connectionCount = 0u;
    for (uint32_t corner = 0u; corner < node->cornerCount; ++corner)
    {
        (void)FoxNetworkRetirePath(network, node->corners[corner]);
    }
    free(node->corners);
    node->corners = NULL;
    node->cornerCount = 0u;
    node->alive = false;
    ++node->generation;
    --network->nodeCount;
    if (network->router != NULL)
    {
        network->router->setNode(network->router->context, index, node->position, false);
    }
}

static bool RecordLineage(FoxNetwork *network, FoxLineage entry)
{
    if (!Grow((void **)&network->lineage, &network->lineageCapacity, network->lineageCount + 1u,
              sizeof(*network->lineage)))
    {
        return false;
    }
    network->lineage[network->lineageCount++] = entry;
    return true;
}

/* Splits a road at a distance along its centre line into two roads meeting
 * at a new node; the node, or FOX_NETWORK_NONE without memory. */
static uint32_t SplitRoad(FoxNetwork *network, uint32_t index, int64_t along)
{
    FoxRoad *road = &network->roads[index];
    uint32_t segment = 0u;
    const FoxVec cut =
        FoxPolylinePoint(road->centre, road->centreLengths, road->centreCount, along, &segment);
    /* A road has at least two points and the cut lies on one of its segments. */
    if (road->centreCount < 2u)
    {
        return FOX_NETWORK_NONE;
    }
    segment = segment + 2u > road->centreCount ? road->centreCount - 2u : segment;
    FoxVec *points = malloc((size_t)(road->centreCount + 1u) * sizeof(*points));
    if (points == NULL)
    {
        return FOX_NETWORK_NONE;
    }
    const uint32_t node = NewNode(network, cut);
    if (node == FOX_NETWORK_NONE)
    {
        free(points);
        return FOX_NETWORK_NONE;
    }
    road = &network->roads[index];
    const uint32_t from = road->nodes[0];
    const uint32_t to = road->nodes[1];
    const FoxRoadLayout layout = road->layout;
    const int32_t builtSpeed = road->builtSpeed;
    int64_t crossings[FOX_ROAD_MAX_CROSSINGS];
    const uint32_t crossingCount = road->crossingCount;
    memcpy(crossings, road->crossings, sizeof(crossings));
    const uint32_t generation = road->generation;
    const int64_t length = road->length;
    const int32_t speedSetting = road->speedSetting;
    const uint8_t signs[2] = {road->signSetting[0], road->signSetting[1]};

    /* First part: points up to the cut, then the cut. */
    uint32_t firstCount = 0u;
    for (uint32_t point = 0u; point <= segment; ++point)
    {
        points[firstCount++] = road->centre[point];
    }
    if (!FoxVecEqual(points[firstCount - 1u], cut))
    {
        points[firstCount++] = cut;
    }
    /* Second part: the cut, then the points after it. */
    FoxVec *secondBuffer = malloc((size_t)(road->centreCount + 1u) * sizeof(*secondBuffer));
    if (secondBuffer == NULL)
    {
        free(points);
        DeleteNode(network, node);
        return FOX_NETWORK_NONE;
    }
    uint32_t secondCount = 0u;
    secondBuffer[secondCount++] = cut;
    for (uint32_t point = segment + 1u; point < road->centreCount; ++point)
    {
        if (!FoxVecEqual(road->centre[point], cut))
        {
            secondBuffer[secondCount++] = road->centre[point];
        }
    }
    DeleteRoad(network, index);
    const uint32_t first = NewRoad(network, from, node, &layout, builtSpeed, points, firstCount);
    const uint32_t last =
        first == FOX_NETWORK_NONE
            ? FOX_NETWORK_NONE
            : NewRoad(network, node, to, &layout, builtSpeed, secondBuffer, secondCount);
    free(points);
    free(secondBuffer);
    if (first == FOX_NETWORK_NONE || last == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
    }
    /* The pieces keep the limit and the signs at the old ends; the new
     * junction starts with the network's own signs. */
    const uint32_t pieces[2] = {first, last};
    for (uint32_t part = 0u; part < 2u; ++part)
    {
        FoxRoad *piece = &network->roads[pieces[part]];
        piece->speedSetting = speedSetting;
        if (speedSetting != 0)
        {
            piece->speedLimit = speedSetting;
        }
        piece->signSetting[part] = signs[part];
        RouteRoad(network, pieces[part], true);
    }
    /* The crossings go with their piece, but none right at the new junction. */
    for (uint32_t crossing = 0u; crossing < crossingCount; ++crossing)
    {
        const int64_t at = crossings[crossing];
        FoxRoad *piece = &network->roads[at < along ? first : last];
        const int64_t local = at < along ? at : at - along;
        if (FoxAbs(at - along) >= CROSSING_ROOM && piece->crossingCount < FOX_ROAD_MAX_CROSSINGS)
        {
            piece->crossings[piece->crossingCount++] = local;
        }
    }
    (void)RecordLineage(network, (FoxLineage){.oldRoad = index,
                                              .oldGeneration = generation,
                                              .road = first,
                                              .generation = network->roads[first].generation,
                                              .start = 0,
                                              .end = along,
                                              .offset = 0});
    (void)RecordLineage(network, (FoxLineage){.oldRoad = index,
                                              .oldGeneration = generation,
                                              .road = last,
                                              .generation = network->roads[last].generation,
                                              .start = along,
                                              .end = length + 1,
                                              .offset = -along});
    return node;
}

typedef struct DirtySet
{
    uint32_t nodes[MAX_DIRTY];
    uint32_t count;
} DirtySet;

static void MarkDirty(DirtySet *set, uint32_t node)
{
    for (uint32_t index = 0u; index < set->count; ++index)
    {
        if (set->nodes[index] == node)
        {
            return;
        }
    }
    if (set->count < MAX_DIRTY)
    {
        set->nodes[set->count++] = node;
    }
}

/* Re-measures dirty nodes, rebuilds the lanes of their roads and the
 * connections at both ends of those roads. */
static bool Refresh(FoxNetwork *network, const DirtySet *dirty)
{
    bool ok = true;
    DirtySet connections = {0};
    uint32_t rebuilt[MAX_DIRTY * FOX_NODE_MAX_ROADS];
    uint32_t rebuiltCount = 0u;
    for (uint32_t index = 0u; index < dirty->count; ++index)
    {
        const uint32_t node = dirty->nodes[index];
        MarkDirty(&connections, node);
        if (!network->nodes[node].alive)
        {
            continue;
        }
        FoxLanesMeasureNode(network, node);
        for (uint32_t slot = 0u; slot < network->nodes[node].roadCount; ++slot)
        {
            const uint32_t road = network->nodes[node].roads[slot];
            bool seen = false;
            for (uint32_t other = 0u; other < rebuiltCount && !seen; ++other)
            {
                seen = rebuilt[other] == road;
            }
            if (!seen)
            {
                rebuilt[rebuiltCount++] = road;
            }
        }
    }
    for (uint32_t index = 0u; index < rebuiltCount; ++index)
    {
        ok = FoxLanesBuildRoad(network, rebuilt[index]) && ok;
        network->roads[rebuilt[index]].revision = network->revision + 1u;
        MarkDirty(&connections, network->roads[rebuilt[index]].nodes[0]);
        MarkDirty(&connections, network->roads[rebuilt[index]].nodes[1]);
    }
    for (uint32_t index = 0u; index < connections.count; ++index)
    {
        const uint32_t node = connections.nodes[index];
        if (network->nodes[node].alive)
        {
            ok = FoxLanesBuildNode(network, node) && ok;
            network->nodes[node].revision = network->revision + 1u;
        }
    }
    Touch(network);
    return ok;
}

static bool Joined(const FoxNetwork *network, uint32_t a, uint32_t b)
{
    const FoxNode *node = &network->nodes[a];
    for (uint32_t index = 0u; index < node->roadCount; ++index)
    {
        const FoxRoad *road = &network->roads[node->roads[index]];
        if ((road->nodes[0] == a && road->nodes[1] == b) ||
            (road->nodes[0] == b && road->nodes[1] == a))
        {
            return true;
        }
    }
    return false;
}

/* ---- Building ---------------------------------------------------------------- */

typedef enum CutKind
{
    CUT_NEW = 0,
    CUT_NODE,
    CUT_SPLIT,
} CutKind;

typedef struct Cut
{
    /* Distance along the new road. */
    int64_t along;
    CutKind kind;
    uint32_t node;
    uint32_t road;
    uint32_t roadGeneration;
    int64_t roadAlong;
    FoxVec point;
    /* Filled while building. */
    uint32_t resolved;
} Cut;

static Cut ResolveEnd(const FoxNetwork *network, FoxVec point, int64_t along)
{
    Cut cut = {.along = along, .kind = CUT_NEW, .point = point, .node = FOX_NETWORK_NONE};
    const uint32_t node = FoxNetworkNearestNode(network, point, FOX_ROAD_SNAP_DISTANCE);
    if (node != FOX_NETWORK_NONE)
    {
        cut.kind = CUT_NODE;
        cut.node = node;
        cut.point = network->nodes[node].position;
        return cut;
    }
    int64_t roadAlong = 0;
    const uint32_t road = FoxNetworkNearestRoad(network, point, FOX_ROAD_SNAP_DISTANCE, &roadAlong);
    if (road != FOX_NETWORK_NONE)
    {
        const FoxRoad *record = &network->roads[road];
        if (roadAlong < FOX_ROAD_MIN_LENGTH || roadAlong > record->length - FOX_ROAD_MIN_LENGTH)
        {
            cut.kind = CUT_NODE;
            cut.node = record->nodes[roadAlong * 2 < record->length ? 0 : 1];
            cut.point = network->nodes[cut.node].position;
        }
        else
        {
            cut.kind = CUT_SPLIT;
            cut.road = road;
            cut.roadGeneration = record->generation;
            cut.roadAlong = roadAlong;
            cut.point = FoxPolylinePoint(record->centre, record->centreLengths, record->centreCount,
                                         roadAlong, NULL);
        }
    }
    return cut;
}

static int64_t Cross2(int64_t ax, int64_t ay, int64_t bx, int64_t by)
{
    return ax * by - ay * bx;
}

/* Where segment p0-p1 of the new road crosses q0-q1, as fractions of 1024
 * along each, in the plane tangent to the ground at p0. */
static bool Crossing(FoxVec p0, FoxVec p1, FoxVec q0, FoxVec q1, int64_t *outT, int64_t *outU)
{
    const FoxVec up = FoxPlanetUp(p0);
    FoxVec along = FoxVecSub(p1, p0);
    along = FoxVecSub(along, FoxVecScale(up, FoxVecDot(along, up), (int64_t)FOX_UNIT * FOX_UNIT));
    const FoxVec e1 = FoxVecDirection(along);
    if (FoxVecIsZero(e1))
    {
        return false;
    }
    const FoxVec e2 = FoxVecScale(FoxVecCross(up, e1), 1, FOX_UNIT);
    /* Plane coordinates in quarter units keep the products small. */
    const FoxVec relative[3] = {FoxVecSub(p1, p0), FoxVecSub(q0, p0), FoxVecSub(q1, p0)};
    int64_t x[3];
    int64_t y[3];
    for (uint32_t index = 0u; index < 3u; ++index)
    {
        x[index] = FoxVecDot(relative[index], e1) / (FOX_UNIT * 4);
        y[index] = FoxVecDot(relative[index], e2) / (FOX_UNIT * 4);
    }
    const int64_t rx = x[0];
    const int64_t ry = y[0];
    const int64_t sx = x[2] - x[1];
    const int64_t sy = y[2] - y[1];
    int64_t denominator = Cross2(rx, ry, sx, sy);
    if (denominator == 0)
    {
        return false;
    }
    int64_t tNumerator = Cross2(x[1], y[1], sx, sy);
    int64_t uNumerator = Cross2(x[1], y[1], rx, ry);
    if (denominator < 0)
    {
        denominator = -denominator;
        tNumerator = -tNumerator;
        uNumerator = -uNumerator;
    }
    if (tNumerator < 0 || tNumerator > denominator || uNumerator < 0 || uNumerator > denominator)
    {
        return false;
    }
    *outT = tNumerator * 1024 / denominator;
    *outU = uNumerator * 1024 / denominator;
    return true;
}

static int CompareCuts(const void *left, const void *right)
{
    const int64_t a = ((const Cut *)left)->along;
    const int64_t b = ((const Cut *)right)->along;
    return (a > b) - (a < b);
}

/* Whether a cut is one found already: where the lines cross at a point both
 * of them bend at, each pair of pieces meeting there finds it. */
static bool KnownCut(const Cut *cuts, uint32_t cutCount, const Cut *cut)
{
    for (uint32_t index = 0u; index < cutCount; ++index)
    {
        const Cut *other = &cuts[index];
        if (other->kind == cut->kind && FoxAbs(other->along - cut->along) < SAME_CUT &&
            (cut->kind == CUT_NODE ? other->node == cut->node
                                   : other->road == cut->road &&
                                         FoxAbs(other->roadAlong - cut->roadAlong) < SAME_CUT))
        {
            return true;
        }
    }
    return false;
}

/* Every place an existing road crosses the new centre line, away from its
 * ends. False when there are more than fit. */
static bool FindCrossings(const FoxNetwork *network, const FoxVec *points, const int64_t *lengths,
                          uint32_t count, Cut *cuts, uint32_t *cutCount, uint32_t capacity)
{
    const int64_t total = lengths[count - 1u];
    const FoxVec middle = FoxPolylinePoint(points, lengths, count, total / 2, NULL);
    int64_t bound = 0;
    for (uint32_t index = 0u; index < count; ++index)
    {
        bound = FoxMax(bound, FoxVecDistance(middle, points[index]));
    }
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (!road->alive || FoxVecDistance(middle, road->middle) > bound + road->boundRadius)
        {
            continue;
        }
        for (uint32_t i = 0u; i + 1u < count; ++i)
        {
            const int64_t spanP = lengths[i + 1u] - lengths[i];
            for (uint32_t j = 0u; j + 1u < road->centreCount; ++j)
            {
                const int64_t spanQ = road->centreLengths[j + 1u] - road->centreLengths[j];
                if (FoxVecDistance(points[i], road->centre[j]) > spanP + spanQ)
                {
                    continue;
                }
                int64_t t = 0;
                int64_t u = 0;
                if (!Crossing(points[i], points[i + 1u], road->centre[j], road->centre[j + 1u], &t,
                              &u))
                {
                    continue;
                }
                const int64_t along = lengths[i] + spanP * t / 1024;
                const int64_t roadAlong = road->centreLengths[j] + spanQ * u / 1024;
                if (along < FOX_ROAD_SNAP_DISTANCE || along > total - FOX_ROAD_SNAP_DISTANCE)
                {
                    continue;
                }
                if (*cutCount >= capacity)
                {
                    return false;
                }
                Cut cut = {.along = along, .resolved = FOX_NETWORK_NONE};
                if (roadAlong < FOX_ROAD_MIN_LENGTH ||
                    roadAlong > road->length - FOX_ROAD_MIN_LENGTH)
                {
                    cut.kind = CUT_NODE;
                    cut.node = road->nodes[roadAlong * 2 < road->length ? 0 : 1];
                    cut.point = network->nodes[cut.node].position;
                }
                else
                {
                    cut.kind = CUT_SPLIT;
                    cut.road = index;
                    cut.roadGeneration = road->generation;
                    cut.roadAlong = roadAlong;
                    cut.point = FoxPolylinePoint(road->centre, road->centreLengths,
                                                 road->centreCount, roadAlong, NULL);
                }
                if (!KnownCut(cuts, *cutCount, &cut))
                {
                    cuts[(*cutCount)++] = cut;
                }
            }
        }
    }
    /* Nodes the line runs over, such as the dead ends of roads it meets end
     * on, join it too. */
    for (uint32_t index = 0u; index < network->nodeCapacity; ++index)
    {
        const FoxNode *node = &network->nodes[index];
        if (!node->alive || FoxVecDistance(middle, node->position) > bound + FOX_ROAD_SNAP_DISTANCE)
        {
            continue;
        }
        bool known = false;
        for (uint32_t cut = 0u; cut < *cutCount && !known; ++cut)
        {
            known = cuts[cut].kind == CUT_NODE && cuts[cut].node == index;
        }
        int64_t best = FOX_ROAD_SNAP_DISTANCE + 1;
        int64_t along = 0;
        for (uint32_t i = 0u; !known && i + 1u < count; ++i)
        {
            int64_t distance = 0;
            const int64_t at =
                ClosestOnSegment(points[i], points[i + 1u], node->position, &distance);
            if (distance < best)
            {
                best = distance;
                along = lengths[i] + at;
            }
        }
        if (known || best > FOX_ROAD_SNAP_DISTANCE || along < FOX_ROAD_SNAP_DISTANCE ||
            along > total - FOX_ROAD_SNAP_DISTANCE)
        {
            continue;
        }
        if (*cutCount >= capacity)
        {
            return false;
        }
        cuts[(*cutCount)++] = (Cut){
            .along = along,
            .kind = CUT_NODE,
            .node = index,
            .point = node->position,
            .resolved = FOX_NETWORK_NONE,
        };
    }
    return true;
}

/* The road and distance that a split of (road, along) now refers to after
 * earlier splits of the same build. */
static void FollowLineage(const FoxNetwork *network, uint32_t *road, uint32_t generation,
                          int64_t *along)
{
    /* Each split leaves one entry per part; a part split again leaves its own. */
    for (uint32_t depth = 0u; depth < network->lineageCount; ++depth)
    {
        bool moved = false;
        for (uint32_t index = 0u; index < network->lineageCount && !moved; ++index)
        {
            const FoxLineage *entry = &network->lineage[index];
            if (entry->oldRoad == *road && entry->oldGeneration == generation &&
                *along >= entry->start && *along < entry->end)
            {
                *road = entry->road;
                *along += entry->offset;
                generation = entry->generation;
                moved = true;
            }
        }
        if (!moved)
        {
            return;
        }
    }
}
/* Room for planning a road: its centre line, the cuts along it and a piece
 * between two cuts. */
typedef struct RoadWork
{
    FoxVec *points;
    int64_t *lengths;
    Cut *cuts;
    FoxVec *piece;
    uint32_t count;
    uint32_t cutCount;
} RoadWork;

static bool RoadWorkInit(RoadWork *work)
{
    *work = (RoadWork){
        .points = malloc(FOX_ROAD_MAX_POINTS * sizeof(*work->points)),
        .lengths = malloc(FOX_ROAD_MAX_POINTS * sizeof(*work->lengths)),
        .cuts = malloc(MAX_CUTS * sizeof(*work->cuts)),
        .piece = malloc((FOX_ROAD_MAX_POINTS + 2u) * sizeof(*work->piece)),
    };
    return work->points != NULL && work->lengths != NULL && work->cuts != NULL &&
           work->piece != NULL;
}

static void RoadWorkFree(RoadWork *work)
{
    free(work->points);
    free(work->lengths);
    free(work->cuts);
    free(work->piece);
}

/* Lays out a road: its centre line with the ends snapped, and every cut it
 * makes, checked before anything changes. */
static FoxRoadResult PlanRoad(const FoxNetwork *network, const FoxPlanet *planet,
                              const FoxRoadPlan *plan, RoadWork *work)
{
    FoxVec *points = work->points;
    int64_t *lengths = work->lengths;
    Cut *cuts = work->cuts;
    const uint32_t count = FoxNetworkPlanCentre(planet, plan, points, FOX_ROAD_MAX_POINTS);
    work->count = count;
    if (count < 2u)
    {
        return FOX_ROAD_NO_GROUND;
    }
    int64_t total = FoxPolylineMeasure(points, lengths, count);
    if (total < FOX_ROAD_MIN_LENGTH)
    {
        return FOX_ROAD_TOO_SHORT;
    }
    if (total > FOX_ROAD_MAX_LENGTH)
    {
        return FOX_ROAD_TOO_LONG;
    }

    /* Ends first: they snap to nodes and roads, and the centre line follows. */
    uint32_t cutCount = 0u;
    cuts[cutCount++] = ResolveEnd(network, points[0], 0);
    Cut end = ResolveEnd(network, points[count - 1u], total);
    points[0] = cuts[0].point;
    points[count - 1u] = end.point;
    total = FoxPolylineMeasure(points, lengths, count);
    end.along = total;
    if (!FindCrossings(network, points, lengths, count, cuts, &cutCount, MAX_CUTS - 1u))
    {
        return FOX_ROAD_CROWDED;
    }
    cuts[cutCount++] = end;
    qsort(cuts + 1, cutCount - 2u, sizeof(*cuts), CompareCuts);

    /* Validate everything before changing anything. */
    for (uint32_t index = 0u; index + 1u < cutCount; ++index)
    {
        const Cut *a = &cuts[index];
        const Cut *b = &cuts[index + 1u];
        if (b->along - a->along < FOX_ROAD_MIN_LENGTH)
        {
            return FOX_ROAD_TOO_SHORT;
        }
        if (a->kind == CUT_NODE && b->kind == CUT_NODE &&
            (a->node == b->node || Joined(network, a->node, b->node)))
        {
            return FOX_ROAD_OVERLAP;
        }
        if (a->kind == CUT_SPLIT && b->kind == CUT_SPLIT && a->road == b->road)
        {
            /* Running along a road rather than across it. */
            return FOX_ROAD_OVERLAP;
        }
    }
    for (uint32_t index = 0u; index < cutCount; ++index)
    {
        const Cut *cut = &cuts[index];
        const uint32_t added = (index == 0u || index + 1u == cutCount) ? 1u : 2u;
        if (cut->kind == CUT_NODE)
        {
            uint32_t uses = added;
            for (uint32_t other = 0u; other < cutCount; ++other)
            {
                if (other != index && cuts[other].kind == CUT_NODE && cuts[other].node == cut->node)
                {
                    return FOX_ROAD_OVERLAP;
                }
            }
            if (network->nodes[cut->node].roadCount + uses > FOX_NODE_MAX_ROADS)
            {
                return FOX_ROAD_CROWDED;
            }
        }
        if (cut->kind == CUT_SPLIT)
        {
            /* Split pieces of the crossed road must stay long enough, also
             * against other splits of the same road. */
            const FoxRoad *road = &network->roads[cut->road];
            for (uint32_t other = 0u; other < cutCount; ++other)
            {
                if (other != index && cuts[other].kind == CUT_SPLIT &&
                    cuts[other].road == cut->road &&
                    FoxAbs(cuts[other].roadAlong - cut->roadAlong) < FOX_ROAD_MIN_LENGTH)
                {
                    return FOX_ROAD_TOO_SHORT;
                }
            }
            if (cut->roadAlong < FOX_ROAD_MIN_LENGTH ||
                road->length - cut->roadAlong < FOX_ROAD_MIN_LENGTH)
            {
                return FOX_ROAD_TOO_SHORT;
            }
        }
    }

    /* Away from its intersections the road must keep clear of other roads:
     * one laid along another would pave over it. */
    int64_t ownExtent[2];
    FoxRoadLayoutExtent(&plan->layout, ownExtent);
    const int64_t ownHalf = FoxMax(ownExtent[0], ownExtent[1]);
    for (uint32_t point = 0u; point < count; ++point)
    {
        int64_t along = 0;
        const uint32_t near =
            FoxNetworkNearestRoad(network, points[point], ownHalf + MAX_HALF_WIDTH, &along);
        if (near == FOX_NETWORK_NONE)
        {
            continue;
        }
        const FoxRoad *other = &network->roads[near];
        /* Near a cut the roads that meet there come close by nature; how
         * close depends on the angle, so the window spans both widths
         * twice, enough down to about thirty degrees. */
        const int64_t otherHalf = FoxRoadReach(other) - FOX_ROAD_SHOULDER;
        const int64_t window =
            FOX_ROAD_MIN_LENGTH + 2 * (CoreHalf(&plan->layout) + CoreHalf(&other->layout));
        bool meets = false;
        for (uint32_t index = 0u; index < cutCount && !meets; ++index)
        {
            const Cut *cut = &cuts[index];
            const bool related = (cut->kind == CUT_SPLIT && cut->road == near) ||
                                 (cut->kind == CUT_NODE &&
                                  (other->nodes[0] == cut->node || other->nodes[1] == cut->node));
            meets = related && FoxAbs(lengths[point] - cut->along) < window;
        }
        if (meets)
        {
            continue;
        }
        const FoxVec closest =
            FoxPolylinePoint(other->centre, other->centreLengths, other->centreCount, along, NULL);
        if (FoxVecDistance(closest, points[point]) < ownHalf + otherHalf)
        {
            return FOX_ROAD_OVERLAP;
        }
    }
    work->cutCount = cutCount;
    return FOX_ROAD_OK;
}

FoxRoadResult FoxNetworkCheckRoad(const FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan)
{
    if (network == NULL || planet == NULL || plan == NULL || !FoxRoadLayoutValid(&plan->layout) ||
        (plan->speedLimit != 0 &&
         (plan->speedLimit < FOX_SPEED_LIMIT_MIN || plan->speedLimit > FOX_SPEED_LIMIT_MAX)))
    {
        return FOX_ROAD_INVALID;
    }
    RoadWork work;
    const FoxRoadResult result =
        RoadWorkInit(&work) ? PlanRoad(network, planet, plan, &work) : FOX_ROAD_OUT_OF_MEMORY;
    RoadWorkFree(&work);
    return result;
}

FoxRoadResult FoxNetworkBuildRoad(FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan)
{
    if (network == NULL || planet == NULL || plan == NULL || !FoxRoadLayoutValid(&plan->layout) ||
        (plan->speedLimit != 0 &&
         (plan->speedLimit < FOX_SPEED_LIMIT_MIN || plan->speedLimit > FOX_SPEED_LIMIT_MAX)))
    {
        return FOX_ROAD_INVALID;
    }
    RoadWork work;
    FoxRoadResult result =
        RoadWorkInit(&work) ? PlanRoad(network, planet, plan, &work) : FOX_ROAD_OUT_OF_MEMORY;
    if (result != FOX_ROAD_OK)
    {
        RoadWorkFree(&work);
        return result;
    }
    const FoxVec *points = work.points;
    const int64_t *lengths = work.lengths;
    Cut *cuts = work.cuts;
    FoxVec *piece = work.piece;
    const uint32_t count = work.count;
    const uint32_t cutCount = work.cutCount;

    /* Build: nodes for every cut, then the pieces between them. */
    DirtySet dirty = {0};
    for (uint32_t index = 0u; index < cutCount; ++index)
    {
        Cut *cut = &cuts[index];
        if (cut->kind == CUT_NODE)
        {
            cut->resolved = cut->node;
        }
        else if (cut->kind == CUT_NEW)
        {
            cut->resolved = NewNode(network, cut->point);
        }
        else
        {
            /* An earlier cut of this build may have split the road already. */
            uint32_t road = cut->road;
            int64_t along = cut->roadAlong;
            if (!network->roads[road].alive ||
                network->roads[road].generation != cut->roadGeneration)
            {
                FollowLineage(network, &road, cut->roadGeneration, &along);
            }
            const uint32_t nodes[2] = {network->roads[road].nodes[0],
                                       network->roads[road].nodes[1]};
            cut->resolved =
                network->roads[road].alive ? SplitRoad(network, road, along) : FOX_NETWORK_NONE;
            MarkDirty(&dirty, nodes[0]);
            MarkDirty(&dirty, nodes[1]);
        }
        if (cut->resolved == FOX_NETWORK_NONE)
        {
            (void)Refresh(network, &dirty);
            RoadWorkFree(&work);
            return FOX_ROAD_OUT_OF_MEMORY;
        }
        MarkDirty(&dirty, cut->resolved);
        cut->point = network->nodes[cut->resolved].position;
    }
    for (uint32_t index = 0u; index + 1u < cutCount; ++index)
    {
        const Cut *a = &cuts[index];
        const Cut *b = &cuts[index + 1u];
        uint32_t pieceCount = 0u;
        piece[pieceCount++] = a->point;
        for (uint32_t point = 0u; point < count; ++point)
        {
            if (lengths[point] > a->along && lengths[point] < b->along)
            {
                piece[pieceCount++] = points[point];
            }
        }
        piece[pieceCount++] = b->point;
        if (NewRoad(network, a->resolved, b->resolved, &plan->layout, plan->speedLimit, piece,
                    pieceCount) == FOX_NETWORK_NONE)
        {
            result = FOX_ROAD_OUT_OF_MEMORY;
            break;
        }
    }
    if (!Refresh(network, &dirty) && result == FOX_ROAD_OK)
    {
        result = FOX_ROAD_OUT_OF_MEMORY;
    }
    RoadWorkFree(&work);
    return result;
}

/* Whether cars are on a road, have leave to come onto it or are leaving it
 * through an intersection at its end. */
static bool RoadInUse(const FoxNetwork *network, uint32_t road)
{
    const FoxRoad *record = &network->roads[road];
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < record->laneCount[direction]; ++lane)
        {
            const FoxPath *path = &network->paths[record->lanes[direction][lane]];
            if (path->vehicleCount != 0u || path->incoming != 0u)
            {
                return true;
            }
            for (uint32_t index = 0u; index < path->outgoingCount; ++index)
            {
                const FoxPath *way = &network->paths[path->outgoing[index]];
                if (way->vehicleCount != 0u || way->claims != 0u)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

/* Closes a road: out of the routes, and its intersections look new to the
 * views, which no longer show it. */
static void CloseRoad(FoxNetwork *network, uint32_t road)
{
    FoxRoad *record = &network->roads[road];
    RouteRoad(network, road, false);
    record->closing = true;
    ++network->closingCount;
    record->revision = network->revision + 1u;
    network->nodes[record->nodes[0]].revision = network->revision + 1u;
    network->nodes[record->nodes[1]].revision = network->revision + 1u;
    Touch(network);
}

/* Takes a road out for good, and the nodes it leaves without roads. */
static void TakeOut(FoxNetwork *network, uint32_t road)
{
    if (network->roads[road].closing)
    {
        network->roads[road].closing = false;
        --network->closingCount;
    }
    const uint32_t ends[2] = {network->roads[road].nodes[0], network->roads[road].nodes[1]};
    DeleteRoad(network, road);
    DirtySet dirty = {0};
    for (uint32_t index = 0u; index < 2u; ++index)
    {
        if (network->nodes[ends[index]].alive && network->nodes[ends[index]].roadCount == 0u)
        {
            DeleteNode(network, ends[index]);
        }
        MarkDirty(&dirty, ends[index]);
    }
    (void)Refresh(network, &dirty);
}

bool FoxNetworkRemoveRoad(FoxNetwork *network, uint32_t road)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive)
    {
        return false;
    }
    if (RoadInUse(network, road))
    {
        if (!network->roads[road].closing)
        {
            CloseRoad(network, road);
        }
        return true;
    }
    TakeOut(network, road);
    return true;
}

bool FoxNetworkSettleClosing(FoxNetwork *network, FoxVec center, int64_t radius)
{
    bool changed = false;
    for (uint32_t index = 0u;
         network != NULL && network->closingCount != 0u && index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        const bool near =
            radius > 0 && FoxVecDistance(road->middle, center) <= road->boundRadius + radius;
        /* After one has gone the lanes of the others may be new while their
         * cars are still on the old ones: whether those are empty waits
         * for the traffic to follow. */
        if (road->alive && road->closing && (near || (!changed && !RoadInUse(network, index))))
        {
            TakeOut(network, index);
            changed = true;
        }
    }
    return changed;
}

bool FoxNetworkAddDriveZone(FoxNetwork *network, FoxVec center, int64_t radius, FoxDriveSide side)
{
    if (network == NULL || radius <= 0 || (side != FOX_DRIVE_RIGHT && side != FOX_DRIVE_LEFT) ||
        network->zoneCount >= FOX_NETWORK_MAX_DRIVE_ZONES)
    {
        return false;
    }
    network->zones[network->zoneCount++] =
        (FoxDriveZone){.center = center, .radius = radius, .side = (uint8_t)side};
    /* Roads whose side changed get new lanes; nodes whose side changed or
     * whose roads did get new connections. */
    const uint32_t nodeCapacity = network->nodeCapacity;
    bool *rebuild = calloc(nodeCapacity == 0u ? 1u : nodeCapacity, sizeof(*rebuild));
    if (rebuild == NULL)
    {
        --network->zoneCount;
        return false;
    }
    bool ok = true;
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        FoxRoad *road = &network->roads[index];
        if (!road->alive)
        {
            continue;
        }
        const uint8_t now = (uint8_t)FoxNetworkDriveSideAt(network, road->middle);
        if (now != road->driveSide)
        {
            road->driveSide = now;
            ok = FoxLanesBuildRoad(network, index) && ok;
            road->revision = network->revision + 1u;
            rebuild[road->nodes[0]] = true;
            rebuild[road->nodes[1]] = true;
        }
    }
    for (uint32_t index = 0u; index < nodeCapacity; ++index)
    {
        FoxNode *node = &network->nodes[index];
        if (!node->alive)
        {
            continue;
        }
        const uint8_t now = (uint8_t)FoxNetworkDriveSideAt(network, node->position);
        if (now != node->driveSide || rebuild[index])
        {
            node->driveSide = now;
            ok = FoxLanesBuildNode(network, index) && ok;
            node->revision = network->revision + 1u;
        }
    }
    free(rebuild);
    Touch(network);
    return ok;
}

/* ---- Signs, lights and limits ------------------------------------------------ */

/* Whether a road laid out as `layout` keeps clear of the roads around it,
 * away from the ends where roads meet it by nature. */
static bool LayoutFits(const FoxNetwork *network, uint32_t roadIndex, const FoxRoadLayout *layout)
{
    const FoxRoad *road = &network->roads[roadIndex];
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    const int64_t own = FoxMax(extent[0], extent[1]);
    for (uint32_t point = 0u; point < road->centreCount; ++point)
    {
        const int64_t along = road->centreLengths[point];
        for (uint32_t index = 0u; index < network->roadCapacity; ++index)
        {
            const FoxRoad *other = &network->roads[index];
            if (index == roadIndex || !other->alive || other->closing ||
                FoxVecDistance(road->centre[point], other->middle) >
                    other->boundRadius + own + MAX_HALF_WIDTH)
            {
                continue;
            }
            const int64_t room = own + FoxRoadReach(other) - FOX_ROAD_SHOULDER;
            /* Near an end both share, the two come close by nature. */
            const int64_t window =
                FOX_ROAD_MIN_LENGTH + 2 * (CoreHalf(layout) + CoreHalf(&other->layout));
            bool shared = false;
            for (uint32_t end = 0u; end < 2u && !shared; ++end)
            {
                const bool meets =
                    other->nodes[0] == road->nodes[end] || other->nodes[1] == road->nodes[end];
                const int64_t fromEnd = end == 0u ? along : road->length - along;
                shared = meets && fromEnd < window;
            }
            if (shared)
            {
                continue;
            }
            for (uint32_t segment = 0u; segment + 1u < other->centreCount; ++segment)
            {
                int64_t distance = 0;
                (void)ClosestOnSegment(other->centre[segment], other->centre[segment + 1u],
                                       road->centre[point], &distance);
                if (distance < room)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

/* Lays a road out anew and rebuilds it and its ends; the limit follows the
 * new layout unless the mayor or the plan set one. */
static void Relayout(FoxNetwork *network, uint32_t roadIndex, const FoxRoadLayout *layout)
{
    FoxRoad *road = &network->roads[roadIndex];
    road->layout = *layout;
    road->driven = *layout;
    if (road->speedSetting == 0)
    {
        road->speedLimit = FoxRoadOwnSpeed(road);
    }
    road->revision = network->revision + 1u;
    RouteRoad(network, roadIndex, true);
    DirtySet dirty = {0};
    MarkDirty(&dirty, road->nodes[0]);
    MarkDirty(&dirty, road->nodes[1]);
    (void)Refresh(network, &dirty);
}

static bool Editable(const FoxNetwork *network, uint32_t road)
{
    return network != NULL && road < network->roadCapacity && network->roads[road].alive &&
           !network->roads[road].closing;
}

FoxRoadResult FoxNetworkWidenRoad(FoxNetwork *network, uint32_t road, uint32_t side,
                                  FoxLaneKind kind, bool forward)
{
    if (!Editable(network, road) || side > 1u || (unsigned)kind >= FOX_LANE_KIND_COUNT)
    {
        return FOX_ROAD_INVALID;
    }
    FoxRoadLayout layout = network->roads[road].layout;
    const uint32_t direction = forward ? 0u : 1u;
    switch (kind)
    {
    case FOX_LANE_BIKE:
        if (layout.bike[side] != FOX_WAY_NONE)
        {
            return FOX_ROAD_INVALID;
        }
        layout.bike[side] = (uint8_t)(forward ? FOX_WAY_FORWARD : FOX_WAY_BACKWARD);
        break;
    case FOX_LANE_FOOT:
        if (layout.foot[side])
        {
            return FOX_ROAD_INVALID;
        }
        layout.foot[side] = true;
        break;
    default:
        if (layout.shared)
        {
            /* The lane driven both ways keeps to the other way now. */
            layout.shared = false;
            layout.lanes[direction] = 0u;
            layout.surface[1u - direction][0] = layout.surface[0][0];
        }
        if (layout.lanes[direction] >= FOX_ROAD_MAX_LANES)
        {
            return FOX_ROAD_INVALID;
        }
        /* The new lane joins its way on the inside, next to the other way. */
        layout.surface[direction][layout.lanes[direction]++] = (uint8_t)kind;
        break;
    }
    if (!FoxRoadLayoutValid(&layout))
    {
        return FOX_ROAD_INVALID;
    }
    if (!LayoutFits(network, road, &layout))
    {
        return FOX_ROAD_OVERLAP;
    }
    Relayout(network, road, &layout);
    return FOX_ROAD_OK;
}

bool FoxNetworkTurnStrip(FoxNetwork *network, uint32_t road, int64_t offset)
{
    if (!Editable(network, road))
    {
        return false;
    }
    const FoxRoad *record = &network->roads[road];
    FoxRoadLayout layout = record->layout;
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t count = FoxRoadLayoutStrips(&layout, (FoxDriveSide)record->driveSide, strips);
    for (uint32_t index = 0u; index < count; ++index)
    {
        const FoxRoadStrip *strip = &strips[index];
        if (offset < strip->left || offset >= strip->right)
        {
            continue;
        }
        if (strip->kind == FOX_LANE_FOOT)
        {
            return false;
        }
        if (strip->kind == FOX_LANE_BIKE)
        {
            /* The first bike lane from the left is the left edge's, if it has one. */
            bool before = false;
            for (uint32_t earlier = 0u; earlier < index; ++earlier)
            {
                before = before || strips[earlier].kind == FOX_LANE_BIKE;
            }
            const uint32_t edge = !before && layout.bike[0] != FOX_WAY_NONE ? 0u : 1u;
            layout.bike[edge] = (uint8_t)(layout.bike[edge] % FOX_WAY_BOTH + 1u);
        }
        else if (FoxRoadLayoutCarLanes(&layout) == 1u)
        {
            /* A lone lane: one way, the other, both ways. */
            const uint8_t surface = layout.shared || layout.lanes[0] != 0u ? layout.surface[0][0]
                                                                           : layout.surface[1][0];
            if (layout.shared)
            {
                layout.shared = false;
                layout.lanes[0] = 1u;
                layout.lanes[1] = 0u;
            }
            else if (layout.lanes[0] != 0u)
            {
                layout.lanes[0] = 0u;
                layout.lanes[1] = 1u;
            }
            else
            {
                layout.shared = true;
                layout.lanes[0] = 1u;
                layout.lanes[1] = 1u;
            }
            layout.surface[0][0] = surface;
            layout.surface[1][0] = surface;
        }
        else
        {
            /* The lane in this slot goes over to the other way, on its inside. */
            const uint32_t direction = strip->way == FOX_WAY_FORWARD ? 0u : 1u;
            uint32_t lane = 0u;
            for (uint32_t candidate = 0u; candidate < layout.lanes[direction]; ++candidate)
            {
                const int64_t centre = FoxRoadLaneOffset(&layout, (FoxDriveSide)record->driveSide,
                                                         direction, candidate);
                if (centre > strip->left && centre < strip->right)
                {
                    lane = candidate;
                }
            }
            const uint32_t other = 1u - direction;
            if (layout.lanes[other] >= FOX_ROAD_MAX_LANES)
            {
                return false;
            }
            const uint8_t surface = layout.surface[direction][lane];
            for (uint32_t next = lane; next + 1u < layout.lanes[direction]; ++next)
            {
                layout.surface[direction][next] = layout.surface[direction][next + 1u];
            }
            --layout.lanes[direction];
            layout.surface[other][layout.lanes[other]++] = surface;
        }
        Relayout(network, road, &layout);
        return true;
    }
    return false;
}

bool FoxNetworkToggleCrossing(FoxNetwork *network, uint32_t road, int64_t along)
{
    if (!Editable(network, road))
    {
        return false;
    }
    FoxRoad *record = &network->roads[road];
    if (FoxRoadLayoutCarLanes(&record->layout) == 0u)
    {
        return false;
    }
    for (uint32_t index = 0u; index < record->crossingCount; ++index)
    {
        if (FoxAbs(record->crossings[index] - along) < CROSSING_ROOM)
        {
            /* The one there goes. */
            for (uint32_t later = index; later + 1u < record->crossingCount; ++later)
            {
                record->crossings[later] = record->crossings[later + 1u];
            }
            --record->crossingCount;
            Relayout(network, road, &record->layout);
            return true;
        }
    }
    if (record->crossingCount >= FOX_ROAD_MAX_CROSSINGS ||
        along < record->trim[0] + CROSSING_ROOM ||
        along > record->length - record->trim[1] - CROSSING_ROOM)
    {
        return false;
    }
    uint32_t at = record->crossingCount;
    while (at > 0u && record->crossings[at - 1u] > along)
    {
        record->crossings[at] = record->crossings[at - 1u];
        --at;
    }
    record->crossings[at] = along;
    ++record->crossingCount;
    Relayout(network, road, &record->layout);
    return true;
}

bool FoxNetworkSetSpeedLimit(FoxNetwork *network, uint32_t road, int32_t speed)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive ||
        (speed != 0 && (speed < FOX_SPEED_LIMIT_MIN || speed > FOX_SPEED_LIMIT_MAX)))
    {
        return false;
    }
    FoxRoad *record = &network->roads[road];
    record->speedSetting = speed;
    record->speedLimit = speed != 0 ? speed : FoxRoadOwnSpeed(record);
    FoxLanesApplyRoadSpeed(network, road);
    record->revision = network->revision + 1u;
    for (uint32_t end = 0u; end < 2u; ++end)
    {
        FoxLanesApplyNodeSpeeds(network, record->nodes[end]);
        network->nodes[record->nodes[end]].revision = network->revision + 1u;
    }
    RouteRoad(network, road, true);
    Touch(network);
    return true;
}

/* Marks a node and its roads changed for the views. */
static void TouchNode(FoxNetwork *network, uint32_t node)
{
    FoxNode *record = &network->nodes[node];
    record->revision = network->revision + 1u;
    for (uint32_t slot = 0u; slot < record->roadCount; ++slot)
    {
        network->roads[record->roads[slot]].revision = network->revision + 1u;
    }
    Touch(network);
}

bool FoxNetworkSetSign(FoxNetwork *network, uint32_t road, uint32_t end, FoxSign sign)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive ||
        end > 1u || (unsigned)sign >= FOX_SIGN_COUNT)
    {
        return false;
    }
    FoxRoad *record = &network->roads[road];
    record->signSetting[end] = (uint8_t)sign;
    const uint32_t node = record->nodes[end];
    FoxLanesResolveSigns(network, node);
    FoxLanesApplyNodeSpeeds(network, node);
    const bool ok = FoxLanesBuildConflicts(network, node);
    TouchNode(network, node);
    return ok;
}

bool FoxNetworkSetSignals(FoxNetwork *network, uint32_t node, bool on)
{
    if (network == NULL || node >= network->nodeCapacity || !network->nodes[node].alive ||
        (on && network->nodes[node].roadCount < 3u))
    {
        return false;
    }
    network->nodes[node].signals = on;
    FoxSignalsAssign(network, node);
    FoxLanesResolveSigns(network, node);
    FoxLanesApplyNodeSpeeds(network, node);
    const bool ok = FoxLanesBuildConflicts(network, node);
    TouchNode(network, node);
    return ok;
}

FoxLight FoxNetworkLight(const FoxNetwork *network, uint32_t road, uint32_t end)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive || end > 1u)
    {
        return FOX_LIGHT_NONE;
    }
    return FoxSignalsLight(network, road, end);
}

FoxLight FoxNetworkArrow(const FoxNetwork *network, uint32_t road, uint32_t end)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive || end > 1u)
    {
        return FOX_LIGHT_NONE;
    }
    return FoxSignalsArrow(network, road, end);
}

uint32_t FoxNetworkLaneTurns(const FoxNetwork *network, uint32_t lane)
{
    const FoxPath *path = FoxNetworkPathRecord(network, lane);
    if (path == NULL || path->retired || path->kind != PATH_LANE)
    {
        return 0u;
    }
    uint32_t turns = 0u;
    for (uint32_t index = 0u; index < path->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, path->outgoing[index]);
        if (connection == NULL)
        {
            continue;
        }
        const bool right = network->nodes[connection->node].driveSide == FOX_DRIVE_RIGHT;
        switch (connection->movement)
        {
        case MOVE_STRAIGHT:
            turns |= FOX_TURN_STRAIGHT;
            break;
        case MOVE_CURB:
            turns |= right ? FOX_TURN_RIGHT : FOX_TURN_LEFT;
            break;
        case MOVE_FAR:
            turns |= right ? FOX_TURN_LEFT : FOX_TURN_RIGHT;
            break;
        default:
            turns |= FOX_TURN_BACK;
            break;
        }
    }
    return turns;
}

FoxNetworkStats FoxNetworkGetStats(const FoxNetwork *network)
{
    FoxNetworkStats stats = {0};
    if (network == NULL)
    {
        return stats;
    }
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (!road->alive)
        {
            continue;
        }
        ++stats.roads;
        ++stats.roadsByLanes[road->layout.shared
                                 ? 1u
                                 : (uint32_t)road->layout.lanes[0] + road->layout.lanes[1]];
        stats.crossings += road->crossingCount;
        stats.footwayLength +=
            road->length * ((road->layout.foot[0] ? 1 : 0) + (road->layout.foot[1] ? 1 : 0));
        stats.roadLength += road->length;
        stats.speedLimitsSet += road->speedSetting != 0 ? 1u : 0u;
        for (uint32_t end = 0u; end < 2u; ++end)
        {
            const FoxNode *node = &network->nodes[road->nodes[end]];
            if (node->roadCount >= 3u && !node->signals)
            {
                ++stats.signs[road->sign[end]];
            }
            for (uint32_t lane = 0u; lane < road->laneCount[end]; ++lane)
            {
                ++stats.lanes;
                stats.laneLength += network->paths[road->lanes[end][lane]].length;
            }
        }
    }
    for (uint32_t index = 0u; index < network->nodeCapacity; ++index)
    {
        const FoxNode *node = &network->nodes[index];
        if (!node->alive)
        {
            continue;
        }
        ++stats.nodes;
        stats.junctions += node->roadCount >= 3u ? 1u : 0u;
        stats.bends += node->roadCount == 2u ? 1u : 0u;
        stats.deadEnds += node->roadCount == 1u ? 1u : 0u;
        stats.signalled += node->signals ? 1u : 0u;
        stats.connections += node->connectionCount;
        for (uint32_t connection = 0u; connection < node->connectionCount; ++connection)
        {
            stats.conflicts += network->paths[node->connections[connection]].conflictCount;
        }
    }
    stats.conflicts /= 2u;
    return stats;
}

uint64_t FoxNetworkHash(const FoxNetwork *network)
{
    if (network == NULL)
    {
        return 0u;
    }
    uint64_t hash = FoxHash64(network->defaultSide);
    for (uint32_t index = 0u; index < network->nodeCapacity; ++index)
    {
        const FoxNode *node = &network->nodes[index];
        if (node->alive)
        {
            hash = FoxHashCombine(hash, index);
            hash = FoxHashCombine(hash, (uint64_t)node->position.x);
            hash = FoxHashCombine(hash, (uint64_t)node->position.y);
            hash = FoxHashCombine(hash, (uint64_t)node->position.z);
            hash = FoxHashCombine(hash, node->driveSide);
            hash = FoxHashCombine(hash, node->signals ? 1u : 0u);
        }
    }
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (!road->alive || road->closing)
        {
            continue;
        }
        hash = FoxHashCombine(hash, index);
        hash = FoxHashCombine(hash, road->nodes[0]);
        hash = FoxHashCombine(hash, road->nodes[1]);
        hash = FoxHashCombine(hash, LayoutHash(&road->layout));
        hash = FoxHashCombine(hash, (uint64_t)(uint32_t)road->builtSpeed);
        for (uint32_t crossing = 0u; crossing < road->crossingCount; ++crossing)
        {
            hash = FoxHashCombine(hash, (uint64_t)road->crossings[crossing]);
        }
        hash = FoxHashCombine(hash, road->driveSide);
        hash = FoxHashCombine(hash, (uint64_t)road->length);
        hash = FoxHashCombine(hash, (uint64_t)(uint32_t)road->speedSetting);
        hash = FoxHashCombine(hash, road->signSetting[0] | (uint64_t)road->signSetting[1] << 8u);
        for (uint32_t point = 0u; point < road->centreCount; ++point)
        {
            hash = FoxHashCombine(hash, (uint64_t)road->centre[point].x);
            hash = FoxHashCombine(hash, (uint64_t)road->centre[point].y);
            hash = FoxHashCombine(hash, (uint64_t)road->centre[point].z);
        }
    }
    for (uint32_t index = 0u; index < network->zoneCount; ++index)
    {
        hash = FoxHashCombine(hash, (uint64_t)network->zones[index].radius);
        hash = FoxHashCombine(hash, network->zones[index].side);
    }
    return hash;
}
