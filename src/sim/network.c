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

static const FoxRoadTypeInfo roadTypes[FOX_ROAD_TYPE_COUNT] = {
    [FOX_ROAD_STREET] = {"street", (int32_t)(FOX_METRES(40) * 10 / 36), 1u, 1u, 1u},
    [FOX_ROAD_AVENUE] = {"avenue", (int32_t)(FOX_METRES(50) * 10 / 36), 2u, 2u, 2u},
    [FOX_ROAD_BOULEVARD] = {"boulevard", (int32_t)(FOX_METRES(60) * 10 / 36), 3u, 3u, 3u},
    [FOX_ROAD_ONE_WAY] = {"one-way street", (int32_t)(FOX_METRES(50) * 10 / 36), 2u, 0u, 2u},
    [FOX_ROAD_HIGHWAY] = {"highway", (int32_t)FOX_ROUTER_MAX_SPEED, 3u, 0u, 4u},
};

const FoxRoadTypeInfo *FoxRoadTypeGet(FoxRoadType type)
{
    return (unsigned)type < FOX_ROAD_TYPE_COUNT ? &roadTypes[type] : NULL;
}

/* Half the paved width of a road type. */
static int64_t TypeHalfWidth(uint32_t type)
{
    const FoxRoadTypeInfo *info = FoxRoadTypeGet((FoxRoadType)type);
    return (int64_t)(info->forwardLanes + info->backwardLanes) * FOX_LANE_WIDTH / 2;
}

#define MAX_HALF_WIDTH (6 * FOX_LANE_WIDTH / 2)

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
    path->points = NULL;
    path->lengths = NULL;
    path->centre = NULL;
    path->outgoing = NULL;
    path->conflicts = NULL;
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
    return network == NULL ? 0u : network->roadCount;
}

uint32_t FoxNetworkNodeCount(const FoxNetwork *network)
{
    return network == NULL ? 0u : network->nodeCount;
}

bool FoxNetworkRoad(const FoxNetwork *network, uint32_t road, FoxRoadView *outView)
{
    if (network == NULL || outView == NULL || road >= network->roadCapacity ||
        !network->roads[road].alive)
    {
        return false;
    }
    const FoxRoad *record = &network->roads[road];
    *outView = (FoxRoadView){
        .nodes = {record->nodes[0], record->nodes[1]},
        .type = (FoxRoadType)record->type,
        .driveSide = (FoxDriveSide)record->driveSide,
        .centre = record->centre,
        .centreCount = record->centreCount,
        .length = record->length,
        .trim = {record->trim[0], record->trim[1]},
        .laneCount = {record->laneCount[0], record->laneCount[1]},
        .revision = record->revision,
    };
    memcpy(outView->lanes, record->lanes, sizeof(outView->lanes));
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
        .roadCount = record->roadCount,
        .radius = record->radius,
        .revision = record->revision,
    };
    memcpy(outView->roads, record->roads, sizeof(outView->roads));
    return true;
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
        FoxVec direction = FoxVecDirection(FoxVecSub(record->points[index + 1u], record->points[index]));
        if (FoxVecIsZero(direction))
        {
            direction = FoxVecDirection(
                FoxVecSub(record->points[record->count - 1u], record->points[0]));
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
        if (!road->alive ||
            FoxVecDistance(point, road->middle) > road->boundRadius + maxDistance)
        {
            continue;
        }
        for (uint32_t segment = 0u; segment + 1u < road->centreCount; ++segment)
        {
            int64_t distance = 0;
            const int64_t along = ClosestOnSegment(road->centre[segment], road->centre[segment + 1u],
                                                   point, &distance);
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

/* ---- Planning --------------------------------------------------------------- */

uint32_t FoxNetworkPlanCentre(const FoxPlanet *planet, const FoxRoadPlan *plan, FoxVec *points,
                              uint32_t capacity)
{
    if (planet == NULL || plan == NULL || points == NULL || capacity < 2u ||
        FoxRoadTypeGet(plan->type) == NULL || FoxVecEqual(plan->start, plan->end))
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
    if (index == network->nodeCapacity &&
        !Grow((void **)&network->nodes, &network->nodeCapacity, index + 1u,
              sizeof(*network->nodes)))
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
    const FoxRoadTypeInfo *type = FoxRoadTypeGet((FoxRoadType)road->type);
    const uint8_t lanes[2] = {type->forwardLanes, type->backwardLanes};
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        if (lanes[direction] == 0u)
        {
            continue;
        }
        const uint32_t from = road->nodes[direction];
        const uint32_t to = road->nodes[1u - direction];
        const uint32_t cost = alive ? TravelCost(road->length, type->speedLimit) : 0u;
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
static uint32_t NewRoad(FoxNetwork *network, uint32_t from, uint32_t to, uint8_t type,
                        const FoxVec *points, uint32_t count)
{
    uint32_t index = 0u;
    while (index < network->roadCapacity && network->roads[index].alive)
    {
        ++index;
    }
    if (index == network->roadCapacity &&
        !Grow((void **)&network->roads, &network->roadCapacity, index + 1u,
              sizeof(*network->roads)))
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
        .type = type,
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
    road->length = FoxPolylineMeasure(centre, lengths, count);
    road->middle = FoxPolylinePoint(centre, lengths, count, road->length / 2, NULL);
    for (uint32_t point = 0u; point < count; ++point)
    {
        road->boundRadius = FoxMax(road->boundRadius, FoxVecDistance(road->middle, centre[point]));
    }
    road->driveSide = (uint8_t)FoxNetworkDriveSideAt(network, road->middle);
    const FoxRoadTypeInfo *info = FoxRoadTypeGet((FoxRoadType)type);
    road->averageSpeed[0] = info->speedLimit;
    road->averageSpeed[1] = info->speedLimit;
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
    const uint8_t type = road->type;
    const uint32_t generation = road->generation;
    const int64_t length = road->length;

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
    const uint32_t first = NewRoad(network, from, node, type, points, firstCount);
    const uint32_t last = first == FOX_NETWORK_NONE
                              ? FOX_NETWORK_NONE
                              : NewRoad(network, node, to, type, secondBuffer, secondCount);
    free(points);
    free(secondBuffer);
    if (first == FOX_NETWORK_NONE || last == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
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
    const uint32_t road =
        FoxNetworkNearestRoad(network, point, FOX_ROAD_SNAP_DISTANCE, &roadAlong);
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
            cut.point = FoxPolylinePoint(record->centre, record->centreLengths,
                                         record->centreCount, roadAlong, NULL);
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
                cuts[(*cutCount)++] = cut;
            }
        }
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
FoxRoadResult FoxNetworkBuildRoad(FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan)
{
    if (network == NULL || planet == NULL || plan == NULL || FoxRoadTypeGet(plan->type) == NULL)
    {
        return FOX_ROAD_INVALID;
    }
    FoxVec *points = malloc(FOX_ROAD_MAX_POINTS * sizeof(*points));
    int64_t *lengths = malloc(FOX_ROAD_MAX_POINTS * sizeof(*lengths));
    Cut *cuts = malloc(MAX_CUTS * sizeof(*cuts));
    FoxVec *piece = malloc((FOX_ROAD_MAX_POINTS + 2u) * sizeof(*piece));
    FoxRoadResult result = FOX_ROAD_OK;
    if (points == NULL || lengths == NULL || cuts == NULL || piece == NULL)
    {
        result = FOX_ROAD_OUT_OF_MEMORY;
        goto cleanup;
    }
    const uint32_t count = FoxNetworkPlanCentre(planet, plan, points, FOX_ROAD_MAX_POINTS);
    if (count < 2u)
    {
        result = FOX_ROAD_NO_GROUND;
        goto cleanup;
    }
    int64_t total = FoxPolylineMeasure(points, lengths, count);
    if (total < FOX_ROAD_MIN_LENGTH)
    {
        result = FOX_ROAD_TOO_SHORT;
        goto cleanup;
    }
    if (total > FOX_ROAD_MAX_LENGTH)
    {
        result = FOX_ROAD_TOO_LONG;
        goto cleanup;
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
        result = FOX_ROAD_CROWDED;
        goto cleanup;
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
            result = FOX_ROAD_TOO_SHORT;
            goto cleanup;
        }
        if (a->kind == CUT_NODE && b->kind == CUT_NODE &&
            (a->node == b->node || Joined(network, a->node, b->node)))
        {
            result = FOX_ROAD_OVERLAP;
            goto cleanup;
        }
        if (a->kind == CUT_SPLIT && b->kind == CUT_SPLIT && a->road == b->road)
        {
            /* Running along a road rather than across it. */
            result = FOX_ROAD_OVERLAP;
            goto cleanup;
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
                    result = FOX_ROAD_OVERLAP;
                    goto cleanup;
                }
            }
            if (network->nodes[cut->node].roadCount + uses > FOX_NODE_MAX_ROADS)
            {
                result = FOX_ROAD_CROWDED;
                goto cleanup;
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
                    result = FOX_ROAD_TOO_SHORT;
                    goto cleanup;
                }
            }
            if (cut->roadAlong < FOX_ROAD_MIN_LENGTH ||
                road->length - cut->roadAlong < FOX_ROAD_MIN_LENGTH)
            {
                result = FOX_ROAD_TOO_SHORT;
                goto cleanup;
            }
        }
    }

    /* Away from its intersections the road must keep clear of other roads:
     * one laid along another would pave over it. */
    const int64_t ownHalf = TypeHalfWidth(plan->type);
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
        const int64_t window = FOX_ROAD_MIN_LENGTH + 2 * (ownHalf + TypeHalfWidth(other->type));
        bool meets = false;
        for (uint32_t index = 0u; index < cutCount && !meets; ++index)
        {
            const Cut *cut = &cuts[index];
            const bool related =
                (cut->kind == CUT_SPLIT && cut->road == near) ||
                (cut->kind == CUT_NODE && (other->nodes[0] == cut->node || other->nodes[1] == cut->node));
            meets = related && FoxAbs(lengths[point] - cut->along) < window;
        }
        if (meets)
        {
            continue;
        }
        const FoxVec closest =
            FoxPolylinePoint(other->centre, other->centreLengths, other->centreCount, along, NULL);
        if (FoxVecDistance(closest, points[point]) < ownHalf + TypeHalfWidth(other->type))
        {
            result = FOX_ROAD_OVERLAP;
            goto cleanup;
        }
    }

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
            const uint32_t nodes[2] = {network->roads[road].nodes[0], network->roads[road].nodes[1]};
            cut->resolved = network->roads[road].alive ? SplitRoad(network, road, along)
                                                       : FOX_NETWORK_NONE;
            MarkDirty(&dirty, nodes[0]);
            MarkDirty(&dirty, nodes[1]);
        }
        if (cut->resolved == FOX_NETWORK_NONE)
        {
            result = FOX_ROAD_OUT_OF_MEMORY;
            (void)Refresh(network, &dirty);
            goto cleanup;
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
        if (NewRoad(network, a->resolved, b->resolved, (uint8_t)plan->type, piece, pieceCount) ==
            FOX_NETWORK_NONE)
        {
            result = FOX_ROAD_OUT_OF_MEMORY;
            break;
        }
    }
    if (!Refresh(network, &dirty) && result == FOX_ROAD_OK)
    {
        result = FOX_ROAD_OUT_OF_MEMORY;
    }

cleanup:
    free(points);
    free(lengths);
    free(cuts);
    free(piece);
    return result;
}

bool FoxNetworkRemoveRoad(FoxNetwork *network, uint32_t road)
{
    if (network == NULL || road >= network->roadCapacity || !network->roads[road].alive)
    {
        return false;
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
    return true;
}

bool FoxNetworkAddDriveZone(FoxNetwork *network, FoxVec center, int64_t radius,
                            FoxDriveSide side)
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
        }
    }
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        if (!road->alive)
        {
            continue;
        }
        hash = FoxHashCombine(hash, index);
        hash = FoxHashCombine(hash, road->nodes[0]);
        hash = FoxHashCombine(hash, road->nodes[1]);
        hash = FoxHashCombine(hash, road->type);
        hash = FoxHashCombine(hash, road->driveSide);
        hash = FoxHashCombine(hash, (uint64_t)road->length);
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
