#include "sim/network_internal.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Connections are sampled about every metre, within these bounds; conflict
 * tests compare every sample. */
#define CONNECTION_MIN_SAMPLES 8u
#define CONNECTION_MAX_SAMPLES 24u
/* The sideways acceleration drivers accept in a bend, and the slowest any
 * bend asks for. */
#define LATERAL_ACCELERATION (FOX_METRES(5) / 2)
#define MIN_CURVE_SPEED FOX_METRES(2)
/* How far from a node its roads' directions are measured. */
#define DIRECTION_PROBE FOX_METRES(6)
/* Where roads meet, each one stops this far beyond the point its edge would
 * touch the next road's: the room of the curb or the bend between them. */
#define JUNCTION_FILLET (FOX_METRES(5) / 2)
/* Two ways through an intersection whose middles come this close carry cars
 * that would touch. */
#define CLOSE_PASS (FOX_METRES(16) / 5)
/* Where two ways cross, a car keeps clear of the other way by as much as
 * the half widths of two cars and a margin take across it, at least this
 * and at most that. */
#define CAR_HALF_WIDTH (FOX_METRES(9) / 10)
#define CROSS_MARGIN (FOX_METRES(3) / 10)
#define MIN_ROOM (FOX_LANE_WIDTH / 2 + FOX_METRES(1))
#define MAX_ROOM FOX_METRES(6)
#define BEND_MARGIN FOX_METRES(1)
#define MIN_BEND_TRIM (FOX_METRES(3) / 2)
/* At a dead end drivers turn round on the road itself: in one sweep where it
 * is wide enough, else to and fro. The turn is planned for a car this long
 * and half as wide, its axles a sixth of the body in from either end, its
 * rear axle turning on this radius at full lock, kept this far off the
 * curbs. A dead end's lanes stop short of its node to leave the room. */
#define TURN_LENGTH (FOX_METRES(9) / 2)
#define TURN_HALF_WIDTH (FOX_METRES(9) / 10)
#define TURN_FRONT (TURN_LENGTH * 5 / 6)
#define TURN_REAR (TURN_LENGTH / 6)
#define TURN_RADIUS FOX_METRES(4)
#define TURN_CURB_ROOM (FOX_METRES(3) / 10)
/* A turn has at most this many legs, sampled every two degrees; the curves
 * leading in and out take this many samples. */
#define TURN_LEGS (FOX_PATH_MAX_CUSPS + 1u)
#define TURN_CURVE_SAMPLES 16u
#define TURN_MAX_POINTS 160u
/* Where the turn may begin: up to this far past the lanes' end, in steps,
 * and how far over to the curb first, in so many steps. */
#define TURN_START_REACH FOX_METRES(20)
#define TURN_START_STEP (FOX_METRES(1) / 2)
#define TURN_SWINGS 4
/* To and fro drivers keep to a slow pace. */
#define MANOEUVRE_SPEED FOX_METRES(3)
/* Through an intersection where one must look out, drivers keep to these:
 * where nobody has priority, and where they give way or stop. */
#define UNSIGNED_SPEED FOX_KMH(30)
#define YIELD_SPEED FOX_KMH(25)

/* The limit of a connection: the lower road's, what its bend allows, and
 * what the sign at its start asks drivers to slow to. */
static int32_t ConnectionSpeed(const FoxNetwork *network, const FoxPath *connection)
{
    const FoxPath *in = &network->paths[connection->from];
    const FoxRoad *from = &network->roads[in->road];
    const FoxRoad *to = &network->roads[network->paths[connection->to].road];
    int64_t speed = FoxMin(FoxMin(from->speedLimit, to->speedLimit), connection->curveSpeed);
    const uint8_t sign = from->sign[in->direction == 0u ? 1u : 0u];
    if (sign == FOX_SIGN_NONE && FoxNodeCarRoads(network, connection->node) >= 3u &&
        !network->nodes[connection->node].signals)
    {
        speed = FoxMin(speed, UNSIGNED_SPEED);
    }
    else if (sign == FOX_SIGN_YIELD || sign == FOX_SIGN_STOP)
    {
        speed = FoxMin(speed, YIELD_SPEED);
    }
    return (int32_t)speed;
}

/* The fastest a path can be driven through its tightest bend: v^2 = a * r,
 * with the radius at each point from the turn between its segments, measured
 * in the plane of the ground. */
static int32_t CurveSpeed(const FoxVec *points, uint32_t count)
{
    int64_t radius = INT64_MAX;
    for (uint32_t index = 1u; index + 1u < count; ++index)
    {
        const FoxVec up = FoxPlanetUp(points[index]);
        FoxVec before = FoxVecSub(points[index], points[index - 1u]);
        FoxVec after = FoxVecSub(points[index + 1u], points[index]);
        before = FoxVecSub(before, FoxVecScale(up, FoxVecDot(before, up), FOX_UNIT * FOX_UNIT));
        after = FoxVecSub(after, FoxVecScale(up, FoxVecDot(after, up), FOX_UNIT * FOX_UNIT));
        const int64_t span = (FoxVecLength(before) + FoxVecLength(after)) / 2;
        const FoxVec a = FoxVecDirection(before);
        const FoxVec b = FoxVecDirection(after);
        if (span <= 0 || FoxVecIsZero(a) || FoxVecIsZero(b))
        {
            continue;
        }
        /* The turn angle in FOX_UNIT, from its sine and whether it turns back. */
        int64_t turn = FoxVecLength(FoxVecScale(FoxVecCross(a, b), 1, FOX_UNIT));
        if (FoxVecDot(a, b) < 0)
        {
            turn = 2 * FOX_UNIT - turn;
        }
        if (turn > 0)
        {
            radius = FoxMin(radius, span * FOX_UNIT / turn);
        }
    }
    if (radius == INT64_MAX)
    {
        return INT32_MAX;
    }
    const int64_t speed = (int64_t)FoxIntegerSqrt((uint64_t)(LATERAL_ACCELERATION * radius));
    return (int32_t)FoxClamp(speed, MIN_CURVE_SPEED, INT32_MAX);
}

/* ---- Lane distance mapping -------------------------------------------------- */

int64_t FoxLaneToCentre(const FoxPath *lane, int64_t distance)
{
    if (lane == NULL || lane->centre == NULL || lane->count == 0u)
    {
        return 0;
    }
    if (distance <= 0 || lane->count == 1u)
    {
        return lane->centre[0];
    }
    if (distance >= lane->length)
    {
        return lane->centre[lane->count - 1u];
    }
    uint32_t low = 0u;
    uint32_t high = lane->count - 1u;
    while (high - low > 1u)
    {
        const uint32_t middle = low + (high - low) / 2u;
        if (lane->lengths[middle] <= distance)
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    const int64_t span = lane->lengths[high] - lane->lengths[low];
    if (span <= 0)
    {
        return lane->centre[low];
    }
    return lane->centre[low] +
           (lane->centre[high] - lane->centre[low]) * (distance - lane->lengths[low]) / span;
}

int64_t FoxCentreToLane(const FoxPath *lane, int64_t centre)
{
    if (lane == NULL || lane->centre == NULL || lane->count == 0u)
    {
        return 0;
    }
    /* Centre distances grow along a forward lane and shrink along a
     * backward one; flip the sign so the search sees them growing. */
    const int64_t sign =
        lane->count > 1u && lane->centre[lane->count - 1u] < lane->centre[0] ? -1 : 1;
    const int64_t target = centre * sign;
    if (lane->count == 1u || target <= lane->centre[0] * sign)
    {
        return 0;
    }
    if (target >= lane->centre[lane->count - 1u] * sign)
    {
        return lane->length;
    }
    uint32_t low = 0u;
    uint32_t high = lane->count - 1u;
    while (high - low > 1u)
    {
        const uint32_t middle = low + (high - low) / 2u;
        if (lane->centre[middle] * sign <= target)
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    const int64_t span = (lane->centre[high] - lane->centre[low]) * sign;
    if (span <= 0)
    {
        return lane->lengths[low];
    }
    return lane->lengths[low] +
           (lane->lengths[high] - lane->lengths[low]) * (target - lane->centre[low] * sign) / span;
}

/* ---- Node measurements ------------------------------------------------------ */

/* Half the width that meets the node: the car lanes, or a path's strips. */
static int64_t HalfWidth(const FoxRoad *road)
{
    const int64_t cars = FoxRoadLayoutCarHalfWidth(&road->driven);
    if (cars != 0)
    {
        return cars;
    }
    int64_t extent[2];
    FoxRoadLayoutExtent(&road->driven, extent);
    return FoxMax(extent[0], extent[1]);
}

/* The way back out of a dead end a road running one way into it leaves its
 * cars: a lone lane is driven both ways, else the innermost lane turns. */
static FoxRoadLayout Driven(const FoxNetwork *network, const FoxRoad *road)
{
    FoxRoadLayout layout = road->layout;
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        /* Lanes of direction 0 run into nodes[1]. */
        const uint32_t into = road->nodes[1u - direction];
        if (layout.lanes[direction] == 0u || layout.lanes[1u - direction] != 0u ||
            FoxNodeCarRoads(network, into) != 1u)
        {
            continue;
        }
        if (layout.lanes[direction] == 1u)
        {
            layout.shared = true;
            layout.lanes[1u - direction] = 1u;
            layout.surface[1u - direction][0] = layout.surface[direction][0];
        }
        else
        {
            --layout.lanes[direction];
            layout.lanes[1u - direction] = 1u;
            layout.surface[1u - direction][0] = layout.surface[direction][layout.lanes[direction]];
        }
    }
    return layout;
}

/* The way back out of a dead end is driven slowly. */
#define WAY_BACK_SPEED FOX_KMH(20)

/* The limit on a lane of a road: the road's, but only the way-back speed on
 * a way back out of a dead end, and no more than its bends allow. */
static int32_t LaneLimit(const FoxRoad *road, uint32_t direction, int32_t curveSpeed)
{
    const bool wayBack = road->layout.lanes[direction] == 0u && road->driven.lanes[direction] != 0u;
    const int64_t limit = wayBack ? FoxMin(road->speedLimit, WAY_BACK_SPEED) : road->speedLimit;
    return (int32_t)FoxMin(limit, curveSpeed);
}

/* The direction a road leaves a node in, FOX_UNIT long. */
static FoxVec LeavingDirection(const FoxRoad *road, uint32_t end)
{
    const int64_t probe = FoxMin(DIRECTION_PROBE, road->length / 2);
    const int64_t along = end == 0u ? probe : road->length - probe;
    const FoxVec point =
        FoxPolylinePoint(road->centre, road->centreLengths, road->centreCount, along, NULL);
    const FoxVec origin = end == 0u ? road->centre[0] : road->centre[road->centreCount - 1u];
    return FoxVecDirection(FoxVecSub(point, origin));
}

uint32_t FoxRoadEndAt(const FoxRoad *road, uint32_t node)
{
    return road->nodes[0] == node ? 0u : 1u;
}

/* How far before a dead end its lanes stop: room for a car to turn round
 * in, to and fro on a street, in one sweep on wider roads. A one-way road
 * has no lanes to turn into. */
static int64_t DeadEndTrim(const FoxRoad *road)
{
    const FoxRoadLayout *layout = &road->driven;
    if (layout->lanes[0] == 0u || layout->lanes[1] == 0u)
    {
        return FOX_METRES(2);
    }
    if (layout->shared)
    {
        return FOX_METRES(23) / 2;
    }
    switch (FoxMax(layout->lanes[0], layout->lanes[1]))
    {
    case 1:
        return FOX_METRES(23) / 2;
    case 2:
        return FOX_METRES(21) / 2;
    default:
        return FOX_METRES(33) / 2;
    }
}

static void PlaneCoordinates(FoxVec direction, FoxVec reference, FoxVec up, int64_t *outX,
                             int64_t *outY);
static bool AngleLess(int64_t ax, int64_t ay, int64_t bx, int64_t by);

void FoxLanesMeasureNode(FoxNetwork *network, uint32_t index)
{
    FoxNode *node = &network->nodes[index];
    const uint32_t count = node->roadCount;
    const uint32_t cars = FoxNodeCarRoads(network, index);
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        FoxRoad *road = &network->roads[node->roads[slot]];
        road->driven = Driven(network, road);
    }
    FoxVec leaving[FOX_NODE_MAX_ROADS];
    int64_t paved[FOX_NODE_MAX_ROADS];
    int64_t trims[FOX_NODE_MAX_ROADS];
    int64_t xs[FOX_NODE_MAX_ROADS];
    int64_t ys[FOX_NODE_MAX_ROADS];
    uint32_t order[FOX_NODE_MAX_ROADS];
    int64_t widest = 0;
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        const FoxRoad *road = &network->roads[node->roads[slot]];
        leaving[slot] = LeavingDirection(road, FoxRoadEndAt(road, index));
        paved[slot] = HalfWidth(road) + FOX_ROAD_SHOULDER;
        widest = FoxMax(widest, HalfWidth(road));
        const bool carRoad = FoxRoadLayoutCarLanes(&road->driven) != 0u;
        trims[slot] = carRoad && cars == 1u ? DeadEndTrim(road) : (count == 2u ? MIN_BEND_TRIM : 0);
        /* Round the node anticlockwise seen from above. */
        PlaneCoordinates(leaving[slot], leaving[0], node->up, &xs[slot], &ys[slot]);
        uint32_t position = slot;
        while (position > 0u &&
               AngleLess(xs[slot], ys[slot], xs[order[position - 1u]], ys[order[position - 1u]]))
        {
            order[position] = order[position - 1u];
            --position;
        }
        order[position] = slot;
    }
    /* Each road reaches only as far as its edges stay clear of the next
     * road round: where the facing edges of two neighbours would meet, plus
     * the room of the curb rounding that corner. Wide angles never close. */
    const int64_t margin = count == 2u ? BEND_MARGIN : JUNCTION_FILLET;
    for (uint32_t k = 0u; count >= 2u && k < count; ++k)
    {
        const uint32_t a = order[k];
        const uint32_t b = order[(k + 1u) % count];
        const int64_t cosine = FoxVecDot(leaving[a], leaving[b]) / FOX_UNIT;
        const int64_t sine = FoxVecDot(FoxVecCross(leaving[a], leaving[b]), node->up) /
                             ((int64_t)FOX_UNIT * FOX_UNIT);
        if (sine <= 0 || cosine < -(int64_t)FOX_UNIT * 866 / 1000)
        {
            continue;
        }
        /* With a along x and b at the angle t: the left edge of a, y = wa,
         * meets the right edge of b where x = (wa cos t + wb) / sin t, and
         * that lies (wa + wb cos t) / sin t along b. */
        const int64_t alongA = (paved[a] * cosine + paved[b] * FOX_UNIT) / sine;
        const int64_t alongB = (paved[a] * FOX_UNIT + paved[b] * cosine) / sine;
        trims[a] = FoxMax(trims[a], FoxMax(0, alongA) + margin);
        trims[b] = FoxMax(trims[b], FoxMax(0, alongB) + margin);
    }
    /* At an intersection every road also keeps out of the width of the
     * roads crossing its way, however wide the angles are. */
    for (uint32_t a = 0u; count >= 3u && a < count; ++a)
    {
        for (uint32_t b = 0u; b < count; ++b)
        {
            const int64_t sine =
                FoxVecLength(FoxVecScale(FoxVecCross(leaving[a], leaving[b]), 1, FOX_UNIT));
            trims[a] = b == a ? trims[a] : FoxMax(trims[a], paved[b] * sine / FOX_UNIT + margin);
        }
    }
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        FoxRoad *road = &network->roads[node->roads[slot]];
        road->trim[FoxRoadEndAt(road, index)] = FoxMin(trims[slot], road->length * 2 / 5);
    }
    node->radius = widest + FOX_METRES(1);
}

/* ---- Lanes ------------------------------------------------------------------- */

static bool AllocatePathArrays(FoxPath *path, uint32_t count, bool centre)
{
    path->points = malloc((size_t)count * sizeof(*path->points));
    path->lengths = malloc((size_t)count * sizeof(*path->lengths));
    path->centre = centre ? malloc((size_t)count * sizeof(*path->centre)) : NULL;
    return path->points != NULL && path->lengths != NULL && (!centre || path->centre != NULL);
}

static void ResetPath(FoxPath *path, uint8_t kind)
{
    const uint32_t generation = path->generation;
    memset(path, 0, sizeof(*path));
    path->generation = generation;
    path->kind = kind;
    path->front = FOX_NETWORK_NONE;
    path->back = FOX_NETWORK_NONE;
    path->from = FOX_NETWORK_NONE;
    path->to = FOX_NETWORK_NONE;
    path->node = FOX_NETWORK_NONE;
    path->road = FOX_NETWORK_NONE;
}

static void FreeNewPath(FoxNetwork *network, uint32_t index)
{
    FoxPath *path = &network->paths[index];
    free(path->points);
    free(path->lengths);
    free(path->centre);
    free(path->outgoing);
    free(path->conflicts);
    free(path->siblings);
    const uint32_t generation = path->generation + 1u;
    memset(path, 0, sizeof(*path));
    path->generation = generation;
}

/* A path of `kind` along a road's centre line from `first` to `last` of
 * it, `offset` units to the right of the centre (seen from nodes[0]
 * towards nodes[1]), running that way. */
static uint32_t BuildAlong(FoxNetwork *network, uint32_t roadIndex, uint8_t kind, int64_t offset,
                           int64_t first, int64_t last)
{
    const uint32_t index = FoxNetworkAllocatePath(network);
    if (index == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
    }
    const FoxRoad *road = &network->roads[roadIndex];
    uint32_t count = 2u;
    for (uint32_t point = 0u; point < road->centreCount; ++point)
    {
        count += road->centreLengths[point] > first && road->centreLengths[point] < last ? 1u : 0u;
    }
    FoxPath *path = &network->paths[index];
    ResetPath(path, kind);
    if (!AllocatePathArrays(path, count, true))
    {
        FreeNewPath(network, index);
        return FOX_NETWORK_NONE;
    }
    uint32_t written = 0u;
    for (uint32_t step = 0u; step < count; ++step)
    {
        int64_t along = 0;
        FoxVec point;
        FoxVec tangent;
        if (step == 0u || step + 1u == count)
        {
            along = step == 0u ? first : last;
            uint32_t segment = 0u;
            point = FoxPolylinePoint(road->centre, road->centreLengths, road->centreCount, along,
                                     &segment);
            tangent = FoxVecSub(road->centre[segment + 1u], road->centre[segment]);
        }
        else
        {
            /* The interior samples in order. */
            uint32_t seen = 0u;
            uint32_t sample = 0u;
            for (sample = 0u; sample < road->centreCount; ++sample)
            {
                if (road->centreLengths[sample] > first && road->centreLengths[sample] < last &&
                    ++seen == step)
                {
                    break;
                }
            }
            along = road->centreLengths[sample];
            point = road->centre[sample];
            const uint32_t before = sample == 0u ? 0u : sample - 1u;
            const uint32_t after = sample + 1u < road->centreCount ? sample + 1u : sample;
            tangent = FoxVecSub(road->centre[after], road->centre[before]);
        }
        const FoxVec right =
            FoxVecDirection(FoxVecCross(FoxVecDirection(tangent), FoxPlanetUp(point)));
        path->points[written] = FoxVecAdd(point, FoxVecScale(right, offset, FOX_UNIT));
        path->centre[written] = along;
        ++written;
    }
    path->count = count;
    path->length = FoxPolylineMeasure(path->points, path->lengths, count);
    path->road = roadIndex;
    path->roadGeneration = road->generation;
    return index;
}

/* One lane along a road's centre line between its trims, `offset` units to
 * the right of the centre (seen from nodes[0] towards nodes[1]). */
static uint32_t BuildLane(FoxNetwork *network, uint32_t roadIndex, uint8_t direction, uint8_t lane,
                          int64_t offset)
{
    const FoxRoad *road = &network->roads[roadIndex];
    const int64_t first = road->trim[0];
    const int64_t last = FoxMax(first + FOX_METRES(1), road->length - road->trim[1]);
    const uint32_t index = BuildAlong(network, roadIndex, PATH_LANE, offset, first, last);
    if (index == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
    }
    FoxPath *path = &network->paths[index];
    const uint32_t count = path->count;
    if (direction == 1u)
    {
        for (uint32_t low = 0u, high = count - 1u; low < high; ++low, --high)
        {
            const FoxVec point = path->points[low];
            path->points[low] = path->points[high];
            path->points[high] = point;
            const int64_t along = path->centre[low];
            path->centre[low] = path->centre[high];
            path->centre[high] = along;
        }
    }
    path->length = FoxPolylineMeasure(path->points, path->lengths, count);
    path->curveSpeed = CurveSpeed(path->points, count);
    path->speedLimit = LaneLimit(road, direction, path->curveSpeed);
    path->direction = direction;
    path->lane = lane;
    return index;
}

/* ---- Walks ------------------------------------------------------------------- */

/* Makes two walk ends meet. */
static void Join(FoxNetwork *network, uint32_t a, uint32_t aEnd, uint32_t b, uint32_t bEnd)
{
    FoxPath *first = &network->paths[a];
    FoxPath *second = &network->paths[b];
    if (first->joinCount[aEnd] < FOX_WALK_MAX_JOINS && second->joinCount[bEnd] < FOX_WALK_MAX_JOINS)
    {
        first->joins[aEnd][first->joinCount[aEnd]++] = FOX_WALK_END(b, bEnd);
        second->joins[bEnd][second->joinCount[bEnd]++] = FOX_WALK_END(a, aEnd);
    }
}

void FoxLanesRetireWalks(FoxNetwork *network, FoxRoad *road)
{
    for (uint32_t side = 0u; side < 2u; ++side)
    {
        for (uint32_t piece = 0u; piece < road->walkCount[side]; ++piece)
        {
            (void)FoxNetworkRetirePath(network, road->walks[side][piece]);
        }
        road->walkCount[side] = 0u;
    }
    for (uint32_t crossing = 0u; crossing < FOX_ROAD_MAX_CROSSINGS; ++crossing)
    {
        if (road->crossingWalks[crossing] != FOX_NETWORK_NONE)
        {
            (void)FoxNetworkRetirePath(network, road->crossingWalks[crossing]);
        }
        road->crossingWalks[crossing] = FOX_NETWORK_NONE;
    }
}

/* The middle of a road's footway on one side, units right of its centre
 * line; false without one. */
static bool FootwayOffset(const FoxRoad *road, uint32_t side, int64_t *outOffset)
{
    if (!road->layout.foot[side])
    {
        return false;
    }
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t count =
        FoxRoadLayoutStrips(&road->layout, (FoxDriveSide)road->driveSide, strips);
    const FoxRoadStrip *strip = &strips[side == 0u ? 0u : count - 1u];
    *outOffset = (strip->left + strip->right) / 2;
    return true;
}

/* The footways along a road, cut where crossings lead over it, and the walks
 * over those crossings; and on every lane where it is crossed. */
static bool BuildWalks(FoxNetwork *network, uint32_t roadIndex)
{
    FoxRoad *road = &network->roads[roadIndex];
    FoxLanesRetireWalks(network, road);
    const int64_t first = road->trim[0];
    const int64_t last = FoxMax(first + FOX_METRES(1), road->length - road->trim[1]);
    /* The crossings between the trims, in order. */
    int64_t cuts[FOX_ROAD_MAX_CROSSINGS];
    uint32_t cutIndex[FOX_ROAD_MAX_CROSSINGS];
    uint32_t cutCount = 0u;
    for (uint32_t crossing = 0u; crossing < road->crossingCount; ++crossing)
    {
        if (road->crossings[crossing] > first + FOX_METRES(1) &&
            road->crossings[crossing] < last - FOX_METRES(1))
        {
            cutIndex[cutCount] = crossing;
            cuts[cutCount++] = road->crossings[crossing];
        }
    }
    bool ok = true;
    int64_t offsets[2] = {0, 0};
    bool footways[2] = {false, false};
    for (uint32_t side = 0u; side < 2u; ++side)
    {
        footways[side] = FootwayOffset(road, side, &offsets[side]);
        for (uint32_t piece = 0u; footways[side] && piece <= cutCount; ++piece)
        {
            const int64_t from = piece == 0u ? first : cuts[piece - 1u];
            const int64_t to = piece == cutCount ? last : cuts[piece];
            const uint32_t walk =
                BuildAlong(network, roadIndex, PATH_WALK, offsets[side], from, to);
            road = &network->roads[roadIndex];
            if (walk == FOX_NETWORK_NONE)
            {
                ok = false;
                break;
            }
            network->paths[walk].walkKind = WALK_FOOTWAY;
            if (piece != 0u)
            {
                Join(network, road->walks[side][piece - 1u], 1u, walk, 0u);
            }
            road->walks[side][road->walkCount[side]++] = walk;
        }
    }
    for (uint32_t cut = 0u;
         ok && footways[0] && footways[1] && road->walkCount[0] == cutCount + 1u &&
         road->walkCount[1] == cutCount + 1u && cut < cutCount;
         ++cut)
    {
        const uint32_t index = FoxNetworkAllocatePath(network);
        road = &network->roads[roadIndex];
        if (index == FOX_NETWORK_NONE)
        {
            ok = false;
            break;
        }
        FoxPath *walk = &network->paths[index];
        ResetPath(walk, PATH_WALK);
        if (!AllocatePathArrays(walk, 2u, false))
        {
            FreeNewPath(network, index);
            ok = false;
            break;
        }
        /* Straight over, from the left footway to the right one. */
        walk->points[0] = network->paths[road->walks[0][cut]]
                              .points[network->paths[road->walks[0][cut]].count - 1u];
        walk->points[1] = network->paths[road->walks[1][cut]]
                              .points[network->paths[road->walks[1][cut]].count - 1u];
        walk->count = 2u;
        walk->length = FoxPolylineMeasure(walk->points, walk->lengths, 2u);
        walk->walkKind = WALK_CROSSING;
        walk->road = roadIndex;
        walk->roadGeneration = road->generation;
        walk->roadAlong = cuts[cut];
        for (uint32_t side = 0u; side < 2u; ++side)
        {
            const uint32_t before = road->walks[side][cut];
            const uint32_t after = road->walks[side][cut + 1u];
            Join(network, index, side, before, 1u);
            Join(network, index, side, after, 0u);
        }
        road->crossingWalks[cutIndex[cut]] = index;
    }
    /* Every lane knows where it is crossed, and whether anyone walks there. */
    road = &network->roads[roadIndex];
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
        {
            FoxPath *path = &network->paths[road->lanes[direction][lane]];
            path->crossingCount = 0u;
            for (uint32_t cut = 0u; cut < cutCount; ++cut)
            {
                const uint32_t at = direction == 0u ? cut : cutCount - 1u - cut;
                path->crossingAt[path->crossingCount] = FoxCentreToLane(path, cuts[at]);
                path->crossingWalk[path->crossingCount++] = road->crossingWalks[cutIndex[at]];
            }
        }
    }
    return ok;
}

/* The walk round each corner of a node, from the footway along the left of
 * one road leaving it to the footway along the right of the next one round
 * anticlockwise: the pavement between the two. */
static bool BuildCorners(FoxNetwork *network, uint32_t nodeIndex)
{
    FoxNode *node = &network->nodes[nodeIndex];
    const uint32_t count = node->roadCount;
    /* The footway ends at this node meet only the corners built now. */
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        FoxRoad *road = &network->roads[node->roads[slot]];
        const uint32_t end = FoxRoadEndAt(road, nodeIndex);
        for (uint32_t side = 0u; side < 2u; ++side)
        {
            if (road->walkCount[side] != 0u)
            {
                const uint32_t piece = end == 0u ? 0u : road->walkCount[side] - 1u;
                network->paths[road->walks[side][piece]].joinCount[end] = 0u;
            }
        }
    }
    if (count < 2u)
    {
        return true;
    }
    int64_t xs[FOX_NODE_MAX_ROADS];
    int64_t ys[FOX_NODE_MAX_ROADS];
    uint32_t order[FOX_NODE_MAX_ROADS];
    FoxVec first = {0, 0, 0};
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        const FoxRoad *road = &network->roads[node->roads[slot]];
        const FoxVec leaving = LeavingDirection(road, FoxRoadEndAt(road, nodeIndex));
        first = slot == 0u ? leaving : first;
        PlaneCoordinates(leaving, first, node->up, &xs[slot], &ys[slot]);
        uint32_t position = slot;
        while (position > 0u &&
               AngleLess(xs[slot], ys[slot], xs[order[position - 1u]], ys[order[position - 1u]]))
        {
            order[position] = order[position - 1u];
            --position;
        }
        order[position] = slot;
    }
    uint32_t *corners = malloc((size_t)count * sizeof(*corners));
    if (corners == NULL)
    {
        return false;
    }
    bool ok = true;
    uint32_t built = 0u;
    for (uint32_t k = 0u; k < count; ++k)
    {
        const uint32_t a = node->roads[order[k]];
        const uint32_t b = node->roads[order[(k + 1u) % count]];
        const FoxRoad *roadA = &network->roads[a];
        const FoxRoad *roadB = &network->roads[b];
        const uint32_t endA = FoxRoadEndAt(roadA, nodeIndex);
        const uint32_t endB = FoxRoadEndAt(roadB, nodeIndex);
        /* Leaving from nodes[0] the left is side 0; leaving from nodes[1] it
         * is side 1. */
        const uint32_t sideA = endA == 0u ? 0u : 1u;
        const uint32_t sideB = endB == 0u ? 1u : 0u;
        if (roadA->walkCount[sideA] == 0u || roadB->walkCount[sideB] == 0u)
        {
            continue;
        }
        const uint32_t walkA = roadA->walks[sideA][endA == 0u ? 0u : roadA->walkCount[sideA] - 1u];
        const uint32_t walkB = roadB->walks[sideB][endB == 0u ? 0u : roadB->walkCount[sideB] - 1u];
        const uint32_t index = FoxNetworkAllocatePath(network);
        if (index == FOX_NETWORK_NONE)
        {
            ok = false;
            break;
        }
        FoxPath *walk = &network->paths[index];
        ResetPath(walk, PATH_WALK);
        if (!AllocatePathArrays(walk, 2u, false))
        {
            FreeNewPath(network, index);
            ok = false;
            break;
        }
        const FoxPath *pathA = &network->paths[walkA];
        const FoxPath *pathB = &network->paths[walkB];
        walk->points[0] = pathA->points[endA == 0u ? 0u : pathA->count - 1u];
        walk->points[1] = pathB->points[endB == 0u ? 0u : pathB->count - 1u];
        walk->count = 2u;
        walk->length = FoxPolylineMeasure(walk->points, walk->lengths, 2u);
        walk->walkKind = WALK_CORNER;
        walk->node = nodeIndex;
        Join(network, index, 0u, walkA, endA);
        Join(network, index, 1u, walkB, endB);
        corners[built++] = index;
    }
    node = &network->nodes[nodeIndex];
    node->corners = corners;
    node->cornerCount = built;
    return ok;
}

bool FoxLanesBuildRoad(FoxNetwork *network, uint32_t roadIndex)
{
    FoxRoad *road = &network->roads[roadIndex];
    bool ok = true;
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
        {
            ok = FoxNetworkRetirePath(network, road->lanes[direction][lane]) && ok;
            road->lanes[direction][lane] = FOX_NETWORK_NONE;
        }
        road->laneCount[direction] = 0u;
    }
    road->driven = Driven(network, road);
    const FoxRoadLayout layout = road->driven;
    for (uint8_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint8_t lane = 0u; lane < layout.lanes[direction]; ++lane)
        {
            const int64_t offset =
                FoxRoadLaneOffset(&layout, (FoxDriveSide)road->driveSide, direction, lane);
            const uint32_t path = BuildLane(network, roadIndex, direction, lane, offset);
            if (path == FOX_NETWORK_NONE)
            {
                ok = false;
                continue;
            }
            road = &network->roads[roadIndex];
            road->lanes[direction][road->laneCount[direction]++] = path;
        }
    }
    return BuildWalks(network, roadIndex) && ok;
}

/* ---- Connections ------------------------------------------------------------- */

typedef struct NodeEnd
{
    /* Into the node along the incoming lanes, and out along the outgoing. */
    FoxVec approach;
    FoxVec leave;
    uint32_t road;
    uint32_t inCount;
    uint32_t outCount;
    uint32_t in[FOX_ROAD_MAX_LANES];
    uint32_t out[FOX_ROAD_MAX_LANES];
    uint8_t rank;
} NodeEnd;

static FoxVec PathEndDirection(const FoxPath *path, bool atEnd)
{
    const uint32_t a = atEnd ? path->count - 2u : 0u;
    return FoxVecDirection(FoxVecSub(path->points[a + 1u], path->points[a]));
}

/* Order of directions by angle from -pi to pi measured from `reference`. */
static void PlaneCoordinates(FoxVec direction, FoxVec reference, FoxVec up, int64_t *outX,
                             int64_t *outY)
{
    const FoxVec left = FoxVecScale(FoxVecCross(up, reference), 1, FOX_UNIT);
    *outX = FoxVecDot(direction, reference) / FOX_UNIT;
    *outY = FoxVecDot(direction, left) / FOX_UNIT;
}

static int AngleGroup(int64_t x, int64_t y)
{
    if (y < 0)
    {
        return 0;
    }
    if (y == 0 && x > 0)
    {
        return 1;
    }
    return y > 0 ? 2 : 3;
}

/* True when direction a lies at a smaller angle than b, angles in (-pi, pi]. */
static bool AngleLess(int64_t ax, int64_t ay, int64_t bx, int64_t by)
{
    const int groupA = AngleGroup(ax, ay);
    const int groupB = AngleGroup(bx, by);
    if (groupA != groupB)
    {
        return groupA < groupB;
    }
    return ax * by - ay * bx > 0;
}

static uint8_t Classify(int64_t x, int64_t y, uint8_t side)
{
    if (x > 0 && FoxAbs(y) * 10 < x * 7)
    {
        return MOVE_STRAIGHT;
    }
    const bool right = y < 0;
    return (right == (side == FOX_DRIVE_RIGHT)) ? MOVE_CURB : MOVE_FAR;
}

static FoxVec Cubic(FoxVec p0, FoxVec p1, FoxVec p2, FoxVec p3, int64_t t)
{
    const FoxVec a = FoxVecLerp(p0, p1, t);
    const FoxVec b = FoxVecLerp(p1, p2, t);
    const FoxVec c = FoxVecLerp(p2, p3, t);
    return FoxVecLerp(FoxVecLerp(a, b, t), FoxVecLerp(b, c, t), t);
}

/* The tangent handles of a cubic between two points and directions. */
static void Handles(FoxVec p0, FoxVec t0, FoxVec p3, FoxVec t3, FoxVec *outP1, FoxVec *outP2)
{
    const int64_t reach = FoxMax(FoxVecDistance(p0, p3) * 2 / 5, FOX_METRES(1));
    *outP1 = FoxVecAdd(p0, FoxVecScale(t0, reach, FOX_UNIT));
    *outP2 = FoxVecSub(p3, FoxVecScale(t3, reach, FOX_UNIT));
}

/* sin(degree) in FOX_UNIT for 0..90 degrees. */
static const int32_t sines[91] = {
    0,     1144,  2287,  3430,  4572,  5712,  6850,  7987,  9121,  10252, 11380, 12505, 13626,
    14742, 15855, 16962, 18064, 19161, 20252, 21336, 22415, 23486, 24550, 25607, 26656, 27697,
    28729, 29753, 30767, 31772, 32768, 33754, 34729, 35693, 36647, 37590, 38521, 39441, 40348,
    41243, 42126, 42995, 43852, 44695, 45525, 46341, 47143, 47930, 48703, 49461, 50203, 50931,
    51643, 52339, 53020, 53684, 54332, 54963, 55578, 56175, 56756, 57319, 57865, 58393, 58903,
    59396, 59870, 60326, 60764, 61183, 61584, 61966, 62328, 62672, 62997, 63303, 63589, 63856,
    64104, 64332, 64540, 64729, 64898, 65048, 65177, 65287, 65376, 65446, 65496, 65526, 65536,
};

static int64_t Sine(int32_t degree)
{
    const int32_t angle = ((degree % 360) + 360) % 360;
    if (angle <= 90)
    {
        return sines[angle];
    }
    if (angle <= 180)
    {
        return sines[180 - angle];
    }
    return angle <= 270 ? -sines[angle - 180] : -sines[360 - angle];
}

static int64_t Cosine(int32_t degree)
{
    return Sine(degree + 90);
}

/* A point in the plane of a dead end: x along the lanes arriving there,
 * from where their centre line ends, and y from the centre line towards
 * the far side. */
typedef struct TurnPoint
{
    int64_t x;
    int64_t y;
} TurnPoint;

/* The room to turn in: the node lies `node` along x, and the paved road and
 * its rounded end reach `edge` from the centre line, less the curb room. */
typedef struct TurnRoom
{
    int64_t node;
    int64_t edge;
} TurnRoom;

/* The rear axle's way through a turn and the points where the car stops to
 * change between driving forward and reversing. */
typedef struct TurnPlan
{
    TurnPoint points[TURN_MAX_POINTS];
    uint32_t count;
    uint32_t cusps[FOX_PATH_MAX_CUSPS];
    uint32_t cuspCount;
} TurnPlan;

static bool TurnAppend(TurnPlan *plan, TurnPoint point)
{
    if (plan->count == TURN_MAX_POINTS)
    {
        return false;
    }
    plan->points[plan->count++] = point;
    return true;
}

/* Whether the car with its rear axle at `at`, heading (hx, hy) in FOX_UNIT,
 * stays on the road and out of the lanes leading in, where the next car
 * waits; while it leads in it is still in its own lane. */
static bool BodyFits(const TurnRoom *room, TurnPoint at, int64_t hx, int64_t hy, bool leading)
{
    static const int64_t lengths[2] = {TURN_FRONT, -TURN_REAR};
    static const int64_t widths[2] = {TURN_HALF_WIDTH, -TURN_HALF_WIDTH};
    for (uint32_t end = 0u; end < 2u; ++end)
    {
        for (uint32_t side = 0u; side < 2u; ++side)
        {
            const int64_t x = at.x + (lengths[end] * hx - widths[side] * hy) / FOX_UNIT;
            const int64_t y = at.y + (lengths[end] * hy + widths[side] * hx) / FOX_UNIT;
            const int64_t past = x - room->node;
            if (FoxAbs(y) > room->edge ||
                (past > 0 && past * past + y * y > room->edge * room->edge) ||
                (!leading && x < 0 && y < 0))
            {
                return false;
            }
        }
    }
    return true;
}

/* A cubic from one point and heading to another: false where it bends
 * tighter than the car steers or leaves the room. The samples after `from`
 * are appended. */
static bool TurnCurve(const TurnRoom *room, TurnPoint from, int64_t fx, int64_t fy, TurnPoint to,
                      int64_t tx, int64_t ty, bool leading, TurnPlan *plan)
{
    const int64_t dx = to.x - from.x;
    const int64_t dy = to.y - from.y;
    const int64_t reach = (int64_t)FoxIntegerSqrt((uint64_t)(dx * dx + dy * dy)) * 2 / 5;
    const TurnPoint handles[2] = {
        {from.x + fx * reach / FOX_UNIT, from.y + fy * reach / FOX_UNIT},
        {to.x - tx * reach / FOX_UNIT, to.y - ty * reach / FOX_UNIT},
    };
    TurnPoint previous = from;
    int64_t px = fx;
    int64_t py = fy;
    for (uint32_t sample = 1u; sample <= TURN_CURVE_SAMPLES; ++sample)
    {
        const int64_t t = (int64_t)sample * FOX_UNIT / TURN_CURVE_SAMPLES;
        const int64_t u = FOX_UNIT - t;
        const int64_t weights[4] = {
            u * u / FOX_UNIT * u / FOX_UNIT,
            3 * u * u / FOX_UNIT * t / FOX_UNIT,
            3 * u * t / FOX_UNIT * t / FOX_UNIT,
            t * t / FOX_UNIT * t / FOX_UNIT,
        };
        const TurnPoint point = sample == TURN_CURVE_SAMPLES
                                    ? to
                                    : (TurnPoint){(weights[0] * from.x + weights[1] * handles[0].x +
                                                   weights[2] * handles[1].x + weights[3] * to.x) /
                                                      FOX_UNIT,
                                                  (weights[0] * from.y + weights[1] * handles[0].y +
                                                   weights[2] * handles[1].y + weights[3] * to.y) /
                                                      FOX_UNIT};
        const int64_t sx = point.x - previous.x;
        const int64_t sy = point.y - previous.y;
        const int64_t span = (int64_t)FoxIntegerSqrt((uint64_t)(sx * sx + sy * sy));
        if (span == 0)
        {
            continue;
        }
        const int64_t hx = sx * FOX_UNIT / span;
        const int64_t hy = sy * FOX_UNIT / span;
        /* The turn from the last span to this one, against what the radius
         * allows over this span. */
        const int64_t turn = FoxAbs(px * hy - py * hx) / FOX_UNIT;
        if (px * hx + py * hy < 0 || span * FOX_UNIT * 20 < TURN_RADIUS * 19 * turn ||
            !BodyFits(room, point, hx, hy, leading) || !TurnAppend(plan, point))
        {
            return false;
        }
        previous = point;
        px = hx;
        py = hy;
    }
    /* It leaves the curve the way the lane out runs. */
    return px * tx + py * ty >= 0;
}

/* The rear axle on the circle about `centre` at a heading: driving forward
 * the circle lies to the left, reversing to the right, and either way the
 * heading turns left as the car goes. */
static TurnPoint OnTurn(TurnPoint centre, int32_t degree, bool forward)
{
    const int64_t side = forward ? 1 : -1;
    return (TurnPoint){centre.x + side * TURN_RADIUS * Sine(degree) / FOX_UNIT,
                       centre.y - side * TURN_RADIUS * Cosine(degree) / FOX_UNIT};
}

/* A turn as a driver makes it: on from its lane, `swing` of the way over
 * to the curb, up to `start`, then at full lock, left going forward and
 * right in reverse, each leg as far as the road allows, until it can bend
 * into the lane out at `end`, heading (ex, ey). */
static bool PlanTurn(const TurnRoom *room, TurnPoint in, TurnPoint end, int64_t ex, int64_t ey,
                     int32_t swing, int64_t start, TurnPlan *plan)
{
    plan->count = 0u;
    plan->cuspCount = 0u;
    const int64_t curb = FoxMin(in.y, -(room->edge - TURN_HALF_WIDTH - FOX_METRES(1) / 10));
    TurnPoint at = {start, in.y + (curb - in.y) * swing / TURN_SWINGS};
    if (!TurnAppend(plan, in) || !TurnCurve(room, in, FOX_UNIT, 0, at, FOX_UNIT, 0, true, plan))
    {
        return false;
    }
    int32_t degree = 0;
    for (uint32_t leg = 0u; leg < TURN_LEGS; ++leg)
    {
        const bool forward = leg % 2u == 0u;
        const int64_t side = forward ? 1 : -1;
        const TurnPoint centre = {at.x - side * TURN_RADIUS * Sine(degree) / FOX_UNIT,
                                  at.y + side * TURN_RADIUS * Cosine(degree) / FOX_UNIT};
        uint32_t steps = 0u;
        for (;;)
        {
            const int64_t behind = ((end.x - at.x) * ex + (end.y - at.y) * ey) / FOX_UNIT;
            if (forward && degree >= 150 && behind > FOX_METRES(1))
            {
                const uint32_t mark = plan->count;
                if (TurnCurve(room, at, Cosine(degree), Sine(degree), end, ex, ey, false, plan))
                {
                    return true;
                }
                plan->count = mark;
            }
            if (degree >= 180)
            {
                break;
            }
            const TurnPoint next = OnTurn(centre, degree + 1, forward);
            if (!BodyFits(room, next, Cosine(degree + 1), Sine(degree + 1), false))
            {
                break;
            }
            at = next;
            ++degree;
            ++steps;
            if (degree % 2 == 0 && !TurnAppend(plan, at))
            {
                return false;
            }
        }
        if (steps == 0u || plan->cuspCount == FOX_PATH_MAX_CUSPS)
        {
            return false;
        }
        const TurnPoint last = plan->points[plan->count - 1u];
        if ((last.x != at.x || last.y != at.y) && !TurnAppend(plan, at))
        {
            return false;
        }
        plan->cusps[plan->cuspCount++] = plan->count - 1u;
    }
    return false;
}

/* A dead end's turn from the lane `in` into the lane `out`: the way of the
 * rear axle, from where it is as the front reaches the end of the lane in
 * to where it is as the front reaches the start of the lane out. Of the
 * turns that fit, the one with the fewest legs; false when none does. The
 * plane's origin and axes are written out. */
static bool DeadEndTurn(const FoxNetwork *network, uint32_t nodeIndex, const FoxPath *in,
                        const FoxPath *out, TurnPlan *best, FoxVec *outOrigin, FoxVec *outForward,
                        FoxVec *outSide)
{
    const FoxNode *node = &network->nodes[nodeIndex];
    const FoxRoad *road = &network->roads[in->road];
    const FoxVec forward = PathEndDirection(in, true);
    /* Towards the far side: left where traffic keeps right. */
    FoxVec side = FoxVecScale(FoxVecCross(node->up, forward), 1, FOX_UNIT);
    if (node->driveSide == FOX_DRIVE_LEFT)
    {
        side = FoxVecNegate(side);
    }
    const FoxVec origin = FoxPolylinePoint(road->centre, road->centreLengths, road->centreCount,
                                           in->centre[in->count - 1u], NULL);
    const FoxVec laneEnd = FoxVecSub(in->points[in->count - 1u], origin);
    const FoxVec outStart = FoxVecSub(out->points[0], origin);
    const FoxVec outward = PathEndDirection(out, false);
    const TurnRoom room = {
        .node = FoxVecDot(FoxVecSub(node->position, origin), forward) / FOX_UNIT,
        .edge = HalfWidth(road) + FOX_ROAD_SHOULDER - TURN_CURB_ROOM,
    };
    const TurnPoint start = {FoxVecDot(laneEnd, forward) / FOX_UNIT - TURN_FRONT,
                             FoxVecDot(laneEnd, side) / FOX_UNIT};
    const int64_t ex = FoxVecDot(outward, forward) / FOX_UNIT;
    const int64_t ey = FoxVecDot(outward, side) / FOX_UNIT;
    const TurnPoint end = {FoxVecDot(outStart, forward) / FOX_UNIT - ex * TURN_FRONT / FOX_UNIT,
                           FoxVecDot(outStart, side) / FOX_UNIT - ey * TURN_FRONT / FOX_UNIT};
    TurnPlan *trial = malloc(sizeof(*trial));
    if (trial == NULL)
    {
        return false;
    }
    bool found = false;
    for (int32_t swing = 0; swing <= TURN_SWINGS; ++swing)
    {
        for (int64_t at = start.x + TURN_FRONT + FOX_METRES(1); at <= TURN_START_REACH;
             at += TURN_START_STEP)
        {
            if (PlanTurn(&room, start, end, ex, ey, swing, at, trial) &&
                (!found || trial->cuspCount < best->cuspCount))
            {
                *best = *trial;
                found = true;
            }
        }
    }
    free(trial);
    *outOrigin = origin;
    *outForward = forward;
    *outSide = side;
    return found;
}

static uint32_t BuildConnection(FoxNetwork *network, uint32_t nodeIndex, uint32_t from, uint32_t to,
                                uint8_t movement)
{
    const uint32_t index = FoxNetworkAllocatePath(network);
    if (index == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
    }
    const FoxPath *in = &network->paths[from];
    const FoxPath *out = &network->paths[to];
    const FoxVec p0 = in->points[in->count - 1u];
    const FoxVec p3 = out->points[0];
    FoxVec p1;
    FoxVec p2;
    Handles(p0, PathEndDirection(in, true), p3, PathEndDirection(out, false), &p1, &p2);
    const int64_t estimate =
        FoxVecDistance(p0, p1) + FoxVecDistance(p1, p2) + FoxVecDistance(p2, p3);
    uint32_t count = (uint32_t)FoxClamp(estimate / FOX_METRES(1) + 1, CONNECTION_MIN_SAMPLES,
                                        CONNECTION_MAX_SAMPLES);
    /* A dead end's turn; where none fits, a plain curve round. */
    TurnPlan *turn = NULL;
    FoxVec origin = {0};
    FoxVec forward = {0};
    FoxVec side = {0};
    if (movement == MOVE_U_TURN)
    {
        turn = malloc(sizeof(*turn));
        if (turn != NULL &&
            !DeadEndTurn(network, nodeIndex, in, out, turn, &origin, &forward, &side))
        {
            free(turn);
            turn = NULL;
        }
        count = turn != NULL ? turn->count : count;
    }

    FoxPath *path = &network->paths[index];
    ResetPath(path, PATH_CONNECTION);
    if (!AllocatePathArrays(path, count, false))
    {
        free(turn);
        FreeNewPath(network, index);
        return FOX_NETWORK_NONE;
    }
    for (uint32_t sample = 0u; sample < count; ++sample)
    {
        path->points[sample] =
            turn != NULL
                ? FoxVecAdd(origin,
                            FoxVecAdd(FoxVecScale(forward, turn->points[sample].x, FOX_UNIT),
                                      FoxVecScale(side, turn->points[sample].y, FOX_UNIT)))
                : Cubic(p0, p1, p2, p3, (int64_t)sample * FOX_UNIT / (int64_t)(count - 1u));
    }
    path->count = count;
    path->length = FoxMax(1, FoxPolylineMeasure(path->points, path->lengths, count));
    path->curveSpeed = CurveSpeed(path->points, count);
    path->node = nodeIndex;
    path->from = from;
    path->to = to;
    path->movement = movement;
    if (turn != NULL)
    {
        /* The points trace the rear axle. Where the legs meet the car stops
         * to change direction, so that bend sets no pace; to and fro it
         * keeps to a walking pace. */
        path->axle = true;
        path->cuspCount = (uint8_t)turn->cuspCount;
        for (uint32_t cusp = 0u; cusp < turn->cuspCount; ++cusp)
        {
            path->cusps[cusp] = path->lengths[turn->cusps[cusp]];
        }
        if (turn->cuspCount != 0u)
        {
            path->curveSpeed = MANOEUVRE_SPEED;
        }
        free(turn);
    }
    path->speedLimit = ConnectionSpeed(network, path);
    return index;
}

static bool AddOutgoing(FoxPath *lane, uint32_t connection)
{
    uint32_t *grown = realloc(lane->outgoing, (size_t)(lane->outgoingCount + 1u) * sizeof(*grown));
    if (grown == NULL)
    {
        return false;
    }
    lane->outgoing = grown;
    lane->outgoing[lane->outgoingCount++] = connection;
    return true;
}

static bool AddNodeConnection(FoxNode *node, uint32_t connection)
{
    uint32_t *grown =
        realloc(node->connections, (size_t)(node->connectionCount + 1u) * sizeof(*grown));
    if (grown == NULL)
    {
        return false;
    }
    node->connections = grown;
    node->connections[node->connectionCount++] = connection;
    return true;
}

static bool Connect(FoxNetwork *network, uint32_t nodeIndex, uint32_t from, uint32_t to,
                    uint8_t movement)
{
    const uint32_t connection = BuildConnection(network, nodeIndex, from, to, movement);
    if (connection == FOX_NETWORK_NONE)
    {
        return false;
    }
    if (!AddNodeConnection(&network->nodes[nodeIndex], connection))
    {
        FreeNewPath(network, connection);
        return false;
    }
    return AddOutgoing(&network->paths[from], connection);
}

/* Joins incoming lanes [first, first + lanes) to the `targets` outgoing
 * lanes of one road keeping their order: lane i goes to the targets its
 * share covers. Turning into a road with more lanes than turn, drivers keep
 * to the side they turn from, as the rules ask: towards the curb into the
 * lanes nearest it, across into the innermost. */
static bool ConnectGroup(FoxNetwork *network, uint32_t nodeIndex, const uint32_t *incoming,
                         uint32_t first, uint32_t lanes, const uint32_t *outgoing, uint32_t targets,
                         uint8_t movement)
{
    bool ok = true;
    for (uint32_t lane = 0u; lane < lanes; ++lane)
    {
        uint32_t low = lane * targets / lanes;
        uint32_t high = (lane + 1u) * targets / lanes;
        if (high <= low)
        {
            high = low + 1u;
        }
        if (lanes < targets && (movement == MOVE_CURB || movement == MOVE_FAR))
        {
            low = movement == MOVE_CURB ? lane : targets - lanes + lane;
            high = low + 1u;
        }
        for (uint32_t target = low; target < high && target < targets; ++target)
        {
            ok = Connect(network, nodeIndex, incoming[first + lane], outgoing[target], movement) &&
                 ok;
        }
    }
    return ok;
}

/* The last segment of b that crosses a, or -1 when they never cross. */
static int32_t LastCrossing(const int64_t *ax, const int64_t *ay, uint32_t aCount,
                            const int64_t *bx, const int64_t *by, uint32_t bCount)
{
    int32_t last = -1;
    for (uint32_t i = 0u; i + 1u < aCount; ++i)
    {
        for (uint32_t j = 0u; j + 1u < bCount; ++j)
        {
            const int64_t d1x = ax[i + 1u] - ax[i];
            const int64_t d1y = ay[i + 1u] - ay[i];
            const int64_t d2x = bx[j + 1u] - bx[j];
            const int64_t d2y = by[j + 1u] - by[j];
            const int64_t o1 = d1x * (by[j] - ay[i]) - d1y * (bx[j] - ax[i]);
            const int64_t o2 = d1x * (by[j + 1u] - ay[i]) - d1y * (bx[j + 1u] - ax[i]);
            const int64_t o3 = d2x * (ay[i] - by[j]) - d2y * (ax[i] - bx[j]);
            const int64_t o4 = d2x * (ay[i + 1u] - by[j]) - d2y * (ax[i + 1u] - bx[j]);
            if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
                ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)) && (int32_t)j > last)
            {
                last = (int32_t)j;
            }
        }
    }
    return last;
}

/* Where two ways that never cross come closer than `reach`, so that the
 * cars on them would touch: the sample of b nearest to a, and of a nearest
 * to b, or -1 when they keep apart. */
static int32_t ClosestPass(const int64_t *ax, const int64_t *ay, const int64_t *bx,
                           const int64_t *by, uint32_t count, int64_t reach, int32_t *outA)
{
    int32_t found = -1;
    int64_t best = reach * reach;
    for (uint32_t i = 0u; i < count; ++i)
    {
        for (uint32_t j = 0u; j < count; ++j)
        {
            const int64_t dx = ax[i] - bx[j];
            const int64_t dy = ay[i] - by[j];
            if (dx * dx + dy * dy < best)
            {
                best = dx * dx + dy * dy;
                found = (int32_t)j;
                *outA = (int32_t)i;
            }
        }
    }
    return found;
}

/* The stretch of way a, from `outFirst` to `outLast` along it, where its
 * middle comes closer to way b than two cars' half widths and a margin, so
 * that cars there would touch; false where it never does. The ways are
 * sampled in plane coordinates, a's `count` samples to its own length. */
static bool TouchingStretch(const FoxPath *a, const int64_t *ax, const int64_t *ay,
                            const int64_t *bx, const int64_t *by, int64_t *outFirst,
                            int64_t *outLast)
{
    const int64_t reach = (2 * CAR_HALF_WIDTH + CROSS_MARGIN) / 4;
    bool found = false;
    for (uint32_t i = 0u; i < a->count && i < CONNECTION_MAX_SAMPLES; ++i)
    {
        bool close = false;
        for (uint32_t j = 0u; j + 1u < CONNECTION_MAX_SAMPLES && !close; ++j)
        {
            /* The nearest point of the segment j of b, by projection. */
            const int64_t sx = bx[j + 1u] - bx[j];
            const int64_t sy = by[j + 1u] - by[j];
            const int64_t px = ax[i] - bx[j];
            const int64_t py = ay[i] - by[j];
            const int64_t span = sx * sx + sy * sy;
            const int64_t along = span == 0 ? 0 : FoxClamp(px * sx + py * sy, 0, span);
            const int64_t dx = span == 0 ? px : px - sx * along / span;
            const int64_t dy = span == 0 ? py : py - sy * along / span;
            close = dx * dx + dy * dy < reach * reach;
        }
        if (close)
        {
            *outLast = a->lengths[i];
            if (!found)
            {
                *outFirst = a->lengths[i];
                found = true;
            }
        }
    }
    return found;
}

/* The room a car keeps from a way it crosses: with the direction of its
 * way and the other's where they cross, in plane coordinates. */
static int64_t CrossRoom(int64_t ax, int64_t ay, int64_t bx, int64_t by)
{
    const int64_t lengthA = (int64_t)FoxIntegerSqrt((uint64_t)(ax * ax + ay * ay));
    const int64_t lengthB = (int64_t)FoxIntegerSqrt((uint64_t)(bx * bx + by * by));
    if (lengthA == 0 || lengthB == 0)
    {
        return MIN_ROOM;
    }
    const int64_t sine = FoxAbs(ax * by - ay * bx) * FOX_UNIT / lengthA / lengthB;
    const int64_t cosine = FoxAbs(ax * bx + ay * by) * FOX_UNIT / lengthA / lengthB;
    const int64_t reach = CAR_HALF_WIDTH * cosine + (CAR_HALF_WIDTH + CROSS_MARGIN) * FOX_UNIT;
    if (sine * MAX_ROOM <= reach)
    {
        return MAX_ROOM;
    }
    return FoxMax(MIN_ROOM, reach / sine);
}

/* The direction of a sampled way at a sample, from the samples around it. */
static void SampleDirection(const int64_t *xs, const int64_t *ys, uint32_t count, int32_t sample,
                            int64_t *outX, int64_t *outY)
{
    const uint32_t at = (uint32_t)FoxMax(0, sample);
    const uint32_t from = at == 0u ? 0u : at - 1u;
    const uint32_t to = at + 1u < count ? at + 1u : at;
    *outX = xs[to] - xs[from];
    *outY = ys[to] - ys[from];
}

/* The distance along a connection to the end of one of its sampled segments;
 * samples past its end repeat its last point. */
static int64_t SegmentEnd(const FoxPath *path, int32_t segment)
{
    const uint32_t point = (uint32_t)segment + 1u;
    return point < path->count ? path->lengths[point] : path->length;
}

/* How much right of way the sign at the end of a lane gives: 2 priority, 1
 * none, 0 give way or stop. */
static uint32_t SignClass(const FoxNetwork *network, const FoxPath *lane)
{
    const FoxRoad *road = &network->roads[lane->road];
    const uint8_t sign = road->sign[lane->direction == 0u ? 1u : 0u];
    return sign == FOX_SIGN_PRIORITY ? 2u : (sign == FOX_SIGN_NONE ? 1u : 0u);
}

/* Whether connection a yields to b. Of two that cross exactly one yields,
 * but for two turning across each other's way head on: both do. */
static bool Yields(const FoxNetwork *network, const FoxNode *node, const FoxPath *a,
                   const FoxPath *b, uint32_t aIndex, uint32_t bIndex)
{
    const FoxPath *aIn = &network->paths[a->from];
    const FoxPath *bIn = &network->paths[b->from];
    const uint32_t aClass = SignClass(network, aIn);
    const uint32_t bClass = SignClass(network, bIn);
    if (aClass != bClass)
    {
        return aClass < bClass;
    }
    if ((a->movement == MOVE_U_TURN) != (b->movement == MOVE_U_TURN))
    {
        return a->movement == MOVE_U_TURN;
    }
    const FoxVec aApproach = PathEndDirection(aIn, true);
    const FoxVec bApproach = PathEndDirection(bIn, true);
    int64_t x = 0;
    int64_t y = 0;
    PlaneCoordinates(bApproach, aApproach, node->up, &x, &y);
    if (x < -(int64_t)FOX_UNIT * 7 / 10)
    {
        /* Head on: the one turning across traffic waits, then the one
         * turning towards the curb. Two turning across each other's way
         * both wait, and whoever came first goes first. */
        if ((a->movement == MOVE_FAR) != (b->movement == MOVE_FAR))
        {
            return a->movement == MOVE_FAR;
        }
        if (a->movement == MOVE_FAR)
        {
            return true;
        }
        if ((a->movement == MOVE_CURB) != (b->movement == MOVE_CURB))
        {
            return a->movement == MOVE_CURB;
        }
        return aIndex > bIndex;
    }
    if (x > (int64_t)FOX_UNIT * 7 / 10)
    {
        /* From the same direction: into one lane the lane nearer the curb
         * ends and gives way; else the lower path goes first. */
        if (a->to == b->to && aIn->road == bIn->road && aIn->lane != bIn->lane)
        {
            return aIn->lane < bIn->lane;
        }
        return aIndex > bIndex;
    }
    /* From the side: yield to whoever comes from the curb side. */
    const bool fromRight = y > 0;
    return node->driveSide == FOX_DRIVE_RIGHT ? fromRight : !fromRight;
}

/* The other ways from the lane of connection `a` of a node and where each
 * parts from it: past the stretch where cars on the two would touch, a
 * sample further on for the bodies at their bends. */
static bool BuildSiblings(FoxNetwork *network, const FoxNode *node, uint32_t a, const int64_t *xs,
                          const int64_t *ys)
{
    FoxPath *path = &network->paths[node->connections[a]];
    uint32_t siblings = 0u;
    for (uint32_t b = 0u; b < node->connectionCount; ++b)
    {
        siblings += b != a && network->paths[node->connections[b]].from == path->from ? 1u : 0u;
    }
    if (siblings == 0u)
    {
        return true;
    }
    path->siblings = malloc((size_t)siblings * sizeof(*path->siblings));
    if (path->siblings == NULL)
    {
        return false;
    }
    const size_t rowA = (size_t)a * CONNECTION_MAX_SAMPLES;
    for (uint32_t b = 0u; b < node->connectionCount; ++b)
    {
        const FoxPath *other = &network->paths[node->connections[b]];
        if (b == a || other->from != path->from)
        {
            continue;
        }
        const size_t rowB = (size_t)b * CONNECTION_MAX_SAMPLES;
        int64_t first = 0;
        int64_t last = 0;
        int64_t parting = 0;
        if (TouchingStretch(path, xs + rowA, ys + rowA, xs + rowB, ys + rowB, &first, &last))
        {
            parting = path->length;
            for (uint32_t i = 0u; i < path->count; ++i)
            {
                if (path->lengths[i] > last)
                {
                    parting = path->lengths[i];
                    break;
                }
            }
        }
        path->siblings[path->siblingCount++] =
            (FoxSibling){.path = node->connections[b], .parting = parting};
    }
    return true;
}

bool FoxLanesBuildConflicts(FoxNetwork *network, uint32_t nodeIndex)
{
    FoxNode *node = &network->nodes[nodeIndex];
    const uint32_t count = node->connectionCount;
    for (uint32_t index = 0u; index < count; ++index)
    {
        FoxPath *path = &network->paths[node->connections[index]];
        free(path->conflicts);
        path->conflicts = NULL;
        path->conflictCount = 0u;
        free(path->siblings);
        path->siblings = NULL;
        path->siblingCount = 0u;
    }
    /* At a dead end one car turns round at a time; nothing crosses. */
    if (count == 0u || FoxNodeCarRoads(network, nodeIndex) < 2u)
    {
        return true;
    }
    /* Plane coordinates of every connection, in quarter units. */
    const FoxVec axis = FoxAbs(node->up.z) < (int64_t)FOX_UNIT * 9 / 10 ? (FoxVec){0, 0, FOX_UNIT}
                                                                        : (FoxVec){FOX_UNIT, 0, 0};
    const FoxVec reference = FoxVecDirection(FoxVecCross(node->up, axis));
    const FoxVec other = FoxVecScale(FoxVecCross(node->up, reference), 1, FOX_UNIT);
    int64_t *xs = malloc((size_t)count * CONNECTION_MAX_SAMPLES * sizeof(*xs));
    int64_t *ys = malloc((size_t)count * CONNECTION_MAX_SAMPLES * sizeof(*ys));
    uint8_t *matrix = calloc((size_t)count * count, 1u);
    bool ok = xs != NULL && ys != NULL && matrix != NULL;
    for (uint32_t index = 0u; ok && index < count; ++index)
    {
        const FoxPath *path = &network->paths[node->connections[index]];
        /* Every row is filled whole; points past a path's end repeat its last. */
        for (uint32_t point = 0u; point < CONNECTION_MAX_SAMPLES; ++point)
        {
            const uint32_t source = point < path->count ? point : path->count - 1u;
            const FoxVec relative = FoxVecSub(path->points[source], node->position);
            const size_t at = (size_t)index * CONNECTION_MAX_SAMPLES + point;
            xs[at] = FoxVecDot(relative, reference) / (FOX_UNIT * 4);
            ys[at] = FoxVecDot(relative, other) / (FOX_UNIT * 4);
        }
    }
    /* matrix[a * count + b]: 0 no conflict, 1 a yields, 2 b yields; meet[a *
     * count + b] how far along b the two meet. */
    int64_t *meet = ok ? malloc((size_t)count * count * sizeof(*meet)) : NULL;
    int64_t *room = ok ? malloc((size_t)count * count * sizeof(*room)) : NULL;
    ok = ok && meet != NULL && room != NULL;
    for (uint32_t a = 0u; ok && a < count; ++a)
    {
        ok = BuildSiblings(network, node, a, xs, ys);
    }
    for (uint32_t a = 0u; ok && a < count; ++a)
    {
        const FoxPath *pa = &network->paths[node->connections[a]];
        for (uint32_t b = a + 1u; b < count; ++b)
        {
            const FoxPath *pb = &network->paths[node->connections[b]];
            if (pa->from == pb->from)
            {
                continue;
            }
            const size_t rowA = (size_t)a * CONNECTION_MAX_SAMPLES;
            const size_t rowB = (size_t)b * CONNECTION_MAX_SAMPLES;
            const int32_t onB = LastCrossing(xs + rowA, ys + rowA, CONNECTION_MAX_SAMPLES,
                                             xs + rowB, ys + rowB, CONNECTION_MAX_SAMPLES);
            const int32_t onA = LastCrossing(xs + rowB, ys + rowB, CONNECTION_MAX_SAMPLES,
                                             xs + rowA, ys + rowA, CONNECTION_MAX_SAMPLES);
            const bool merge = pa->to == pb->to;
            int64_t meetB = 0;
            int64_t meetA = 0;
            int32_t sampleA = -1;
            int32_t sampleB = -1;
            if (merge)
            {
                /* Lanes that merge share everything to the end. */
                meetB = pb->length;
                meetA = pa->length;
            }
            else if (onB >= 0)
            {
                meetB = SegmentEnd(pb, onB);
                meetA = onA < 0 ? pa->length : SegmentEnd(pa, onA);
                sampleA = onA;
                sampleB = onB;
            }
            else
            {
                /* Ways that pass too close for two cars, such as turns
                 * across from either side of a small intersection, take
                 * turns as if they crossed where they come closest. */
                int32_t nearA = 0;
                const int32_t nearB = ClosestPass(xs + rowA, ys + rowA, xs + rowB, ys + rowB,
                                                  CONNECTION_MAX_SAMPLES, CLOSE_PASS / 4, &nearA);
                if (nearB < 0)
                {
                    continue;
                }
                meetB = nearB < (int32_t)pb->count ? pb->lengths[nearB] : pb->length;
                meetA = nearA < (int32_t)pa->count ? pa->lengths[nearA] : pa->length;
                sampleA = nearA;
                sampleB = nearB;
            }
            meet[a * count + b] = meetB;
            meet[b * count + a] = meetA;
            int64_t ax = 0;
            int64_t ay = 0;
            int64_t bx = 0;
            int64_t by = 0;
            SampleDirection(xs + rowA, ys + rowA, CONNECTION_MAX_SAMPLES, sampleA, &ax, &ay);
            SampleDirection(xs + rowB, ys + rowB, CONNECTION_MAX_SAMPLES, sampleB, &bx, &by);
            int64_t keep = merge || sampleA < 0 ? MIN_ROOM : CrossRoom(ax, ay, bx, by);
            /* Ways that run close together for a while, as into a merge,
             * meet in the middle of where cars on them would touch, and the
             * room takes all of it in. */
            int64_t firstA = 0;
            int64_t lastA = 0;
            int64_t firstB = 0;
            int64_t lastB = 0;
            if (TouchingStretch(pa, xs + rowA, ys + rowA, xs + rowB, ys + rowB, &firstA, &lastA) &&
                TouchingStretch(pb, xs + rowB, ys + rowB, xs + rowA, ys + rowA, &firstB, &lastB) &&
                (meetA - keep > firstA || meetA + keep < lastA || meetB - keep > firstB ||
                 meetB + keep < lastB))
            {
                meetA = (firstA + lastA) / 2;
                meetB = (firstB + lastB) / 2;
                keep = FoxMax(keep, FoxMax(lastA - firstA, lastB - firstB) / 2 + CROSS_MARGIN);
                meet[a * count + b] = meetB;
                meet[b * count + a] = meetA;
            }
            room[a * count + b] = keep;
            room[b * count + a] = keep;
            matrix[a * count + b] =
                Yields(network, node, pa, pb, node->connections[a], node->connections[b]) ? 1u : 2u;
            matrix[b * count + a] =
                Yields(network, node, pb, pa, node->connections[b], node->connections[a]) ? 1u : 2u;
        }
    }
    for (uint32_t a = 0u; ok && a < count; ++a)
    {
        uint32_t conflicts = 0u;
        for (uint32_t b = 0u; b < count; ++b)
        {
            conflicts += matrix[a * count + b] != 0u ? 1u : 0u;
        }
        FoxPath *path = &network->paths[node->connections[a]];
        if (conflicts == 0u)
        {
            continue;
        }
        path->conflicts = malloc((size_t)conflicts * sizeof(*path->conflicts));
        if (path->conflicts == NULL)
        {
            ok = false;
            break;
        }
        for (uint32_t b = 0u; b < count; ++b)
        {
            if (matrix[a * count + b] != 0u)
            {
                path->conflicts[path->conflictCount++] = (FoxConflict){
                    .path = node->connections[b],
                    .yields = matrix[a * count + b] == 1u,
                    .otherAt = meet[a * count + b],
                    .room = room[a * count + b],
                };
            }
        }
    }
    free(xs);
    free(ys);
    free(matrix);
    free(meet);
    free(room);
    return ok;
}

bool FoxLanesBuildNode(FoxNetwork *network, uint32_t nodeIndex)
{
    FoxNode *node = &network->nodes[nodeIndex];
    bool ok = true;
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        ok = FoxNetworkRetirePath(network, node->connections[index]) && ok;
    }
    free(node->connections);
    node->connections = NULL;
    node->connectionCount = 0u;
    for (uint32_t index = 0u; index < node->cornerCount; ++index)
    {
        ok = FoxNetworkRetirePath(network, node->corners[index]) && ok;
    }
    free(node->corners);
    node->corners = NULL;
    node->cornerCount = 0u;
    if (!node->alive)
    {
        return ok;
    }
    ok = BuildCorners(network, nodeIndex) && ok;
    node = &network->nodes[nodeIndex];
    if (FoxNodeCarRoads(network, nodeIndex) < 3u)
    {
        node->signals = false;
    }
    FoxSignalsAssign(network, nodeIndex);
    FoxLanesResolveSigns(network, nodeIndex);

    NodeEnd ends[FOX_NODE_MAX_ROADS];
    uint32_t endCount = 0u;
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        const uint32_t roadIndex = node->roads[slot];
        const FoxRoad *road = &network->roads[roadIndex];
        const uint32_t end = FoxRoadEndAt(road, nodeIndex);
        /* Lanes of direction 0 end at nodes[1]; those of direction 1 at nodes[0]. */
        const uint32_t inDirection = end == 1u ? 0u : 1u;
        NodeEnd *entry = &ends[endCount++];
        *entry = (NodeEnd){
            .road = roadIndex,
            .rank = (uint8_t)FoxRoadLayoutCarLanes(&road->layout),
        };
        for (uint32_t lane = 0u; lane < road->laneCount[inDirection]; ++lane)
        {
            entry->in[entry->inCount++] = road->lanes[inDirection][lane];
            FoxPath *path = &network->paths[road->lanes[inDirection][lane]];
            free(path->outgoing);
            path->outgoing = NULL;
            path->outgoingCount = 0u;
        }
        for (uint32_t lane = 0u; lane < road->laneCount[1u - inDirection]; ++lane)
        {
            entry->out[entry->outCount++] = road->lanes[1u - inDirection][lane];
        }
        const FoxVec leaving = LeavingDirection(road, end);
        entry->leave = entry->outCount != 0u
                           ? PathEndDirection(&network->paths[entry->out[0]], false)
                           : leaving;
        entry->approach = entry->inCount != 0u
                              ? PathEndDirection(&network->paths[entry->in[0]], true)
                              : FoxVecNegate(leaving);
    }

    for (uint32_t i = 0u; i < endCount; ++i)
    {
        const NodeEnd *in = &ends[i];
        if (in->inCount == 0u)
        {
            continue;
        }
        /* The other roads one can leave by, ordered from the curb side. */
        uint32_t order[FOX_NODE_MAX_ROADS];
        int64_t xs[FOX_NODE_MAX_ROADS];
        int64_t ys[FOX_NODE_MAX_ROADS];
        uint32_t outCount = 0u;
        for (uint32_t j = 0u; j < endCount; ++j)
        {
            if (j == i || ends[j].outCount == 0u)
            {
                continue;
            }
            PlaneCoordinates(ends[j].leave, in->approach, node->up, &xs[j], &ys[j]);
            uint32_t position = outCount++;
            while (position > 0u)
            {
                const uint32_t previous = order[position - 1u];
                const bool before = node->driveSide == FOX_DRIVE_RIGHT
                                        ? AngleLess(xs[j], ys[j], xs[previous], ys[previous])
                                        : AngleLess(xs[previous], ys[previous], xs[j], ys[j]);
                if (!before)
                {
                    break;
                }
                order[position] = previous;
                --position;
            }
            order[position] = j;
        }
        if (outCount == 0u)
        {
            /* A dead end: every lane turns around, and only here; where the
             * road goes on, drivers plan their way so they never need to. */
            if (in->outCount != 0u)
            {
                ok = ConnectGroup(network, nodeIndex, in->in, 0u, in->inCount, in->out,
                                  in->outCount, MOVE_U_TURN) &&
                     ok;
            }
            continue;
        }
        /* Where a road goes straight on, every lane may: the curb lane also
         * turns to the curb side and the innermost lane across, as drivers
         * expect. Otherwise the lanes share the roads out in order. */
        uint32_t straight = outCount;
        for (uint32_t k = 0u; k < outCount; ++k)
        {
            const int64_t x = xs[order[k]];
            const int64_t y = FoxAbs(ys[order[k]]);
            const bool better =
                straight == outCount || y * xs[order[straight]] < FoxAbs(ys[order[straight]]) * x;
            if (Classify(x, ys[order[k]], node->driveSide) == MOVE_STRAIGHT && better)
            {
                straight = k;
            }
        }
        for (uint32_t k = 0u; straight != outCount && k < outCount; ++k)
        {
            const NodeEnd *out = &ends[order[k]];
            const uint32_t first = k > straight ? in->inCount - 1u : 0u;
            const uint32_t lanes = k == straight ? in->inCount : 1u;
            ok = ConnectGroup(network, nodeIndex, in->in, first, lanes, out->out, out->outCount,
                              Classify(xs[order[k]], ys[order[k]], node->driveSide)) &&
                 ok;
        }
        if (straight != outCount)
        {
            continue;
        }
        if (in->inCount >= outCount)
        {
            for (uint32_t k = 0u; k < outCount; ++k)
            {
                const uint32_t first = k * in->inCount / outCount;
                const uint32_t last = (k + 1u) * in->inCount / outCount;
                const NodeEnd *out = &ends[order[k]];
                ok = ConnectGroup(network, nodeIndex, in->in, first, last - first, out->out,
                                  out->outCount,
                                  Classify(xs[order[k]], ys[order[k]], node->driveSide)) &&
                     ok;
            }
        }
        else
        {
            for (uint32_t lane = 0u; lane < in->inCount; ++lane)
            {
                const uint32_t first = lane * outCount / in->inCount;
                const uint32_t last = (lane + 1u) * outCount / in->inCount;
                for (uint32_t k = first; k < last; ++k)
                {
                    const NodeEnd *out = &ends[order[k]];
                    ok = ConnectGroup(network, nodeIndex, in->in, lane, 1u, out->out, out->outCount,
                                      Classify(xs[order[k]], ys[order[k]], node->driveSide)) &&
                         ok;
                }
            }
        }
    }
    return FoxLanesBuildConflicts(network, nodeIndex) && ok;
}

/* ---- Signs and limits ---------------------------------------------------------- */

void FoxLanesResolveSigns(FoxNetwork *network, uint32_t nodeIndex)
{
    const FoxNode *node = &network->nodes[nodeIndex];
    /* An explicit priority sign outranks every road size. */
    uint32_t classes[FOX_NODE_MAX_ROADS];
    uint32_t highest = 0u;
    uint32_t lowest = UINT32_MAX;
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        const FoxRoad *road = &network->roads[node->roads[slot]];
        const uint8_t setting = road->signSetting[FoxRoadEndAt(road, nodeIndex)];
        classes[slot] = setting == FOX_SIGN_PRIORITY ? 256u : FoxRoadLayoutCarLanes(&road->layout);
        if (setting == FOX_SIGN_AUTO || setting == FOX_SIGN_PRIORITY)
        {
            highest = classes[slot] > highest ? classes[slot] : highest;
            lowest = classes[slot] < lowest ? classes[slot] : lowest;
        }
    }
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        FoxRoad *road = &network->roads[node->roads[slot]];
        const uint32_t end = FoxRoadEndAt(road, nodeIndex);
        uint8_t sign = road->signSetting[end];
        if (node->signals || FoxNodeCarRoads(network, nodeIndex) < 3u)
        {
            /* Lights rule; bends and dead ends need no sign. */
            sign = FOX_SIGN_NONE;
        }
        else if (sign == FOX_SIGN_AUTO)
        {
            sign = highest == lowest          ? FOX_SIGN_NONE
                   : classes[slot] == highest ? FOX_SIGN_PRIORITY
                                              : FOX_SIGN_YIELD;
        }
        road->sign[end] = sign;
    }
}

void FoxLanesApplyRoadSpeed(FoxNetwork *network, uint32_t roadIndex)
{
    const FoxRoad *road = &network->roads[roadIndex];
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
        {
            FoxPath *path = &network->paths[road->lanes[direction][lane]];
            path->speedLimit = LaneLimit(road, direction, path->curveSpeed);
        }
    }
}

void FoxLanesApplyNodeSpeeds(FoxNetwork *network, uint32_t nodeIndex)
{
    const FoxNode *node = &network->nodes[nodeIndex];
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        FoxPath *path = &network->paths[node->connections[index]];
        path->speedLimit = ConnectionSpeed(network, path);
    }
}
