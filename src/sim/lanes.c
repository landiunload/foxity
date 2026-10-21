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
#define BEND_MARGIN FOX_METRES(1)
#define MIN_BEND_TRIM (FOX_METRES(3) / 2)
/* The lanes of a dead end stop this far before its node and swing out onto
 * the turning circle. */
#define TURNAROUND_ENTRY FOX_METRES(9)
/* A turning circle is driven in this many steps of twenty degrees: half a
 * turn and twice the thirty degrees that lead in and out of it. */
#define TURNAROUND_STEPS 12u
#define TURNAROUND_LEAD_STEPS 5u
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
    if (sign == FOX_SIGN_NONE && network->nodes[connection->node].roadCount >= 3u &&
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
    const int64_t sign = lane->count > 1u && lane->centre[lane->count - 1u] < lane->centre[0] ? -1 : 1;
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

static int64_t HalfWidth(const FoxRoad *road)
{
    const FoxRoadTypeInfo *type = FoxRoadTypeGet((FoxRoadType)road->type);
    return (int64_t)(type->forwardLanes + type->backwardLanes) * FOX_LANE_WIDTH / 2;
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

static void PlaneCoordinates(FoxVec direction, FoxVec reference, FoxVec up, int64_t *outX,
                             int64_t *outY);
static bool AngleLess(int64_t ax, int64_t ay, int64_t bx, int64_t by);

void FoxLanesMeasureNode(FoxNetwork *network, uint32_t index)
{
    FoxNode *node = &network->nodes[index];
    const uint32_t count = node->roadCount;
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
        trims[slot] = count == 1u ? TURNAROUND_ENTRY : (count == 2u ? MIN_BEND_TRIM : 0);
        /* Round the node anticlockwise seen from above. */
        PlaneCoordinates(leaving[slot], leaving[0], node->up, &xs[slot], &ys[slot]);
        uint32_t position = slot;
        while (position > 0u && AngleLess(xs[slot], ys[slot], xs[order[position - 1u]],
                                          ys[order[position - 1u]]))
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
            trims[a] = b == a ? trims[a]
                              : FoxMax(trims[a], paved[b] * sine / FOX_UNIT + margin);
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
    const uint32_t generation = path->generation + 1u;
    memset(path, 0, sizeof(*path));
    path->generation = generation;
}

/* One lane along a road's centre line between its trims, `offset` units to
 * the right of the centre (seen from nodes[0] towards nodes[1]). */
static uint32_t BuildLane(FoxNetwork *network, uint32_t roadIndex, uint8_t direction, uint8_t lane,
                          int64_t offset)
{
    const uint32_t index = FoxNetworkAllocatePath(network);
    if (index == FOX_NETWORK_NONE)
    {
        return FOX_NETWORK_NONE;
    }
    const FoxRoad *road = &network->roads[roadIndex];
    const int64_t first = road->trim[0];
    const int64_t last = FoxMax(first + FOX_METRES(1), road->length - road->trim[1]);
    uint32_t count = 2u;
    for (uint32_t point = 0u; point < road->centreCount; ++point)
    {
        count += road->centreLengths[point] > first && road->centreLengths[point] < last ? 1u : 0u;
    }
    FoxPath *path = &network->paths[index];
    ResetPath(path, PATH_LANE);
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
    path->count = count;
    path->length = FoxPolylineMeasure(path->points, path->lengths, count);
    path->curveSpeed = CurveSpeed(path->points, count);
    path->speedLimit = (int32_t)FoxMin(road->speedLimit, path->curveSpeed);
    path->road = roadIndex;
    path->roadGeneration = road->generation;
    path->direction = direction;
    path->lane = lane;
    return index;
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
    const FoxRoadTypeInfo *type = FoxRoadTypeGet((FoxRoadType)road->type);
    const uint8_t counts[2] = {type->forwardLanes, type->backwardLanes};
    const int64_t side = road->driveSide == FOX_DRIVE_RIGHT ? 1 : -1;
    const bool twoWay = counts[1] != 0u;
    for (uint8_t direction = 0u; direction < 2u; ++direction)
    {
        for (uint8_t lane = 0u; lane < counts[direction]; ++lane)
        {
            /* Twice the offset in lane widths: the curb lane is outermost. A
             * one-way road is centred on its centre line. */
            const int64_t doubled = twoWay ? 2 * (int64_t)counts[direction] - (int64_t)lane * 2 - 1
                                           : (int64_t)counts[direction] - (int64_t)lane * 2 - 1;
            const int64_t offset =
                (direction == 0u ? side : -side) * doubled * FOX_LANE_WIDTH / 2;
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
    return ok;
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

/* A dead end's turn: from the end of the lane in it swings out onto a circle
 * about the node, drives two thirds of the way round and comes back into the
 * lane out, as a driver uses a turning circle. Fills TURNAROUND_COUNT points. */
#define TURNAROUND_COUNT (2u * TURNAROUND_LEAD_STEPS + TURNAROUND_STEPS + 1u)
static void TurnaroundPoints(const FoxNetwork *network, uint32_t nodeIndex, const FoxPath *in,
                             const FoxPath *out, FoxVec *points)
{
    /* Twenty degrees in FOX_UNIT, and where the circle is joined: 120 degrees
     * either side of straight on. */
    const int64_t cosStep = 61584;
    const int64_t sinStep = 22415;
    const FoxNode *node = &network->nodes[nodeIndex];
    const int64_t radius =
        FoxRoadTurnaroundRadius((FoxRoadType)network->roads[in->road].type);
    const FoxVec forward = PathEndDirection(in, true);
    /* Round towards the middle of the road: left where traffic keeps right. */
    FoxVec side = FoxVecScale(FoxVecCross(node->up, forward), 1, FOX_UNIT);
    if (node->driveSide == FOX_DRIVE_LEFT)
    {
        side = FoxVecNegate(side);
    }
    FoxVec *arc = points + TURNAROUND_LEAD_STEPS;
    int64_t cosine = -(int64_t)FOX_UNIT / 2;
    int64_t sine = -56756;
    for (uint32_t step = 0u; step <= TURNAROUND_STEPS; ++step)
    {
        arc[step] = FoxVecAdd(node->position,
                              FoxVecAdd(FoxVecScale(forward, radius * cosine / FOX_UNIT, FOX_UNIT),
                                        FoxVecScale(side, radius * sine / FOX_UNIT, FOX_UNIT)));
        const int64_t turned = (cosine * cosStep - sine * sinStep) / FOX_UNIT;
        sine = (cosine * sinStep + sine * cosStep) / FOX_UNIT;
        cosine = turned;
    }
    /* Onto the circle heading thirty degrees outwards, off it thirty degrees
     * inwards. */
    const int64_t cos30 = 56756;
    const int64_t sin30 = (int64_t)FOX_UNIT / 2;
    const FoxVec onto = FoxVecSub(FoxVecScale(forward, cos30, FOX_UNIT),
                                  FoxVecScale(side, sin30, FOX_UNIT));
    const FoxVec off = FoxVecNegate(FoxVecAdd(FoxVecScale(forward, cos30, FOX_UNIT),
                                              FoxVecScale(side, sin30, FOX_UNIT)));
    const FoxVec start = in->points[in->count - 1u];
    const FoxVec end = out->points[0];
    FoxVec p1;
    FoxVec p2;
    Handles(start, forward, arc[0], onto, &p1, &p2);
    for (uint32_t step = 0u; step < TURNAROUND_LEAD_STEPS; ++step)
    {
        points[step] = Cubic(start, p1, p2, arc[0],
                             (int64_t)step * FOX_UNIT / (int64_t)TURNAROUND_LEAD_STEPS);
    }
    Handles(arc[TURNAROUND_STEPS], off, end, PathEndDirection(out, false), &p1, &p2);
    for (uint32_t step = 1u; step <= TURNAROUND_LEAD_STEPS; ++step)
    {
        arc[TURNAROUND_STEPS + step] =
            Cubic(arc[TURNAROUND_STEPS], p1, p2, end,
                  (int64_t)step * FOX_UNIT / (int64_t)TURNAROUND_LEAD_STEPS);
    }
}

static uint32_t BuildConnection(FoxNetwork *network, uint32_t nodeIndex, uint32_t from, uint32_t to,
                                uint8_t movement)
{
    _Static_assert(TURNAROUND_COUNT <= CONNECTION_MAX_SAMPLES,
                   "conflict tests compare at most CONNECTION_MAX_SAMPLES points");
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
    const uint32_t count =
        movement == MOVE_U_TURN
            ? TURNAROUND_COUNT
            : (uint32_t)FoxClamp(estimate / FOX_METRES(1) + 1, CONNECTION_MIN_SAMPLES,
                                 CONNECTION_MAX_SAMPLES);

    FoxPath *path = &network->paths[index];
    ResetPath(path, PATH_CONNECTION);
    if (!AllocatePathArrays(path, count, false))
    {
        FreeNewPath(network, index);
        return FOX_NETWORK_NONE;
    }
    if (movement == MOVE_U_TURN)
    {
        TurnaroundPoints(network, nodeIndex, &network->paths[from], &network->paths[to],
                         path->points);
    }
    else
    {
        for (uint32_t sample = 0u; sample < count; ++sample)
        {
            path->points[sample] =
                Cubic(p0, p1, p2, p3, (int64_t)sample * FOX_UNIT / (int64_t)(count - 1u));
        }
    }
    path->count = count;
    path->length = FoxMax(1, FoxPolylineMeasure(path->points, path->lengths, count));
    path->curveSpeed = CurveSpeed(path->points, count);
    path->node = nodeIndex;
    path->from = from;
    path->to = to;
    path->movement = movement;
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
 * share covers. */
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

/* Whether connection a yields to b; the caller gives b the opposite. */
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
         * turning towards the curb. */
        if ((a->movement == MOVE_FAR) != (b->movement == MOVE_FAR))
        {
            return a->movement == MOVE_FAR;
        }
        if ((a->movement == MOVE_CURB) != (b->movement == MOVE_CURB))
        {
            return a->movement == MOVE_CURB;
        }
        return aIndex > bIndex;
    }
    if (x > (int64_t)FOX_UNIT * 7 / 10)
    {
        /* From the same direction: the lower path goes first. */
        return aIndex > bIndex;
    }
    /* From the side: yield to whoever comes from the curb side. */
    const bool fromRight = y > 0;
    return node->driveSide == FOX_DRIVE_RIGHT ? fromRight : !fromRight;
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
    }
    if (count == 0u)
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
    ok = ok && meet != NULL;
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
            if (!merge && onB < 0)
            {
                continue;
            }
            /* Lanes that merge share everything to the end. */
            meet[a * count + b] = merge ? pb->length : SegmentEnd(pb, onB);
            meet[b * count + a] = merge || onA < 0 ? pa->length : SegmentEnd(pa, onA);
            const bool aYields = Yields(network, node, pa, pb, node->connections[a],
                                        node->connections[b]);
            matrix[a * count + b] = aYields ? 1u : 2u;
            matrix[b * count + a] = aYields ? 2u : 1u;
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
                };
            }
        }
    }
    free(xs);
    free(ys);
    free(matrix);
    free(meet);
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
    if (!node->alive)
    {
        return ok;
    }
    if (node->roadCount < 3u)
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
            .rank = FoxRoadTypeGet((FoxRoadType)road->type)->rank,
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
                    ok = ConnectGroup(network, nodeIndex, in->in, lane, 1u, out->out,
                                      out->outCount,
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
        classes[slot] = setting == FOX_SIGN_PRIORITY
                            ? 256u
                            : FoxRoadTypeGet((FoxRoadType)road->type)->rank;
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
        if (node->signals || node->roadCount < 3u)
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
            path->speedLimit = (int32_t)FoxMin(road->speedLimit, path->curveSpeed);
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
