#include "sim/traffic.h"

#include "sim/network_internal.h"
#include "sim/random.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define NONE FOX_NETWORK_NONE
#define TICKS FOX_TRAFFIC_TICKS_PER_SECOND
#define ROUTE_CAPACITY 128u
/* The driver model: emergency braking and the gap kept when standing; the
 * rest depends on the vehicle. */
#define MAX_BRAKE FOX_METRES(9)
#define MIN_GAP FOX_METRES(2)
/* How far ahead a car looks for leaders and slower paths. */
#define LOOKAHEAD FOX_METRES(150)
#define STANDING_SPEED (FOX_METRES(1) / 2)
#define MOVING_SPEED FOX_METRES(2)
/* A lane whose last car moves this fast is not filling up. */
#define FREE_FLOW_SPEED FOX_METRES(5)
/* Cars this close to their line, beyond their braking distance and two
 * seconds of driving, take part in their intersection's rounds. */
#define DECISION_MARGIN FOX_METRES(15)
/* A car crosses the way of one it gives way to only when it is through a
 * gap before the other one could get there: this long at first, down to the
 * shortest after waiting the patience. One passing first must be through
 * the safety margin before it comes itself. Out of patience a driver also
 * goes ahead of those that can still stop for it comfortably with room to
 * spare, at first leaving them time to react, none after twice the
 * patience. */
#define REACTION_TICKS (INT64_C(3) * TICKS / 2)
/* How many lanes back drivers look for those they give way to. */
#define UPSTREAM_LANES 3u
#define UPSTREAM_PENDING 64u
#define SPARE_ROOM FOX_METRES(5)
#define GAP_TICKS (INT64_C(2) * TICKS)
#define MIN_GAP_TICKS (INT64_C(1) * TICKS / 2)
#define PATIENCE_TICKS (UINT64_C(20) * TICKS)
#define SAFETY_TICKS (INT64_C(1) * TICKS)
/* A car's rear this far past where two ways meet leaves the other one free,
 * and its front stays this far short of it while it waits. */
#define PASSED_MARGIN FOX_METRES(2)
#define CLEARANCE (FOX_LANE_WIDTH / 2 + FOX_METRES(1))
/* A car turning across the oncoming lanes drives in to wait when it gets at
 * least this far past its line; cars from one lane wait for each other until
 * the rear of the one ahead is this far along, where their ways part. */
#define MIN_HOLD FOX_METRES(4)
/* A car this close behind another one in an intersection goes no faster. */
#define LEADER_REACH FOX_METRES(20)
#define PARTING FOX_METRES(8)
#define HOLD_NONE INT64_MAX
/* At a stop sign a car stands this close to the line before it may go. */
#define STOP_LINE_REACH FOX_METRES(5)
/* Out of a hundred new vehicles, this many are trucks. */
#define TRUCK_PERCENT 15u
/* How many paths behind a vehicle are remembered for drawing its rear. */
#define TRAIL 3u
/* A car standing this long is taken off the road, as city builders do: an
 * overloaded network can always tie itself into a knot, and this undoes it
 * while the spawner brings the car back elsewhere. Waiting through a few
 * red lights takes less. */
#define STUCK_TICKS (120u * TICKS)
#define STATS_TICKS (UINT64_C(2) * TICKS)
#define REROUTE_TICKS (UINT64_C(30) * TICKS)
#define SPAWN_PER_TICK 2u
#define SPAWN_CLEARANCE FOX_METRES(14)
#define COLORS 8u

/* How each kind of vehicle drives: its length, acceleration, comfortable
 * braking, time gap and top speed. */
typedef struct Driving
{
    int64_t length;
    int64_t acceleration;
    int64_t brake;
    int64_t headwayMs;
    int64_t topSpeed;
} Driving;

static const Driving drivings[FOX_VEHICLE_KIND_COUNT] = {
    [FOX_VEHICLE_CAR] = {FOX_CAR_LENGTH, FOX_METRES(2), FOX_METRES(5) / 2, 1200, FOX_METRES(60)},
    [FOX_VEHICLE_TRUCK] = {FOX_TRUCK_LENGTH, FOX_METRES(6) / 5, FOX_METRES(8) / 5, 1800,
                           FOX_KMH(80)},
};

typedef struct Vehicle
{
    bool alive;
    bool permitted;
    bool clear;
    bool parking;
    /* Stood at the line of a stop sign. */
    bool stopped;
    uint8_t kind;
    uint8_t color;
    uint64_t serial;
    uint32_t path;
    uint32_t pathGeneration;
    int64_t s;
    uint32_t prevPath;
    uint32_t prevGeneration;
    int64_t prevS;
    int32_t speed;
    int32_t acceleration;
    /* Neighbours on the same path towards the front and the back. */
    uint32_t ahead;
    uint32_t behind;
    /* The connection chosen at the end of the current lane, the one this car
     * has claimed ahead, and the one behind it that it has not yet cleared. */
    uint32_t connection;
    uint32_t claim;
    uint32_t clearing;
    /* Beyond a lane too short to stop on, the connection claimed together
     * with the one before it. */
    uint32_t through;
    /* How far along its connection a car turning across the oncoming lanes
     * may drive to wait there for a gap; HOLD_NONE to drive through. */
    int64_t holdAt;
    uint32_t waitTicks;
    /* The tick the car joined its intersection's rounds; zero outside them. */
    uint64_t arrival;
    /* Q16 share of the speed limit this driver aims for. */
    int32_t desire;
    uint32_t destination;
    uint32_t route[ROUTE_CAPACITY];
    uint32_t routeLength;
    uint32_t routeIndex;
    uint64_t routeRevision;
    uint64_t routeTick;
    /* The paths driven before the current one, newest first, for the view. */
    uint32_t trail[TRAIL];
    uint32_t trailGeneration[TRAIL];
    int64_t trailLength[TRAIL];
    uint32_t trailCount;
    FoxRandom random;
} Vehicle;

static const Driving *DrivingOf(const Vehicle *vehicle)
{
    return &drivings[vehicle->kind];
}

typedef struct Candidate Candidate;

typedef struct TakenWay TakenWay;

struct FoxTraffic
{
    uint64_t seed;
    uint32_t target;
    Vehicle *vehicles;
    uint32_t capacity;
    uint32_t count;
    uint64_t tick;
    uint64_t nextSerial;
    uint64_t spawned;
    uint64_t arrived;
    uint64_t removed;
    uint64_t stuck;
    FoxRandom random;
    Candidate *candidates;
    TakenWay *taken;
    uint32_t candidateCapacity;
};

FoxTraffic *FoxTrafficCreate(const FoxTrafficDesc *desc)
{
    if (desc == NULL || desc->targetVehicles > FOX_TRAFFIC_MAX_VEHICLES)
    {
        return NULL;
    }
    FoxTraffic *traffic = calloc(1u, sizeof(*traffic));
    if (traffic == NULL)
    {
        return NULL;
    }
    traffic->seed = desc->seed;
    traffic->target = desc->targetVehicles;
    traffic->nextSerial = 1u;
    traffic->random = FoxRandomStream(desc->seed, UINT64_C(0x7AFF1C));
    return traffic;
}

void FoxTrafficDestroy(FoxTraffic *traffic)
{
    if (traffic != NULL)
    {
        free(traffic->vehicles);
        free(traffic->candidates);
        free(traffic->taken);
        free(traffic);
    }
}

void FoxTrafficSetTarget(FoxTraffic *traffic, uint32_t vehicles)
{
    if (traffic != NULL)
    {
        traffic->target = vehicles > FOX_TRAFFIC_MAX_VEHICLES ? FOX_TRAFFIC_MAX_VEHICLES : vehicles;
    }
}

uint32_t FoxTrafficTarget(const FoxTraffic *traffic)
{
    return traffic == NULL ? 0u : traffic->target;
}

uint32_t FoxTrafficCapacity(const FoxTraffic *traffic)
{
    return traffic == NULL ? 0u : traffic->capacity;
}

/* ---- Paths and lists -------------------------------------------------------- */

static FoxPath *PathOf(FoxNetwork *network, uint32_t path)
{
    return FoxNetworkPathRecordMutable(network, path);
}

static uint32_t EdgeOf(const FoxPath *lane)
{
    return lane->road * 2u + lane->direction;
}

static uint32_t EndNode(const FoxNetwork *network, const FoxPath *lane)
{
    return network->roads[lane->road].nodes[lane->direction == 0u ? 1u : 0u];
}

static void Unlink(FoxTraffic *traffic, FoxNetwork *network, uint32_t slot)
{
    Vehicle *vehicle = &traffic->vehicles[slot];
    FoxPath *path = PathOf(network, vehicle->path);
    if (path == NULL)
    {
        return;
    }
    if (vehicle->ahead != NONE)
    {
        traffic->vehicles[vehicle->ahead].behind = vehicle->behind;
    }
    else
    {
        path->front = vehicle->behind;
    }
    if (vehicle->behind != NONE)
    {
        traffic->vehicles[vehicle->behind].ahead = vehicle->ahead;
    }
    else
    {
        path->back = vehicle->ahead;
    }
    --path->vehicleCount;
    vehicle->ahead = NONE;
    vehicle->behind = NONE;
}

/* The neighbours a car at `s` would have on a path. */
static void FindPlace(const FoxTraffic *traffic, const FoxPath *path, int64_t s, uint32_t *outAhead,
                      uint32_t *outBehind)
{
    uint32_t ahead = path->back;
    uint32_t behind = NONE;
    while (ahead != NONE && traffic->vehicles[ahead].s <= s)
    {
        behind = ahead;
        ahead = traffic->vehicles[ahead].ahead;
    }
    *outAhead = ahead;
    *outBehind = behind;
}

static void Link(FoxTraffic *traffic, FoxNetwork *network, uint32_t slot, uint32_t pathIndex,
                 int64_t s)
{
    Vehicle *vehicle = &traffic->vehicles[slot];
    FoxPath *path = PathOf(network, pathIndex);
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, path, s, &ahead, &behind);
    vehicle->path = pathIndex;
    vehicle->pathGeneration = path->generation;
    vehicle->s = s;
    vehicle->ahead = ahead;
    vehicle->behind = behind;
    if (ahead != NONE)
    {
        traffic->vehicles[ahead].behind = slot;
    }
    else
    {
        path->front = slot;
    }
    if (behind != NONE)
    {
        traffic->vehicles[behind].ahead = slot;
    }
    else
    {
        path->back = slot;
    }
    ++path->vehicleCount;
}

static void Release(FoxNetwork *network, uint32_t *claim)
{
    if (*claim != NONE)
    {
        FoxPath *claimed = PathOf(network, *claim);
        if (claimed != NULL && claimed->claims > 0u)
        {
            --claimed->claims;
        }
        *claim = NONE;
    }
}

/* The lane a claimed connection leads into stops counting the vehicle. */
static void Arrive(FoxNetwork *network, uint32_t claim, const Vehicle *vehicle)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, claim);
    FoxPath *lane = connection == NULL ? NULL : PathOf(network, connection->to);
    const uint32_t length = (uint32_t)DrivingOf(vehicle)->length;
    if (lane != NULL)
    {
        lane->incoming = lane->incoming > length ? lane->incoming - length : 0u;
    }
}

static void ReleaseClaims(FoxNetwork *network, Vehicle *vehicle)
{
    if (vehicle->claim != NONE)
    {
        Arrive(network, vehicle->claim, vehicle);
    }
    if (vehicle->through != NONE)
    {
        Arrive(network, vehicle->through, vehicle);
    }
    Release(network, &vehicle->claim);
    Release(network, &vehicle->through);
    Release(network, &vehicle->clearing);
    vehicle->permitted = false;
}

static void Kill(FoxTraffic *traffic, FoxNetwork *network, uint32_t slot, bool linked)
{
    Vehicle *vehicle = &traffic->vehicles[slot];
    if (linked)
    {
        Unlink(traffic, network, slot);
    }
    ReleaseClaims(network, vehicle);
    vehicle->alive = false;
    --traffic->count;
    ++traffic->removed;
}

/* ---- Routes ------------------------------------------------------------------ */

static uint32_t RandomNode(FoxTraffic *traffic, const FoxNetwork *network, uint32_t avoid)
{
    if (network->nodeCapacity == 0u)
    {
        return NONE;
    }
    for (uint32_t attempt = 0u; attempt < 16u; ++attempt)
    {
        const uint32_t node = FoxRandomBelow(&traffic->random, network->nodeCapacity);
        if (network->nodes[node].alive && node != avoid)
        {
            return node;
        }
    }
    return NONE;
}

static bool LaneLeadsTo(const FoxNetwork *network, const FoxPath *lane, uint32_t edge)
{
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        const FoxPath *to =
            connection == NULL ? NULL : FoxNetworkPathRecord(network, connection->to);
        if (to != NULL && EdgeOf(to) == edge)
        {
            return true;
        }
    }
    return false;
}

/* What the router reckons an edge costs. */
static uint64_t EdgeCost(const FoxNetwork *network, uint32_t edge)
{
    return network->roads[edge / 2u].routedCost[edge % 2u];
}

/* A route from the end of a lane. The router knows roads, not lanes, and
 * would as soon begin by turning back down the road the car is on; a route
 * must begin with a turn the lane really has. When the cheapest one does
 * not, each way the lane leads is tried, with the cheapest route on from
 * there. */
static void PlanRoute(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle,
                      uint32_t laneIndex, bool keepDestination)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const uint32_t from = EndNode(network, lane);
    vehicle->routeLength = 0u;
    vehicle->routeIndex = 0u;
    vehicle->routeRevision = network->revision;
    vehicle->routeTick = traffic->tick;
    if (network->router == NULL)
    {
        return;
    }
    if (!keepDestination || vehicle->destination >= network->nodeCapacity ||
        !network->nodes[vehicle->destination].alive || vehicle->destination == from)
    {
        vehicle->destination = RandomNode(traffic, network, from);
    }
    const uint32_t destination = vehicle->destination;
    if (destination == NONE)
    {
        return;
    }
    const FoxRouter *router = network->router;
    vehicle->routeLength =
        router->findRoute(router->context, from, destination, vehicle->route, ROUTE_CAPACITY);
    if (vehicle->routeLength != 0u && LaneLeadsTo(network, lane, vehicle->route[0]))
    {
        return;
    }
    vehicle->routeLength = 0u;
    uint32_t onward[ROUTE_CAPACITY];
    uint64_t best = UINT64_MAX;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        const FoxPath *next = FoxNetworkPathRecord(network, connection->to);
        const uint32_t edge = EdgeOf(next);
        bool tried = false;
        for (uint32_t earlier = 0u; earlier < index && !tried; ++earlier)
        {
            const FoxPath *other = FoxNetworkPathRecord(network, lane->outgoing[earlier]);
            tried = EdgeOf(FoxNetworkPathRecord(network, other->to)) == edge;
        }
        if (tried)
        {
            continue;
        }
        const uint32_t beyond = EndNode(network, next);
        uint32_t length = 0u;
        uint64_t cost = EdgeCost(network, edge);
        if (beyond != destination)
        {
            length = router->findRoute(router->context, beyond, destination, onward,
                                       ROUTE_CAPACITY - 1u);
            if (length == 0u)
            {
                continue;
            }
            for (uint32_t step = 0u; step < length; ++step)
            {
                cost += EdgeCost(network, onward[step]);
            }
        }
        if (cost < best)
        {
            best = cost;
            vehicle->route[0] = edge;
            memcpy(vehicle->route + 1, onward, (size_t)length * sizeof(*onward));
            vehicle->routeLength = length + 1u;
        }
    }
}

/* The connection from a lane into the road want, into a lane leading on
 * to fter where possible, the emptiest such lane first. */
static uint32_t RouteConnection(const FoxNetwork *network, const FoxPath *lane, uint32_t want,
                                uint32_t after)
{
    uint32_t best = NONE;
    bool bestGood = false;
    uint32_t bestLoad = UINT32_MAX;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const uint32_t candidate = lane->outgoing[index];
        const FoxPath *connection = FoxNetworkPathRecord(network, candidate);
        const FoxPath *to = FoxNetworkPathRecord(network, connection->to);
        if (EdgeOf(to) != want)
        {
            continue;
        }
        const bool good = after == NONE || LaneLeadsTo(network, to, after);
        const uint32_t load = to->vehicleCount + to->incoming / (uint32_t)FOX_CAR_LENGTH;
        if (best == NONE || (good && !bestGood) || (good == bestGood && load < bestLoad))
        {
            best = candidate;
            bestGood = good;
            bestLoad = load;
        }
    }
    return best;
}

/* A random turn, turning around only where nothing else is possible. */
static uint32_t RandomConnection(const FoxNetwork *network, const FoxPath *lane, uint64_t seed)
{
    uint32_t turns = 0u;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        turns += connection->movement != MOVE_U_TURN ? 1u : 0u;
    }
    const uint32_t choices = turns != 0u ? turns : lane->outgoingCount;
    const uint32_t pick = (uint32_t)(FoxHash64(seed) % choices);
    uint32_t seen = 0u;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        if (turns != 0u && connection->movement == MOVE_U_TURN)
        {
            continue;
        }
        if (seen++ == pick)
        {
            return lane->outgoing[index];
        }
    }
    return NONE;
}

/* The connection a car will take at the end of a lane it has not reached
 * yet: the one after its next road. */
static uint32_t PlanThrough(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    if (lane->outgoingCount == 0u || vehicle->parking)
    {
        return NONE;
    }
    if (network->router == NULL || vehicle->routeLength == 0u)
    {
        /* The same pick every tick, so the claim matches the turn taken. */
        return RandomConnection(network, lane, FoxHashCombine(vehicle->serial, laneIndex));
    }
    const uint32_t index = vehicle->routeIndex + 1u;
    if (index >= vehicle->routeLength)
    {
        return NONE;
    }
    return RouteConnection(network, lane, vehicle->route[index],
                           index + 1u < vehicle->routeLength ? vehicle->route[index + 1u] : NONE);
}

/* The connection a car on a lane takes at its end: towards the next road of
 * its route, into a lane that leads on to the road after it, the emptiest
 * such lane first. Without a usable route, a random turn. */
static void ChooseConnection(FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle)
{
    vehicle->connection = NONE;
    vehicle->permitted = false;
    vehicle->clear = false;
    vehicle->stopped = false;
    vehicle->holdAt = HOLD_NONE;
    vehicle->arrival = 0u;
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    if (lane == NULL || lane->kind != PATH_LANE || lane->outgoingCount == 0u)
    {
        return;
    }
    if (network->router != NULL && !vehicle->parking && vehicle->routeLength != 0u &&
        (vehicle->routeRevision != network->revision ||
         traffic->tick - vehicle->routeTick > REROUTE_TICKS))
    {
        PlanRoute(traffic, network, vehicle, vehicle->path, true);
    }
    for (uint32_t attempt = 0u; attempt < 2u; ++attempt)
    {
        const uint32_t want =
            vehicle->routeIndex < vehicle->routeLength ? vehicle->route[vehicle->routeIndex] : NONE;
        const uint32_t after = vehicle->routeIndex + 1u < vehicle->routeLength
                                   ? vehicle->route[vehicle->routeIndex + 1u]
                                   : NONE;
        if (want == NONE)
        {
            break;
        }
        const uint32_t best = RouteConnection(network, lane, want, after);
        if (best != NONE)
        {
            vehicle->connection = best;
            return;
        }
        /* Off the route: plan again from here once. */
        if (attempt == 0u)
        {
            PlanRoute(traffic, network, vehicle, vehicle->path, true);
        }
    }
    vehicle->routeLength = 0u;
    vehicle->routeIndex = 0u;
    vehicle->connection = RandomConnection(network, lane, FoxRandomNext(&vehicle->random));
}

static void EnterLane(FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    const uint32_t edge = EdgeOf(lane);
    const bool routed = vehicle->routeLength != 0u;
    if (vehicle->routeIndex < vehicle->routeLength && vehicle->route[vehicle->routeIndex] == edge)
    {
        ++vehicle->routeIndex;
    }
    else if (routed)
    {
        vehicle->routeLength = 0u;
        vehicle->routeIndex = 0u;
    }
    if (vehicle->through != NONE)
    {
        /* Already claimed together with the connection just left. */
        vehicle->connection = vehicle->through;
        vehicle->claim = vehicle->through;
        vehicle->through = NONE;
        vehicle->permitted = true;
        vehicle->clear = true;
        return;
    }
    if (vehicle->routeIndex >= vehicle->routeLength)
    {
        if (routed && vehicle->routeLength != 0u)
        {
            ++traffic->arrived;
        }
        if (traffic->count > traffic->target)
        {
            vehicle->parking = true;
        }
        else
        {
            PlanRoute(traffic, network, vehicle, vehicle->path, false);
        }
    }
    ChooseConnection(traffic, network, vehicle);
}

/* ---- Spawning ------------------------------------------------------------------ */

static uint32_t AcquireSlot(FoxTraffic *traffic)
{
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        if (!traffic->vehicles[slot].alive)
        {
            return slot;
        }
    }
    if (traffic->capacity >= FOX_TRAFFIC_MAX_VEHICLES)
    {
        return NONE;
    }
    const uint32_t capacity = traffic->capacity < 64u ? 64u : traffic->capacity * 2u;
    Vehicle *grown = realloc(traffic->vehicles, (size_t)capacity * sizeof(*grown));
    if (grown == NULL)
    {
        return NONE;
    }
    memset(grown + traffic->capacity, 0, (size_t)(capacity - traffic->capacity) * sizeof(*grown));
    traffic->vehicles = grown;
    const uint32_t slot = traffic->capacity;
    traffic->capacity = capacity;
    return slot;
}

static void Spawn(FoxTraffic *traffic, FoxNetwork *network)
{
    for (uint32_t spawn = 0u;
         spawn < SPAWN_PER_TICK && traffic->count < traffic->target && network->roadCount != 0u;
         ++spawn)
    {
        const uint32_t roadIndex = FoxRandomBelow(&traffic->random, network->roadCapacity);
        const FoxRoad *road = &network->roads[roadIndex];
        if (!road->alive)
        {
            continue;
        }
        const uint32_t direction = road->laneCount[0] == 0u ? 1u
                                   : road->laneCount[1] == 0u
                                       ? 0u
                                       : FoxRandomBelow(&traffic->random, 2u);
        if (road->laneCount[direction] == 0u)
        {
            continue;
        }
        const uint32_t laneIndex =
            road->lanes[direction][FoxRandomBelow(&traffic->random, road->laneCount[direction])];
        const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
        if (lane == NULL || lane->retired || lane->length < 2 * SPAWN_CLEARANCE)
        {
            continue;
        }
        const int64_t s =
            FoxRandomRange(&traffic->random, SPAWN_CLEARANCE, lane->length - SPAWN_CLEARANCE);
        uint32_t ahead = NONE;
        uint32_t behind = NONE;
        FindPlace(traffic, lane, s, &ahead, &behind);
        if ((ahead != NONE && traffic->vehicles[ahead].s - s < SPAWN_CLEARANCE) ||
            (behind != NONE && s - traffic->vehicles[behind].s < SPAWN_CLEARANCE))
        {
            continue;
        }
        const uint32_t slot = AcquireSlot(traffic);
        if (slot == NONE)
        {
            return;
        }
        const uint64_t serial = traffic->nextSerial++;
        Vehicle *vehicle = &traffic->vehicles[slot];
        *vehicle = (Vehicle){
            .alive = true,
            .serial = serial,
            .prevPath = NONE,
            .ahead = NONE,
            .behind = NONE,
            .connection = NONE,
            .claim = NONE,
            .clearing = NONE,
            .through = NONE,
            .holdAt = HOLD_NONE,
            .destination = NONE,
            .random = FoxRandomStream(traffic->seed, serial),
        };
        vehicle->kind = FoxRandomBelow(&vehicle->random, 100u) < TRUCK_PERCENT
                            ? (uint8_t)FOX_VEHICLE_TRUCK
                            : (uint8_t)FOX_VEHICLE_CAR;
        vehicle->color = (uint8_t)FoxRandomBelow(&vehicle->random, COLORS);
        vehicle->desire =
            (int32_t)FoxRandomRange(&vehicle->random, FOX_UNIT * 95 / 100, FOX_UNIT * 108 / 100);
        Link(traffic, network, slot, laneIndex, s);
        vehicle->prevPath = laneIndex;
        vehicle->prevGeneration = vehicle->pathGeneration;
        vehicle->prevS = s;
        ++traffic->count;
        ++traffic->spawned;
        PlanRoute(traffic, network, vehicle, laneIndex, false);
        ChooseConnection(traffic, network, vehicle);
    }
}

/* ---- Intersections --------------------------------------------------------------- */

/* Whether a lane has room for one more vehicle of `length` beyond every one
 * already on its way there, or moves freely enough not to need it. */
static bool RoomOn(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t laneIndex,
                   int64_t length)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    if (lane == NULL)
    {
        return false;
    }
    const Vehicle *tail = lane->back != NONE ? &traffic->vehicles[lane->back] : NULL;
    if (tail != NULL && tail->speed > FREE_FLOW_SPEED)
    {
        return true;
    }
    if (tail == NULL && lane->incoming == 0u)
    {
        return true;
    }
    /* Each vehicle on its way keeps a gap besides its length. */
    const int64_t coming = (int64_t)lane->incoming * (FOX_CAR_LENGTH + MIN_GAP) / FOX_CAR_LENGTH;
    const int64_t room = (tail != NULL ? tail->s - DrivingOf(tail)->length : lane->length) - coming;
    return room >= length + MIN_GAP;
}

static void Claim(FoxNetwork *network, uint32_t connectionIndex, const Vehicle *vehicle)
{
    FoxPath *connection = PathOf(network, connectionIndex);
    ++connection->claims;
    PathOf(network, connection->to)->incoming += (uint32_t)DrivingOf(vehicle)->length;
}

static int64_t BrakingDistance(const Vehicle *vehicle)
{
    const int64_t speed = vehicle->speed;
    return speed * speed / (2 * DrivingOf(vehicle)->brake);
}

/* The light at the end of a lane. */
static FoxLight LightAt(const FoxNetwork *network, const FoxPath *lane)
{
    return FoxSignalsLight(network, lane->road, lane->direction == 0u ? 1u : 0u);
}

/* The light for a way through an intersection: the arrow while it shows
 * for a turn across the oncoming lanes, else the one at the end of its
 * lane. */
static FoxLight LightFor(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    const FoxPath *lane = FoxNetworkPathRecord(network, connection->from);
    const FoxLight arrow =
        connection->movement == MOVE_FAR
            ? FoxSignalsArrow(network, lane->road, lane->direction == 0u ? 1u : 0u)
            : FOX_LIGHT_NONE;
    return arrow != FOX_LIGHT_NONE ? arrow : LightAt(network, lane);
}

/* Right of way from the sign at the end of a lane: 2 priority, 1 none, 0
 * give way or stop. Under lights every approach is equal. */
static int64_t SignClass(const FoxNetwork *network, const FoxPath *lane)
{
    const uint8_t sign = network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u];
    return sign == FOX_SIGN_PRIORITY ? 2 : (sign == FOX_SIGN_NONE ? 1 : 0);
}

/* Ticks to cover a distance from a speed, accelerating up to a cap and
 * keeping it. */
static int64_t CoverTicks(int64_t speed, int64_t acceleration, int64_t cap, int64_t distance)
{
    if (distance <= 0)
    {
        return 0;
    }
    cap = FoxMax(cap, FOX_METRES(1));
    if (speed >= cap)
    {
        return distance * (int64_t)TICKS / cap;
    }
    const int64_t rising = (cap * cap - speed * speed) / (2 * acceleration);
    if (distance <= rising)
    {
        const int64_t root =
            (int64_t)FoxIntegerSqrt((uint64_t)(speed * speed + 2 * acceleration * distance));
        return (int64_t)TICKS * (root - speed) / acceleration;
    }
    return (int64_t)TICKS * (cap - speed) / acceleration +
           (distance - rising) * (int64_t)TICKS / cap;
}

/* The soonest a vehicle could cover a distance: accelerating as hard as it
 * can to a little over the limit. */
static int64_t EarliestTicks(const Vehicle *vehicle, int64_t limit, int64_t distance)
{
    return CoverTicks(vehicle->speed, DrivingOf(vehicle)->acceleration,
                      FoxMax((int64_t)vehicle->speed, limit * 11 / 10), distance);
}

/* The latest a vehicle driving on covers a distance: never above nine
 * tenths of the limit, and with seven tenths of its acceleration, which is
 * about what the driver model averages easing into its speed. */
static int64_t LatestTicks(const Vehicle *vehicle, int64_t limit, int64_t distance)
{
    const int64_t cap = limit * 9 / 10;
    return CoverTicks(FoxMin((int64_t)vehicle->speed, cap),
                      DrivingOf(vehicle)->acceleration * 7 / 10, cap, distance);
}

/* Where a car stops along the way it holds, or HOLD_NONE when it drives
 * through. Once its light is red a car waiting inside the intersection only
 * leaves it: everyone else lets it. */
static int64_t HoldOf(const FoxNetwork *network, const Vehicle *vehicle)
{
    if (vehicle->holdAt == HOLD_NONE)
    {
        return HOLD_NONE;
    }
    return LightFor(network, vehicle->connection) == FOX_LIGHT_RED ? HOLD_NONE : vehicle->holdAt;
}

/* When a car would get to where its way meets another one at the earliest,
 * when its rear would be past it at the latest, and the gap it leaves
 * before anyone else comes. */
typedef struct Window
{
    int64_t arrive;
    int64_t clear;
    int64_t gap;
} Window;

/* How fast a car on a way, or about to take it, can go at most for the one
 * close ahead of it: no faster than that one, and at least a walking pace. */
static int64_t LeaderPace(const FoxTraffic *traffic, const FoxNetwork *network,
                          const Vehicle *holder, const FoxPath *way)
{
    const FoxPath *beyond = FoxNetworkPathRecord(network, way->to);
    const Vehicle *leader = NULL;
    if (holder->ahead != NONE)
    {
        leader = &traffic->vehicles[holder->ahead];
    }
    else if (way->front != NONE && FoxNetworkPathRecord(network, holder->path) != way)
    {
        leader = &traffic->vehicles[way->back];
    }
    else if (beyond->back != NONE && traffic->vehicles[beyond->back].s -
                                             DrivingOf(&traffic->vehicles[beyond->back])->length <
                                         LEADER_REACH)
    {
        leader = &traffic->vehicles[beyond->back];
    }
    return leader == NULL ? INT64_MAX : FoxMax((int64_t)leader->speed, FOX_METRES(1));
}

/* Whether a car on a way, or about to take it, `position` along it (negative
 * before its line) stands in the way of one with the window `mine` where the
 * two meet, `meet` along the way. It does not when it is through a margin
 * before, waits short of the meeting point, or comes a gap after. */
static bool HolderInWay(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *holder,
                        const FoxPath *way, int64_t position, int64_t meet, Window mine)
{
    const int64_t length = DrivingOf(holder)->length;
    if (position - length > meet + PASSED_MARGIN)
    {
        return false;
    }
    const int64_t hold = HoldOf(network, holder);
    if (hold != HOLD_NONE && hold <= meet - CLEARANCE)
    {
        return false;
    }
    const int64_t arrive = EarliestTicks(holder, way->speedLimit, meet - CLEARANCE - position);
    const int64_t clear =
        hold != HOLD_NONE && hold - length < meet + PASSED_MARGIN
            ? INT64_MAX
            : LatestTicks(holder,
                          FoxMin(way->speedLimit, LeaderPace(traffic, network, holder, way)),
                          meet + length + PASSED_MARGIN - position);
    const bool before = clear != INT64_MAX && clear + SAFETY_TICKS <= mine.arrive;
    return !before && arrive < mine.clear + mine.gap;
}

/* Whether the cars holding a way stand in the way of one meeting it: those
 * on it, those that have left it until their rear is past, and the one about
 * to enter it. Claims of cars not found among them come through a lane too
 * short to stop on and always do. */
static bool HoldersInWay(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t wayIndex,
                         const FoxPath *way, int64_t meet, Window mine)
{
    if (way->claims == 0u)
    {
        return false;
    }
    uint32_t holders = 0u;
    for (uint32_t slot = way->front; slot != NONE; slot = traffic->vehicles[slot].behind)
    {
        const Vehicle *holder = &traffic->vehicles[slot];
        ++holders;
        if (HolderInWay(traffic, network, holder, way, holder->s, meet, mine))
        {
            return true;
        }
    }
    const FoxPath *beyond = FoxNetworkPathRecord(network, way->to);
    for (uint32_t slot = beyond->back; slot != NONE && traffic->vehicles[slot].clearing == wayIndex;
         slot = traffic->vehicles[slot].ahead)
    {
        const Vehicle *leaving = &traffic->vehicles[slot];
        ++holders;
        if (HolderInWay(traffic, network, leaving, way, way->length + leaving->s, meet, mine))
        {
            return true;
        }
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, way->from);
    if (lane->front != NONE && traffic->vehicles[lane->front].claim == wayIndex)
    {
        const Vehicle *coming = &traffic->vehicles[lane->front];
        ++holders;
        if (HolderInWay(traffic, network, coming, way, coming->s - lane->length, meet, mine))
        {
            return true;
        }
    }
    return way->claims > holders;
}

/* A way given away earlier in this round of its intersection, and to whom. */
struct TakenWay
{
    uint32_t connection;
    uint32_t slot;
};

static bool TakenInWay(const FoxTraffic *traffic, const FoxNetwork *network, const TakenWay *taken,
                       uint32_t takenCount, uint32_t wayIndex, const FoxPath *way, int64_t meet,
                       Window mine)
{
    for (uint32_t index = 0u; index < takenCount; ++index)
    {
        if (taken[index].connection != wayIndex)
        {
            continue;
        }
        const Vehicle *holder = &traffic->vehicles[taken[index].slot];
        const FoxPath *path = FoxNetworkPathRecord(network, holder->path);
        const int64_t position = path->kind == PATH_LANE ? holder->s - path->length : holder->s;
        if (HolderInWay(traffic, network, holder, way, position, meet, mine))
        {
            return true;
        }
    }
    return false;
}

/* Whether the light or sign at the end of a lane will stop a car `distance`
 * before it. */
static bool WillStop(const FoxNetwork *network, const Vehicle *vehicle, const FoxPath *lane,
                     int64_t distance)
{
    const FoxLight light = vehicle->connection != NONE ? LightFor(network, vehicle->connection)
                                                       : LightAt(network, lane);
    if (light == FOX_LIGHT_RED)
    {
        return true;
    }
    if (light == FOX_LIGHT_YELLOW)
    {
        /* Whoever can still stop comfortably does. */
        return distance > BrakingDistance(vehicle) + FOX_METRES(2);
    }
    return light == FOX_LIGHT_NONE &&
           network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u] == FOX_SIGN_STOP &&
           !vehicle->stopped;
}

/* Whether a car's route still takes it onto the edge `target` beyond the
 * lane `laneIndex` it is coming to; a car without a route, or whose route
 * cannot go on from that lane and will be planned again there, might go
 * anywhere. */
static bool WillReach(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex,
                      uint32_t target)
{
    if (network->router == NULL || vehicle->routeLength == 0u ||
        PlanThrough(network, vehicle, laneIndex) == NONE)
    {
        return true;
    }
    const uint32_t reach = vehicle->routeIndex + UPSTREAM_LANES + 1u;
    const uint32_t last = vehicle->routeLength < reach ? vehicle->routeLength : reach;
    for (uint32_t index = vehicle->routeIndex; index < last; ++index)
    {
        if (vehicle->route[index] == target)
        {
            return true;
        }
    }
    return false;
}

/* Whether a car `toLine` before its line, `toMeet` before where it would
 * meet another one, could be there within `window` ticks, and for a driver
 * out of patience also could not stop at its line comfortably after
 * `reaction` ticks; `reaction` is negative while it is patient. */
static bool Coming(const Vehicle *vehicle, int64_t limit, int64_t toLine, int64_t toMeet,
                   int64_t window, int64_t reaction)
{
    if (EarliestTicks(vehicle, limit, toMeet) >= window)
    {
        return false;
    }
    return reaction < 0 || toLine <= BrakingDistance(vehicle) +
                                         (int64_t)vehicle->speed * reaction / (int64_t)TICKS +
                                         SPARE_ROOM;
}

/* A lane further back to look along: how far from its start the meeting
 * point lies, and how many lanes further back may still follow. */
typedef struct UpstreamLane
{
    uint32_t lane;
    uint32_t depth;
    int64_t toMeet;
} UpstreamLane;

/* Whether a car further back, before the lane `laneIndex`, could come onto
 * the edge `target` in time: from the intersections and lanes leading into
 * it, UPSTREAM_LANES lanes back. `toMeet` is measured from the start of the
 * lane. Such a car only sees the intersection once on the last lane before
 * it, and has no more than `room` to stop in. Nobody gets past the first car
 * of a lane a light or sign holds. Where there are more ways in than can be
 * followed, someone is taken to be coming. */
static bool Upstream(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t laneIndex,
                     int64_t room, int64_t toMeet, uint32_t target, int64_t window,
                     int64_t reaction)
{
    UpstreamLane pending[UPSTREAM_PENDING];
    uint32_t count = 0u;
    pending[count++] = (UpstreamLane){.lane = laneIndex, .depth = UPSTREAM_LANES, .toMeet = toMeet};
    while (count != 0u)
    {
        const UpstreamLane at = pending[--count];
        const FoxPath *lane = FoxNetworkPathRecord(network, at.lane);
        if (at.toMeet * (int64_t)TICKS /
                FoxMax(FOX_METRES(1), (int64_t)lane->speedLimit * 11 / 10) >=
            window)
        {
            continue;
        }
        const FoxRoad *road = &network->roads[lane->road];
        const FoxNode *node = &network->nodes[road->nodes[lane->direction == 0u ? 0u : 1u]];
        for (uint32_t index = 0u; index < node->connectionCount; ++index)
        {
            const uint32_t beforeIndex = node->connections[index];
            const FoxPath *before = FoxNetworkPathRecord(network, beforeIndex);
            if (before->to != at.lane)
            {
                continue;
            }
            for (uint32_t slot = before->front; slot != NONE; slot = traffic->vehicles[slot].behind)
            {
                const Vehicle *coming = &traffic->vehicles[slot];
                if (WillReach(network, coming, at.lane, target) &&
                    Coming(coming, lane->speedLimit, room, before->length - coming->s + at.toMeet,
                           window, reaction))
                {
                    return true;
                }
            }
            const FoxPath *previous = FoxNetworkPathRecord(network, before->from);
            bool held = false;
            for (uint32_t slot = previous->front; slot != NONE && !held;
                 slot = traffic->vehicles[slot].behind)
            {
                const Vehicle *coming = &traffic->vehicles[slot];
                const int64_t left = previous->length - coming->s + before->length;
                held = slot == previous->front &&
                       WillStop(network, coming, previous, previous->length - coming->s);
                if (!held && coming->connection == beforeIndex &&
                    WillReach(network, coming, at.lane, target) &&
                    Coming(coming, previous->speedLimit, room, left + at.toMeet, window, reaction))
                {
                    return true;
                }
            }
            if (held || at.depth <= 1u)
            {
                continue;
            }
            if (count == UPSTREAM_PENDING)
            {
                return true;
            }
            pending[count++] = (UpstreamLane){
                .lane = before->from,
                .depth = at.depth - 1u,
                .toMeet = at.toMeet + before->length + previous->length,
            };
        }
    }
    return false;
}

/* Whether a car that has not taken a way yet could get to where it meets
 * another one, `meet` along it, within `window` ticks: from the lane before
 * it, or from further back. Nobody gets past the first car of a lane while
 * a light or sign holds that one. Where no lights rule, one standing at its
 * line waiting for its own turn that came after the driver who asks, came
 * `arrival`, lets that one go first, and so does any for a driver out of
 * patience: no ring of cars waits for each other. */
static bool Approaching(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t wayIndex,
                        const FoxPath *way, int64_t meet, int64_t window, int64_t reaction,
                        uint64_t arrival)
{
    if (LightFor(network, wayIndex) == FOX_LIGHT_RED)
    {
        /* Nobody comes through a red light; once it turns green, the cars
         * there find this one in their way. */
        return false;
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, way->from);
    for (uint32_t slot = lane->front; slot != NONE; slot = traffic->vehicles[slot].behind)
    {
        const Vehicle *coming = &traffic->vehicles[slot];
        const int64_t distance = lane->length - coming->s;
        if (slot == lane->front &&
            (WillStop(network, coming, lane, distance) ||
             (LightAt(network, lane) == FOX_LIGHT_NONE && coming->speed <= STANDING_SPEED &&
              !coming->clear && !coming->permitted &&
              (reaction >= 0 || (coming->arrival != 0u && coming->arrival > arrival)))))
        {
            return false;
        }
        if (coming->connection == wayIndex && coming->claim != wayIndex &&
            Coming(coming, lane->speedLimit, distance, distance + meet - CLEARANCE, window,
                   reaction))
        {
            return true;
        }
    }
    const FoxPath *beyond = FoxNetworkPathRecord(network, way->to);
    return Upstream(traffic, network, way->from, lane->length, lane->length + meet - CLEARANCE,
                    EdgeOf(beyond), window, reaction);
}

/* How far along a connection one that conflicts with it meets it. */
static int64_t MeetingOn(const FoxNetwork *network, uint32_t connectionIndex, const FoxPath *other)
{
    for (uint32_t index = 0u; index < other->conflictCount; ++index)
    {
        if (other->conflicts[index].path == connectionIndex)
        {
            return other->conflicts[index].otherAt;
        }
    }
    return FoxNetworkPathRecord(network, connectionIndex)->length;
}

/* How far along its connection a car `toLine` before its line (negative
 * once past it) may go: up to the first meeting with a way a car holds or
 * was given in this round and stands in its way there, or with one a car
 * it gives way to is coming along; HOLD_NONE when it is free throughout.
 * `outMerge` tells whether that way runs into the same lane. */
static int64_t FirstBlocked(const FoxTraffic *traffic, const FoxNetwork *network,
                            const Vehicle *vehicle, int64_t toLine, int64_t gap, int64_t reaction,
                            const TakenWay *taken, uint32_t takenCount, bool giveWay,
                            bool *outMerge)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->connection);
    const int64_t length = DrivingOf(vehicle)->length;
    int64_t first = HOLD_NONE;
    *outMerge = false;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxConflict *conflict = &connection->conflicts[index];
        const FoxPath *other = FoxNetworkPathRecord(network, conflict->path);
        if (other == NULL)
        {
            continue;
        }
        const int64_t meet = MeetingOn(network, vehicle->connection, other);
        const int64_t toMeet = toLine + meet;
        if (meet >= first || toMeet + length + PASSED_MARGIN <= 0)
        {
            continue;
        }
        const Window mine = {
            .arrive = EarliestTicks(vehicle, connection->speedLimit, toMeet - CLEARANCE),
            .clear = LatestTicks(vehicle, connection->speedLimit, toMeet + length + PASSED_MARGIN),
            .gap = gap,
        };
        if (HoldersInWay(traffic, network, conflict->path, other, conflict->otherAt, mine) ||
            TakenInWay(traffic, network, taken, takenCount, conflict->path, other,
                       conflict->otherAt, mine) ||
            (giveWay && conflict->yields &&
             Approaching(traffic, network, conflict->path, other, conflict->otherAt,
                         mine.clear + gap, reaction, vehicle->arrival)))
        {
            first = meet;
            *outMerge = other->to == connection->to;
        }
    }
    return first;
}

/* Whether a car `toLine` before the line of an intersection further ahead
 * would find cars holding ways that cross its connection there in its way:
 * a driver sees that coming before reaching the lane up to it. */
static bool HeldOn(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *vehicle,
                   uint32_t connectionIndex, int64_t toLine)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    const int64_t length = DrivingOf(vehicle)->length;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxConflict *conflict = &connection->conflicts[index];
        const FoxPath *other = FoxNetworkPathRecord(network, conflict->path);
        if (other == NULL)
        {
            continue;
        }
        const int64_t toMeet = toLine + MeetingOn(network, connectionIndex, other);
        const Window mine = {
            .arrive = EarliestTicks(vehicle, connection->speedLimit, toMeet - CLEARANCE),
            .clear = LatestTicks(vehicle, connection->speedLimit, toMeet + length + PASSED_MARGIN),
            .gap = GAP_TICKS,
        };
        if (HoldersInWay(traffic, network, conflict->path, other, conflict->otherAt, mine))
        {
            return true;
        }
    }
    return false;
}

/* The same for the way a car takes at the end of a lane, `onward`, or for
 * every way there when its route cannot go on from that lane. */
static bool HeldAhead(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *vehicle,
                      const FoxPath *lane, uint32_t onward, int64_t toLine)
{
    if (onward != NONE)
    {
        return HeldOn(traffic, network, vehicle, onward, toLine);
    }
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        if (HeldOn(traffic, network, vehicle, lane->outgoing[index], toLine))
        {
            return true;
        }
    }
    return false;
}

/* Whether any way crossing a connection is held. */
static bool AnyClaimCrosses(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxPath *other = FoxNetworkPathRecord(network, connection->conflicts[index].path);
        if (other != NULL && other->claims > 0u)
        {
            return true;
        }
    }
    return false;
}

/* The car that left a lane by another way than `except` and whose rear is
 * not yet past where the ways part, the rearmost one; NONE without one.
 * `outRear` is how far past the line its rear is. */
static uint32_t Departing(const FoxTraffic *traffic, const FoxNetwork *network, const FoxPath *lane,
                          uint32_t except, int64_t *outRear)
{
    uint32_t found = NONE;
    *outRear = PARTING;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        if (lane->outgoing[index] == except || connection->back == NONE)
        {
            continue;
        }
        const Vehicle *last = &traffic->vehicles[connection->back];
        const int64_t rear = last->s - DrivingOf(last)->length;
        if (rear < *outRear)
        {
            *outRear = rear;
            found = connection->back;
        }
    }
    return found;
}

/* How far along a connection it first meets a way it gives way to that may
 * be driven now: a car waiting inside stops short of all of them, not
 * between them. */
static int64_t FirstGiveWay(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    int64_t first = HOLD_NONE;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxConflict *conflict = &connection->conflicts[index];
        const FoxPath *other = FoxNetworkPathRecord(network, conflict->path);
        if (other != NULL && conflict->yields && LightFor(network, conflict->path) != FOX_LIGHT_RED)
        {
            first = FoxMin(first, MeetingOn(network, connectionIndex, other));
        }
    }
    return first;
}

/* Whether a car may drive into the intersection to wait for a gap in the
 * oncoming traffic, its front at `hold` along its connection: when it turns
 * across that traffic on a green, nobody waits on its way already, and
 * standing there it is in the way of no way that may be driven while it
 * waits, which only a red light makes sure of. */
static bool MayWaitInside(const FoxTraffic *traffic, const FoxNetwork *network,
                          const Vehicle *vehicle, const FoxPath *lane, const FoxPath *connection,
                          int64_t hold)
{
    if (connection->movement != MOVE_FAR || LightAt(network, lane) != FOX_LIGHT_GREEN)
    {
        return false;
    }
    for (uint32_t slot = connection->front; slot != NONE; slot = traffic->vehicles[slot].behind)
    {
        if (traffic->vehicles[slot].holdAt != HOLD_NONE)
        {
            return false;
        }
    }
    const int64_t rear = hold - DrivingOf(vehicle)->length - PASSED_MARGIN;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxPath *other = FoxNetworkPathRecord(network, connection->conflicts[index].path);
        if (other == NULL)
        {
            continue;
        }
        const int64_t meet = MeetingOn(network, vehicle->connection, other);
        if (meet + CLEARANCE > rear && meet - CLEARANCE < hold &&
            LightFor(network, connection->conflicts[index].path) != FOX_LIGHT_RED)
        {
            return false;
        }
    }
    return true;
}

/* A car taking part in its intersection's round. */
struct Candidate
{
    uint32_t node;
    int64_t rank;
    uint64_t arrival;
    uint64_t serial;
    uint32_t slot;
    /* How far the car is from its line; negative once inside. */
    int64_t toLine;
};

static int CompareCandidates(const void *left, const void *right)
{
    const Candidate *a = (const Candidate *)left;
    const Candidate *b = (const Candidate *)right;
    if (a->node != b->node)
    {
        return a->node < b->node ? -1 : 1;
    }
    if (a->rank != b->rank)
    {
        return a->rank < b->rank ? -1 : 1;
    }
    if (a->arrival != b->arrival)
    {
        return a->arrival < b->arrival ? -1 : 1;
    }
    return (a->serial > b->serial) - (a->serial < b->serial);
}

/* The order of a round: the road with right of way first, of each road
 * those inside the intersection, then going straight, turning to the curb,
 * across and back. */
static int64_t Rank(const FoxNetwork *network, const FoxPath *lane, const FoxPath *connection,
                    bool inside)
{
    return (2 - SignClass(network, lane)) * 8 + (inside ? 0 : 4) + (int64_t)connection->movement;
}

/* Whether the light or sign at the end of a lane holds a car back for now;
 * at a stop sign it stands at the line first. */
static bool HeldAtLine(const FoxNetwork *network, Vehicle *vehicle, const FoxPath *lane,
                       int64_t distance)
{
    if (LightAt(network, lane) == FOX_LIGHT_NONE && !vehicle->stopped &&
        network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u] == FOX_SIGN_STOP)
    {
        vehicle->stopped = vehicle->speed <= STANDING_SPEED && distance <= STOP_LINE_REACH;
        return !vehicle->stopped;
    }
    return WillStop(network, vehicle, lane, distance);
}

/* A car that holds its way but has not reached its line gives the way back
 * when its light turns yellow or red and it can still stop comfortably, and
 * when it stands with no room beyond or behind a car from its lane still in
 * the way: holding it would only block the cars crossing, and it takes its
 * turn again once it can go. */
static bool GiveBack(const FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle)
{
    if (vehicle->through != NONE || vehicle->claim != vehicle->connection)
    {
        return false;
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    const FoxLight light = LightFor(network, vehicle->connection);
    const bool standing = vehicle->speed <= STANDING_SPEED;
    const bool stop =
        (light == FOX_LIGHT_RED || light == FOX_LIGHT_YELLOW) &&
        lane->length - vehicle->s >= BrakingDistance(vehicle) + (standing ? 0 : FOX_METRES(2));
    if (!stop && !standing)
    {
        return false;
    }
    const uint32_t beyond = FoxNetworkPathRecord(network, vehicle->claim)->to;
    int64_t rear = 0;
    Arrive(network, vehicle->claim, vehicle);
    if (!stop && Departing(traffic, network, lane, vehicle->connection, &rear) == NONE &&
        RoomOn(traffic, network, beyond, DrivingOf(vehicle)->length))
    {
        /* There is room once its own share is not counted: it keeps the way. */
        PathOf(network, beyond)->incoming += (uint32_t)DrivingOf(vehicle)->length;
        return false;
    }
    Release(network, &vehicle->claim);
    vehicle->permitted = false;
    vehicle->clear = false;
    vehicle->holdAt = HOLD_NONE;
    return true;
}

/* Whether the candidate arrays have room for one more. */
static bool CandidateRoom(FoxTraffic *traffic, uint32_t count)
{
    if (count < traffic->candidateCapacity)
    {
        return true;
    }
    const uint32_t capacity = count < 64u ? 64u : count * 2u;
    Candidate *grown = realloc(traffic->candidates, (size_t)capacity * sizeof(*grown));
    if (grown != NULL)
    {
        traffic->candidates = grown;
    }
    TakenWay *taken = realloc(traffic->taken, (size_t)capacity * sizeof(*taken));
    if (taken != NULL)
    {
        traffic->taken = taken;
    }
    if (grown == NULL || taken == NULL)
    {
        return false;
    }
    traffic->candidateCapacity = capacity;
    return true;
}

/* Each intersection hands out its way in, one round per tick, by the rules
 * of the road. The cars near the front of its lanes take part, and those
 * waiting inside it; the road with right of way goes first. A car may go
 * when no car holding or given a way it crosses would be there with it, and
 * none it gives way to could come within a gap of its being through: by the
 * sign, from the curb side where the roads are equal, and the oncoming
 * traffic when it turns across it. A car standing at its line waiting for
 * its own turn lets the others go, so no ring of cars waits for each other.
 * A car turning across the oncoming lanes drives in up to their way and
 * waits there, and once its light is red it leaves before anyone else
 * comes. The lane beyond must have room. A car claims its way only once it
 * could no longer stop comfortably; until then one with right of way can
 * still come first. */
static void Arbitrate(FoxTraffic *traffic, FoxNetwork *network)
{
    uint32_t count = 0u;
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive || vehicle->connection == NONE)
        {
            continue;
        }
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        const bool inside = path->kind == PATH_CONNECTION;
        int64_t toLine = -vehicle->s;
        if (inside)
        {
            if (vehicle->holdAt == HOLD_NONE)
            {
                continue;
            }
        }
        else
        {
            if (path->front != slot || (vehicle->permitted && !GiveBack(traffic, network, vehicle)))
            {
                continue;
            }
            vehicle->holdAt = HOLD_NONE;
            toLine = path->length - vehicle->s;
            const int64_t zone =
                BrakingDistance(vehicle) + (int64_t)vehicle->speed * 2 + DECISION_MARGIN;
            if (toLine > zone)
            {
                /* Far from the line: drive on, decide later. */
                vehicle->clear = true;
                vehicle->arrival = 0u;
                continue;
            }
            if (HeldAtLine(network, vehicle, path, toLine))
            {
                vehicle->clear = false;
                vehicle->arrival = 0u;
                PathOf(network, vehicle->connection)->waited = true;
                continue;
            }
            if (vehicle->arrival == 0u)
            {
                vehicle->arrival = traffic->tick;
            }
        }
        if (!CandidateRoom(traffic, count))
        {
            /* Without memory the car simply waits. */
            vehicle->clear = inside;
            continue;
        }
        const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->connection);
        traffic->candidates[count++] = (Candidate){
            .node = connection->node,
            .rank =
                Rank(network, FoxNetworkPathRecord(network, connection->from), connection, inside),
            .arrival = vehicle->arrival,
            .serial = vehicle->serial,
            .slot = slot,
            .toLine = toLine,
        };
    }
    if (count == 0u)
    {
        return;
    }
    qsort(traffic->candidates, count, sizeof(*traffic->candidates), CompareCandidates);
    uint32_t takenCount = 0u;
    for (uint32_t index = 0u; index < count; ++index)
    {
        const Candidate *candidate = &traffic->candidates[index];
        if (index == 0u || traffic->candidates[index - 1u].node != candidate->node)
        {
            takenCount = 0u;
        }
        Vehicle *vehicle = &traffic->vehicles[candidate->slot];
        const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->connection);
        const FoxPath *lane = FoxNetworkPathRecord(network, connection->from);
        const bool inside = FoxNetworkPathRecord(network, vehicle->path)->kind == PATH_CONNECTION;
        const bool leaving = inside && LightFor(network, vehicle->connection) == FOX_LIGHT_RED;
        /* The longer a driver has waited, the shorter the gap it takes. */
        const uint64_t waited =
            inside ? vehicle->waitTicks
                   : traffic->tick - (vehicle->arrival != 0u ? vehicle->arrival : traffic->tick);
        const int64_t patience = (int64_t)PATIENCE_TICKS;
        const int64_t wait = (int64_t)(waited < 2u * PATIENCE_TICKS ? waited : 2u * PATIENCE_TICKS);
        const int64_t gap =
            GAP_TICKS - (GAP_TICKS - MIN_GAP_TICKS) * FoxMin(wait, patience) / patience;
        bool merge = false;
        const int64_t reaction =
            wait < patience ? -1 : REACTION_TICKS - REACTION_TICKS * (wait - patience) / patience;
        const int64_t blocked =
            FirstBlocked(traffic, network, vehicle, candidate->toLine, gap, reaction,
                         traffic->taken, takenCount, !leaving, &merge);
        if (inside)
        {
            /* Waiting inside: on through once the way is free. */
            if (blocked == HOLD_NONE)
            {
                vehicle->holdAt = HOLD_NONE;
                traffic->taken[takenCount++] =
                    (TakenWay){.connection = vehicle->connection, .slot = candidate->slot};
            }
            continue;
        }
        int64_t hold = HOLD_NONE;
        int64_t rear = 0;
        if (blocked != HOLD_NONE)
        {
            hold = FoxMin(blocked, FirstGiveWay(network, vehicle->connection)) - CLEARANCE;
            if (merge || hold < MIN_HOLD ||
                !MayWaitInside(traffic, network, vehicle, lane, connection, hold))
            {
                vehicle->clear = false;
                PathOf(network, vehicle->connection)->waited = true;
                continue;
            }
        }
        const FoxPath *beyond = FoxNetworkPathRecord(network, connection->to);
        const int64_t length = DrivingOf(vehicle)->length;
        const uint32_t through = hold == HOLD_NONE && beyond->length < length + 2 * MIN_GAP
                                     ? PlanThrough(network, vehicle, connection->to)
                                     : NONE;
        bool clear = false;
        if (Departing(traffic, network, lane, vehicle->connection, &rear) != NONE)
        {
            /* A car from the same lane is still where the ways part. */
            clear = false;
        }
        else if (through != NONE)
        {
            /* Beyond a lane too short to stop on the next intersection must
             * be free as well. */
            clear = beyond->back == NONE && beyond->incoming == 0u &&
                    !AnyClaimCrosses(network, through) &&
                    RoomOn(traffic, network, FoxNetworkPathRecord(network, through)->to, length);
        }
        else
        {
            clear = RoomOn(traffic, network, connection->to, length);
        }
        vehicle->clear = clear;
        if (!clear)
        {
            PathOf(network, vehicle->connection)->waited = true;
            continue;
        }
        vehicle->holdAt = hold;
        traffic->taken[takenCount++] =
            (TakenWay){.connection = vehicle->connection, .slot = candidate->slot};
        if (candidate->toLine <= BrakingDistance(vehicle) + FOX_METRES(4))
        {
            Claim(network, vehicle->connection, vehicle);
            vehicle->claim = vehicle->connection;
            vehicle->permitted = true;
            vehicle->arrival = 0u;
            if (through != NONE)
            {
                Claim(network, through, vehicle);
                vehicle->through = through;
            }
        }
    }
}
/* ---- Driving ------------------------------------------------------------------- */

/* How far ahead a car waits inside the intersection, or HOLD_NONE. */
static int64_t ToHold(const FoxNetwork *network, const Vehicle *vehicle)
{
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    if (vehicle->holdAt == HOLD_NONE)
    {
        return HOLD_NONE;
    }
    if (path->kind == PATH_CONNECTION)
    {
        return vehicle->holdAt - vehicle->s;
    }
    return vehicle->permitted || vehicle->clear ? path->length - vehicle->s + vehicle->holdAt
                                                : HOLD_NONE;
}

/* The speed a driver aims for under a limit: a little above or below it,
 * never above what the vehicle can do. */
static int64_t ScaledLimit(const Vehicle *vehicle, int32_t limit)
{
    return FoxMin((int64_t)limit * vehicle->desire / FOX_UNIT, DrivingOf(vehicle)->topSpeed);
}

/* The intelligent driver model in integers: free-road acceleration towards
 * the desired speed minus the interaction with the leader. */
static int32_t DriverModel(const Driving *driving, int64_t speed, int64_t desired, int64_t gap,
                           int64_t leaderSpeed, bool hasLeader)
{
    int64_t freeTerm = FOX_UNIT * 16;
    if (desired > 0)
    {
        const int64_t ratio = FoxMin(speed * FOX_UNIT / desired, 4 * FOX_UNIT);
        const int64_t square = ratio * ratio / FOX_UNIT;
        freeTerm = square * square / FOX_UNIT;
    }
    int64_t interaction = 0;
    if (hasLeader)
    {
        if (gap <= 0)
        {
            return -MAX_BRAKE;
        }
        /* sqrt(a * b) of the model's two accelerations. */
        const int64_t rootAB =
            (int64_t)FoxIntegerSqrt((uint64_t)(driving->acceleration * driving->brake));
        const int64_t closing = speed * (speed - leaderSpeed) / (2 * rootAB);
        const int64_t wanted = MIN_GAP + FoxMax(0, speed * driving->headwayMs / 1000 + closing);
        const int64_t ratio = FoxMin(wanted * FOX_UNIT / gap, 16 * FOX_UNIT);
        interaction = ratio * ratio / FOX_UNIT;
    }
    const int64_t acceleration =
        driving->acceleration * (FOX_UNIT - freeTerm - interaction) / FOX_UNIT;
    return (int32_t)FoxClamp(acceleration, -MAX_BRAKE, driving->acceleration);
}

static void Accelerate(const FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    const Driving *driving = DrivingOf(vehicle);
    const int64_t speed = vehicle->speed;
    int64_t desired = ScaledLimit(vehicle, path->speedLimit);
    /* Over the limit of the path itself: comfortable braking at least. */
    int64_t needed = speed > desired ? driving->brake : 0;
    int64_t gap = 0;
    int64_t leaderSpeed = 0;
    bool hasLeader = false;
    if (vehicle->ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[vehicle->ahead];
        gap = leader->s - DrivingOf(leader)->length - vehicle->s;
        leaderSpeed = leader->speed;
        hasLeader = true;
    }
    int64_t travelled = path->length - vehicle->s;
    uint32_t next = NONE;
    int64_t rear = 0;
    const uint32_t departing = path->kind == PATH_LANE && !hasLeader
                                   ? Departing(traffic, network, path, vehicle->connection, &rear)
                                   : NONE;
    if (departing != NONE)
    {
        /* A car from this lane still where the ways part. */
        gap = travelled + rear;
        leaderSpeed = traffic->vehicles[departing].speed;
        hasLeader = true;
    }
    if (path->kind == PATH_LANE)
    {
        if (vehicle->connection == NONE || (!vehicle->permitted && !vehicle->clear))
        {
            /* The stop line, or the end of the road. */
            if (!hasLeader || travelled < gap)
            {
                gap = travelled;
                leaderSpeed = 0;
                hasLeader = true;
            }
        }
        else
        {
            next = vehicle->connection;
        }
    }
    else
    {
        next = path->to;
    }
    bool lookedAhead = false;
    while (next != NONE && travelled < LOOKAHEAD)
    {
        const FoxPath *upcoming = FoxNetworkPathRecord(network, next);
        if (upcoming == NULL)
        {
            break;
        }
        /* Slow down in time for a slower path ahead: aim for a speed from
         * which comfortable braking makes it, and brake at least as hard as
         * still reaching it needs. */
        const int64_t limit = ScaledLimit(vehicle, upcoming->speedLimit);
        const int64_t ahead = FoxMax(FOX_METRES(1), travelled);
        const int64_t reachable =
            (int64_t)FoxIntegerSqrt((uint64_t)(limit * limit + 2 * driving->brake * ahead));
        desired = FoxMin(desired, reachable);
        if (speed > limit)
        {
            needed = FoxMax(needed, (speed * speed - limit * limit) / (2 * ahead));
        }
        if (!hasLeader && upcoming->back != NONE)
        {
            const Vehicle *leader = &traffic->vehicles[upcoming->back];
            gap = travelled + leader->s - DrivingOf(leader)->length;
            leaderSpeed = leader->speed;
            hasLeader = true;
        }
        if (hasLeader)
        {
            break;
        }
        uint32_t onward = NONE;
        if (upcoming->kind == PATH_LANE && !lookedAhead && vehicle->routeLength != 0u)
        {
            /* The intersection at the end of the next lane, seen from here:
             * where cars hold the way the route takes, stop at its line, and
             * else slow down in time for that way. */
            lookedAhead = true;
            onward = PlanThrough(network, vehicle, next);
            if (HeldAhead(traffic, network, vehicle, upcoming, onward,
                          travelled + upcoming->length))
            {
                gap = travelled + upcoming->length;
                leaderSpeed = 0;
                hasLeader = true;
                break;
            }
        }
        travelled += upcoming->length;
        next = upcoming->kind == PATH_CONNECTION ? upcoming->to : onward;
    }
    const int64_t hold = ToHold(network, vehicle);
    if (hold != HOLD_NONE && (!hasLeader || hold < gap))
    {
        /* Where it waits inside the intersection. */
        gap = hold;
        leaderSpeed = 0;
        hasLeader = true;
    }
    int32_t acceleration =
        DriverModel(driving, vehicle->speed, desired, gap, leaderSpeed, hasLeader);
    if (needed > 0)
    {
        acceleration = (int32_t)FoxMin(acceleration, -FoxMin(needed, MAX_BRAKE));
    }
    vehicle->acceleration = acceleration;
}

static void Integrate(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    int64_t speed = FoxMax(0, (int64_t)vehicle->speed + vehicle->acceleration / TICKS);
    int64_t advance = ((int64_t)vehicle->speed + speed) / ((int64_t)TICKS * 2);
    /* Hard limits: never into the car ahead, never over a line without leave. */
    int64_t room = INT64_MAX;
    int64_t roomSpeed = 0;
    if (vehicle->ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[vehicle->ahead];
        room = leader->s - DrivingOf(leader)->length - FOX_METRES(1) / 2 - vehicle->s;
        roomSpeed = leader->speed;
    }
    else if (path->kind == PATH_LANE && (vehicle->connection == NONE || !vehicle->permitted))
    {
        room = path->length - FOX_METRES(1) / 4 - vehicle->s;
    }
    else
    {
        const uint32_t next = path->kind == PATH_LANE ? vehicle->connection : path->to;
        const FoxPath *upcoming = FoxNetworkPathRecord(network, next);
        if (upcoming != NULL && upcoming->back != NONE)
        {
            const Vehicle *leader = &traffic->vehicles[upcoming->back];
            room = path->length - vehicle->s + leader->s - DrivingOf(leader)->length -
                   FOX_METRES(1) / 2;
            roomSpeed = leader->speed;
        }
    }
    int64_t rear = 0;
    const uint32_t departing = path->kind == PATH_LANE && vehicle->ahead == NONE
                                   ? Departing(traffic, network, path, vehicle->connection, &rear)
                                   : NONE;
    if (departing != NONE && path->length - vehicle->s + rear - FOX_METRES(1) / 2 < room)
    {
        room = path->length - vehicle->s + rear - FOX_METRES(1) / 2;
        roomSpeed = traffic->vehicles[departing].speed;
    }
    const int64_t hold = ToHold(network, vehicle);
    if (hold < room)
    {
        room = hold;
        roomSpeed = 0;
    }
    if (advance > room)
    {
        advance = FoxMax(0, room);
        speed = FoxMin(speed, roomSpeed);
    }
    vehicle->s += advance;
    vehicle->speed = (int32_t)speed;
    if (speed <= STANDING_SPEED)
    {
        ++vehicle->waitTicks;
    }
    else if (speed > MOVING_SPEED)
    {
        vehicle->waitTicks = 0u;
    }
}

/* Remembers the path a vehicle leaves, for drawing the rest of its body. */
static void PushTrail(Vehicle *vehicle, const FoxPath *path)
{
    for (uint32_t index = TRAIL - 1u; index > 0u; --index)
    {
        vehicle->trail[index] = vehicle->trail[index - 1u];
        vehicle->trailGeneration[index] = vehicle->trailGeneration[index - 1u];
        vehicle->trailLength[index] = vehicle->trailLength[index - 1u];
    }
    vehicle->trail[0] = vehicle->path;
    vehicle->trailGeneration[0] = vehicle->pathGeneration;
    vehicle->trailLength[0] = path->length;
    vehicle->trailCount = vehicle->trailCount < TRAIL ? vehicle->trailCount + 1u : TRAIL;
}

/* Moves a car onto the next paths it has reached; false when it left the
 * road. */
static bool Transfer(FoxTraffic *traffic, FoxNetwork *network, uint32_t slot)
{
    Vehicle *vehicle = &traffic->vehicles[slot];
    for (uint32_t step = 0u; step < 4u; ++step)
    {
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        if (vehicle->s < path->length)
        {
            break;
        }
        const int64_t over = vehicle->s - path->length;
        if (path->kind == PATH_LANE)
        {
            if (vehicle->connection == NONE)
            {
                Kill(traffic, network, slot, true);
                return false;
            }
            if (!vehicle->permitted)
            {
                vehicle->s = path->length - 1;
                break;
            }
            const uint32_t connection = vehicle->connection;
            PushTrail(vehicle, path);
            Unlink(traffic, network, slot);
            Link(traffic, network, slot, connection, over);
        }
        else
        {
            const uint32_t to = path->to;
            PushTrail(vehicle, path);
            Unlink(traffic, network, slot);
            Link(traffic, network, slot, to, over);
            /* The claim now covers the connection being left behind. */
            Release(network, &vehicle->clearing);
            if (vehicle->claim != NONE)
            {
                Arrive(network, vehicle->claim, vehicle);
            }
            vehicle->clearing = vehicle->claim;
            vehicle->claim = NONE;
            vehicle->permitted = false;
            vehicle->holdAt = HOLD_NONE;
            EnterLane(traffic, network, vehicle);
        }
    }
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    /* The connection behind is free once the car's rear has left it. */
    if (vehicle->clearing != NONE && path->kind == PATH_LANE &&
        vehicle->s >= DrivingOf(vehicle)->length)
    {
        Release(network, &vehicle->clearing);
    }
    if (vehicle->waitTicks > STUCK_TICKS)
    {
        ++traffic->stuck;
        Kill(traffic, network, slot, true);
        return false;
    }
    if (vehicle->parking && path->kind == PATH_LANE && vehicle->clearing == NONE &&
        vehicle->s >= path->length / 2)
    {
        Kill(traffic, network, slot, true);
        return false;
    }
    return true;
}

static void UpdateCosts(FoxNetwork *network)
{
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        FoxRoad *road = &network->roads[index];
        if (!road->alive)
        {
            continue;
        }
        const int32_t limit = road->speedLimit;
        for (uint32_t direction = 0u; direction < 2u; ++direction)
        {
            if (road->laneCount[direction] == 0u)
            {
                continue;
            }
            const int64_t sample = road->speedSamples[direction] != 0u
                                       ? road->speedSum[direction] / road->speedSamples[direction]
                                       : limit;
            const int64_t average =
                FoxClamp(((int64_t)road->averageSpeed[direction] * 3 + sample) / 4,
                         FOX_METRES(1) / 2, limit);
            road->averageSpeed[direction] = (int32_t)average;
            road->speedSum[direction] = 0;
            road->speedSamples[direction] = 0u;
            const uint32_t cost =
                (uint32_t)FoxClamp(road->length * 1000 / average, 1, UINT32_MAX - 1u);
            const uint32_t routed = road->routedCost[direction];
            const uint32_t change = cost > routed ? cost - routed : routed - cost;
            if (network->router != NULL && (uint64_t)change * 10u > routed)
            {
                road->routedCost[direction] = cost;
                network->router->setEdge(network->router->context, index * 2u + direction,
                                         road->nodes[direction], road->nodes[1u - direction], cost);
            }
        }
    }
}

void FoxTrafficStep(FoxTraffic *traffic, FoxNetwork *network)
{
    if (traffic == NULL || network == NULL)
    {
        return;
    }
    ++traffic->tick;
    FoxSignalsStep(network);
    Spawn(traffic, network);
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (vehicle->alive)
        {
            vehicle->prevPath = vehicle->path;
            vehicle->prevGeneration = vehicle->pathGeneration;
            vehicle->prevS = vehicle->s;
        }
    }
    Arbitrate(traffic, network);
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        if (traffic->vehicles[slot].alive)
        {
            Accelerate(traffic, network, &traffic->vehicles[slot]);
        }
    }
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        if (traffic->vehicles[slot].alive)
        {
            Integrate(traffic, network, &traffic->vehicles[slot]);
        }
    }
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive || !Transfer(traffic, network, slot))
        {
            continue;
        }
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        if (path->kind == PATH_LANE)
        {
            FoxRoad *road = &network->roads[path->road];
            road->speedSum[path->direction] += vehicle->speed;
            ++road->speedSamples[path->direction];
        }
    }
    if (traffic->tick % STATS_TICKS == 0u)
    {
        UpdateCosts(network);
    }
}

/* ---- Network changes ----------------------------------------------------------- */

/* Follows replaced roads to the one now holding a centre-line distance. */
static bool ResolveRoad(const FoxNetwork *network, uint32_t *road, uint32_t generation,
                        int64_t *centre)
{
    for (uint32_t depth = 0u; depth < 32u; ++depth)
    {
        if (*road < network->roadCapacity && network->roads[*road].alive &&
            network->roads[*road].generation == generation)
        {
            return true;
        }
        bool moved = false;
        for (uint32_t index = 0u; index < network->lineageCount && !moved; ++index)
        {
            const FoxLineage *entry = &network->lineage[index];
            if (entry->oldRoad == *road && entry->oldGeneration == generation &&
                *centre >= entry->start && *centre < entry->end)
            {
                *road = entry->road;
                generation = entry->generation;
                *centre += entry->offset;
                moved = true;
            }
        }
        if (!moved)
        {
            return false;
        }
    }
    return false;
}

/* The live lane that replaced a lane around a centre-line distance. */
static uint32_t ResolveLane(const FoxNetwork *network, uint32_t laneIndex, int64_t *centre)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    if (lane == NULL || lane->kind != PATH_LANE)
    {
        return NONE;
    }
    if (!lane->retired)
    {
        return laneIndex;
    }
    uint32_t road = lane->road;
    if (!ResolveRoad(network, &road, lane->roadGeneration, centre))
    {
        return NONE;
    }
    const FoxRoad *record = &network->roads[road];
    return lane->lane < record->laneCount[lane->direction]
               ? record->lanes[lane->direction][lane->lane]
               : NONE;
}

static uint32_t MapPath(const FoxNetwork *network, uint32_t pathIndex, int64_t s, int64_t *outS)
{
    const FoxPath *old = FoxNetworkPathRecord(network, pathIndex);
    if (old == NULL)
    {
        return NONE;
    }
    if (old->kind == PATH_LANE)
    {
        int64_t centre = FoxLaneToCentre(old, s);
        const uint32_t lane = ResolveLane(network, pathIndex, &centre);
        const FoxPath *record = FoxNetworkPathRecord(network, lane);
        if (record == NULL)
        {
            return NONE;
        }
        *outS = FoxClamp(FoxCentreToLane(record, centre), 0, record->length - 1);
        return lane;
    }
    /* A connection maps to the one joining the successors of its lanes. */
    const FoxPath *from = FoxNetworkPathRecord(network, old->from);
    const FoxPath *to = FoxNetworkPathRecord(network, old->to);
    if (from == NULL || to == NULL || old->node >= network->nodeCapacity ||
        !network->nodes[old->node].alive)
    {
        return NONE;
    }
    int64_t fromCentre = from->centre[from->count - 1u];
    int64_t toCentre = to->centre[0];
    const uint32_t newFrom = ResolveLane(network, old->from, &fromCentre);
    const uint32_t newTo = ResolveLane(network, old->to, &toCentre);
    const FoxNode *node = &network->nodes[old->node];
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        const FoxPath *candidate = FoxNetworkPathRecord(network, node->connections[index]);
        if (candidate->from == newFrom && candidate->to == newTo)
        {
            *outS =
                FoxClamp(s * candidate->length / FoxMax(1, old->length), 0, candidate->length - 1);
            return node->connections[index];
        }
    }
    return NONE;
}

void FoxTrafficRemap(FoxTraffic *traffic, FoxNetwork *network)
{
    if (traffic == NULL || network == NULL)
    {
        return;
    }
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive)
        {
            continue;
        }
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        if (path != NULL && !path->retired)
        {
            continue;
        }
        Unlink(traffic, network, slot);
        int64_t s = 0;
        const uint32_t moved = MapPath(network, vehicle->path, vehicle->s, &s);
        if (moved == NONE)
        {
            Kill(traffic, network, slot, false);
            continue;
        }
        const FoxPath *target = FoxNetworkPathRecord(network, moved);
        uint32_t ahead = NONE;
        uint32_t behind = NONE;
        FindPlace(traffic, target, s, &ahead, &behind);
        const Vehicle *leader = ahead != NONE ? &traffic->vehicles[ahead] : NULL;
        if ((leader != NULL && leader->s - s < DrivingOf(leader)->length) ||
            (behind != NONE && s - traffic->vehicles[behind].s < DrivingOf(vehicle)->length))
        {
            Kill(traffic, network, slot, false);
            continue;
        }
        Link(traffic, network, slot, moved, s);
        vehicle->trailCount = 0u;
        vehicle->holdAt = HOLD_NONE;
    }
    /* Claims are counted afresh from the cars that still hold live ones. */
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        network->paths[index].claims = 0u;
        network->paths[index].incoming = 0u;
    }
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive)
        {
            continue;
        }
        const uint32_t length = (uint32_t)DrivingOf(vehicle)->length;
        if (vehicle->claim != NONE && FoxNetworkPathLive(network, vehicle->claim))
        {
            ++network->paths[vehicle->claim].claims;
            network->paths[network->paths[vehicle->claim].to].incoming += length;
        }
        else
        {
            vehicle->claim = NONE;
            vehicle->permitted = false;
        }
        if (vehicle->through != NONE && vehicle->claim != NONE &&
            FoxNetworkPathLive(network, vehicle->through))
        {
            ++network->paths[vehicle->through].claims;
            network->paths[network->paths[vehicle->through].to].incoming += length;
        }
        else
        {
            vehicle->through = NONE;
        }
        if (vehicle->clearing != NONE && FoxNetworkPathLive(network, vehicle->clearing))
        {
            ++network->paths[vehicle->clearing].claims;
        }
        else
        {
            vehicle->clearing = NONE;
        }
        /* A turn that went away is chosen again; a claim on it went with it. */
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        if (path->kind == PATH_LANE &&
            (vehicle->connection == NONE || !FoxNetworkPathLive(network, vehicle->connection)))
        {
            ChooseConnection(traffic, network, vehicle);
            vehicle->permitted = false;
        }
        vehicle->prevPath = vehicle->path;
        vehicle->prevGeneration = vehicle->pathGeneration;
        vehicle->prevS = vehicle->s;
    }
}

/* ---- Views ----------------------------------------------------------------------- */

bool FoxTrafficVehicle(const FoxTraffic *traffic, uint32_t slot, FoxVehicleView *outView)
{
    if (traffic == NULL || outView == NULL || slot >= traffic->capacity ||
        !traffic->vehicles[slot].alive)
    {
        return false;
    }
    const Vehicle *vehicle = &traffic->vehicles[slot];
    *outView = (FoxVehicleView){
        .serial = vehicle->serial,
        .path = vehicle->path,
        .distance = vehicle->s,
        .speed = vehicle->speed,
        .kind = (FoxVehicleKind)vehicle->kind,
        .length = DrivingOf(vehicle)->length,
        .color = vehicle->color,
        .waitTicks = vehicle->waitTicks,
        .next = vehicle->connection,
    };
    return true;
}

/* Whether a remembered path is still the one the vehicle drove. */
static bool TrailLive(const FoxNetwork *network, uint32_t path, uint32_t generation)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    return record != NULL && record->generation == generation && record->count >= 2u;
}

/* The point `distance` along the current path of a vehicle; a negative
 * distance lies on the paths behind it, and beyond the oldest one it
 * continues straight back from its start. */
static FoxVec PointAlong(const FoxNetwork *network, const Vehicle *vehicle, int64_t distance)
{
    uint32_t path = vehicle->path;
    for (uint32_t index = 0u; distance < 0 && index < vehicle->trailCount; ++index)
    {
        if (!TrailLive(network, vehicle->trail[index], vehicle->trailGeneration[index]))
        {
            break;
        }
        distance += vehicle->trailLength[index];
        path = vehicle->trail[index];
    }
    FoxVec point;
    FoxVec direction;
    (void)FoxNetworkPathPoint(network, path, FoxMax(0, distance), &point, &direction);
    if (distance < 0)
    {
        point = FoxVecSub(point, FoxVecScale(direction, -distance, FOX_UNIT));
    }
    return point;
}

bool FoxTrafficVehiclePose(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t slot,
                           int64_t blend, FoxVec *outPosition, FoxVec *outForward)
{
    if (traffic == NULL || network == NULL || slot >= traffic->capacity ||
        !traffic->vehicles[slot].alive)
    {
        return false;
    }
    const Vehicle *vehicle = &traffic->vehicles[slot];
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    if (path == NULL || path->count < 2u)
    {
        return false;
    }
    /* The front between the last two ticks, counted along the current path. */
    int64_t previous = vehicle->s;
    if (vehicle->prevPath == vehicle->path && vehicle->prevGeneration == vehicle->pathGeneration)
    {
        previous = vehicle->prevS;
    }
    else
    {
        int64_t behind = 0;
        for (uint32_t index = 0u; index < vehicle->trailCount; ++index)
        {
            behind += vehicle->trailLength[index];
            if (vehicle->trail[index] == vehicle->prevPath &&
                vehicle->trailGeneration[index] == vehicle->prevGeneration)
            {
                previous = vehicle->prevS - behind;
                break;
            }
        }
    }
    const int64_t front =
        previous + (vehicle->s - previous) * FoxClamp(blend, 0, FOX_UNIT) / FOX_UNIT;
    /* The axles sit a sixth of the body in from either end. */
    const int64_t length = DrivingOf(vehicle)->length;
    const FoxVec ahead = PointAlong(network, vehicle, front - length / 6);
    const FoxVec rear = PointAlong(network, vehicle, front - length * 5 / 6);
    if (outPosition != NULL)
    {
        *outPosition = FoxVecLerp(ahead, rear, FOX_UNIT / 2);
    }
    if (outForward != NULL)
    {
        FoxVec forward = FoxVecDirection(FoxVecSub(ahead, rear));
        if (FoxVecIsZero(forward))
        {
            (void)FoxNetworkPathPoint(network, vehicle->path, vehicle->s, NULL, &forward);
        }
        *outForward = forward;
    }
    return true;
}

FoxTrafficStats FoxTrafficGetStats(const FoxTraffic *traffic)
{
    FoxTrafficStats stats = {0};
    if (traffic == NULL)
    {
        return stats;
    }
    int64_t speeds = 0;
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        const Vehicle *vehicle = &traffic->vehicles[slot];
        if (vehicle->alive)
        {
            ++stats.vehicles;
            stats.standing += vehicle->speed <= STANDING_SPEED ? 1u : 0u;
            stats.trucks += vehicle->kind == FOX_VEHICLE_TRUCK ? 1u : 0u;
            stats.waitingLong += vehicle->waitTicks >= 30u * TICKS ? 1u : 0u;
            stats.maxSpeed = vehicle->speed > stats.maxSpeed ? vehicle->speed : stats.maxSpeed;
            speeds += vehicle->speed;
        }
    }
    stats.spawned = traffic->spawned;
    stats.arrived = traffic->arrived;
    stats.removed = traffic->removed;
    stats.stuck = traffic->stuck;
    stats.meanSpeed = stats.vehicles != 0u ? (int32_t)(speeds / stats.vehicles) : 0;
    return stats;
}

uint64_t FoxTrafficHash(const FoxTraffic *traffic)
{
    if (traffic == NULL)
    {
        return 0u;
    }
    uint64_t hash = FoxHashCombine(traffic->tick, traffic->spawned);
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        const Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive)
        {
            continue;
        }
        hash = FoxHashCombine(hash, slot);
        hash = FoxHashCombine(hash, vehicle->serial);
        hash = FoxHashCombine(hash, vehicle->path);
        hash = FoxHashCombine(hash, (uint64_t)vehicle->s);
        hash = FoxHashCombine(hash, (uint64_t)(uint32_t)vehicle->speed);
        hash = FoxHashCombine(hash, vehicle->kind);
    }
    return hash;
}
