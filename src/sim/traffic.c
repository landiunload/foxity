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
/* Head starts in an intersection's order: per step of right of way the sign
 * gives, and for going straight. Whoever has waited longer than the head
 * start of the road given way to is let in. */
#define SIGN_PRIORITY_TICKS (INT64_C(8) * TICKS)
#define STRAIGHT_PRIORITY_TICKS (INT64_C(3) * TICKS / 2)
/* A car may cross the way another one holds when it is through this long
 * before the other one gets there. */
#define GAP_TICKS (INT64_C(2) * TICKS)
/* A car's rear this far past where two ways meet leaves the other one free. */
#define PASSED_MARGIN FOX_METRES(3)
/* A car waiting this close to its line when its green ends may still clear
 * the intersection. */
#define CLEARING_REACH FOX_METRES(3)
/* At a stop sign a car stands this close to the line before it may go. */
#define STOP_LINE_REACH FOX_METRES(5)
/* Out of a hundred new vehicles, this many are trucks. */
#define TRUCK_PERCENT 15u
/* How many paths behind a vehicle are remembered for drawing its rear. */
#define TRAIL 3u
/* A car standing this long is taken off the road, as city builders do: an
 * overloaded network can always tie itself into a knot, and this undoes it
 * while the spawner brings the car back elsewhere. */
#define STUCK_TICKS (60u * TICKS)
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
    [FOX_VEHICLE_TRUCK] = {FOX_TRUCK_LENGTH, FOX_METRES(9) / 10, FOX_METRES(8) / 5, 1800,
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

/* A way given away or held in an intersection's round, and in how many ticks
 * its car gets there. */
typedef struct TakenWay
{
    uint32_t connection;
    int64_t eta;
} TakenWay;

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
static void FindPlace(const FoxTraffic *traffic, const FoxPath *path, int64_t s,
                      uint32_t *outAhead, uint32_t *outBehind)
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
        const FoxPath *to = connection == NULL ? NULL : FoxNetworkPathRecord(network, connection->to);
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
        const uint32_t direction = road->laneCount[0] == 0u   ? 1u
                                   : road->laneCount[1] == 0u ? 0u
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
            .destination = NONE,
            .random = FoxRandomStream(traffic->seed, serial),
        };
        vehicle->kind = FoxRandomBelow(&vehicle->random, 100u) < TRUCK_PERCENT
                            ? (uint8_t)FOX_VEHICLE_TRUCK
                            : (uint8_t)FOX_VEHICLE_CAR;
        vehicle->color = (uint8_t)FoxRandomBelow(&vehicle->random, COLORS);
        vehicle->desire = (int32_t)FoxRandomRange(&vehicle->random, FOX_UNIT * 95 / 100,
                                                   FOX_UNIT * 108 / 100);
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
    const int64_t room =
        (tail != NULL ? tail->s - DrivingOf(tail)->length : lane->length) - coming;
    return room >= length + MIN_GAP;
}

static void Claim(FoxNetwork *network, uint32_t connectionIndex, const Vehicle *vehicle)
{
    FoxPath *connection = PathOf(network, connectionIndex);
    ++connection->claims;
    PathOf(network, connection->to)->incoming += (uint32_t)DrivingOf(vehicle)->length;
}

static int64_t TravelTicks(const Vehicle *vehicle, int64_t distance);

/* Whether the cars holding a claimed connection stand in the way of one
 * meeting it `at` along it: those on it and those that have left it until
 * their rear is past the meeting point, and the one about to enter unless the
 * other car, through `through` ticks from now, is gone before it gets
 * there. */
static bool ClaimInWay(const FoxTraffic *traffic, const FoxNetwork *network,
                       uint32_t connectionIndex, const FoxPath *connection, int64_t at,
                       int64_t through)
{
    uint32_t holders = connection->vehicleCount;
    if (connection->back != NONE)
    {
        const Vehicle *last = &traffic->vehicles[connection->back];
        if (last->s - DrivingOf(last)->length <= at + PASSED_MARGIN)
        {
            return true;
        }
    }
    /* Those that have left are the last ones on the lane beyond; their rear
     * lies as far along the connection as they are short of their length. */
    const FoxPath *beyond = FoxNetworkPathRecord(network, connection->to);
    for (uint32_t slot = beyond->back;
         slot != NONE && traffic->vehicles[slot].clearing == connectionIndex;
         slot = traffic->vehicles[slot].ahead)
    {
        const Vehicle *leaving = &traffic->vehicles[slot];
        if (connection->length + leaving->s - DrivingOf(leaving)->length <= at + PASSED_MARGIN)
        {
            return true;
        }
        ++holders;
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, connection->from);
    if (lane != NULL && lane->front != NONE)
    {
        const Vehicle *coming = &traffic->vehicles[lane->front];
        if (coming->claim == connectionIndex)
        {
            if (through == 0 || TravelTicks(coming, lane->length - coming->s + at) <= through)
            {
                return true;
            }
            ++holders;
        }
    }
    /* The rest come through a lane too short to stop on and are on their
     * way. */
    return connection->claims > holders;
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

/* Ticks until the rear of a car `distance` before its line is past the point
 * `meet` along its connection: at full acceleration, but no faster than the
 * way allows. */
static int64_t PassTicks(const Vehicle *vehicle, const FoxPath *connection, int64_t distance,
                         int64_t meet)
{
    const int64_t beyond = meet + DrivingOf(vehicle)->length + PASSED_MARGIN;
    return FoxMax(TravelTicks(vehicle, distance + beyond),
                  TravelTicks(vehicle, distance) +
                      beyond * TICKS / FoxMax(FOX_METRES(1), connection->speedLimit));
}

/* Whether a connection crosses a claimed one, or one given away or reserved
 * earlier in this round of its intersection. A car `distance` before its
 * line may still go when its rear is past where the two meet a gap ahead of
 * the other car getting there; without a car every claim blocks. */
static bool Blocked(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t connectionIndex,
                    const TakenWay *taken, uint32_t takenCount, const Vehicle *vehicle,
                    int64_t distance)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const uint32_t otherIndex = connection->conflicts[index].path;
        const FoxPath *other = FoxNetworkPathRecord(network, otherIndex);
        if (other == NULL)
        {
            continue;
        }
        const int64_t through =
            vehicle == NULL ? 0
                            : PassTicks(vehicle, connection, distance,
                                        MeetingOn(network, connectionIndex, other)) +
                                  GAP_TICKS;
        if (other->claims > 0u &&
            ClaimInWay(traffic, network, otherIndex, other, connection->conflicts[index].otherAt,
                       through))
        {
            return true;
        }
        for (uint32_t slot = 0u; slot < takenCount; ++slot)
        {
            if (taken[slot].connection == otherIndex && taken[slot].eta <= through)
            {
                return true;
            }
        }
    }
    return false;
}

/* A car taking part in its intersection's round. */
struct Candidate
{
    uint32_t node;
    int64_t key;
    uint64_t serial;
    uint32_t slot;
    /* Ticks until the car reaches its line, and how far it is from it. */
    int64_t eta;
    int64_t distance;
};

static int CompareCandidates(const void *left, const void *right)
{
    const Candidate *a = (const Candidate *)left;
    const Candidate *b = (const Candidate *)right;
    if (a->node != b->node)
    {
        return a->node < b->node ? -1 : 1;
    }
    if (a->key != b->key)
    {
        return a->key < b->key ? -1 : 1;
    }
    return (a->serial > b->serial) - (a->serial < b->serial);
}

static int64_t BrakingDistance(const Vehicle *vehicle)
{
    const int64_t speed = vehicle->speed;
    return speed * speed / (2 * DrivingOf(vehicle)->brake);
}

/* Ticks a vehicle needs for a distance at full acceleration from its speed:
 * an estimate that errs early. */
static int64_t TravelTicks(const Vehicle *vehicle, int64_t distance)
{
    const int64_t speed = vehicle->speed;
    const int64_t acceleration = DrivingOf(vehicle)->acceleration;
    const int64_t root = (int64_t)FoxIntegerSqrt(
        (uint64_t)(speed * speed + 2 * acceleration * FoxMax(0, distance)));
    return (int64_t)TICKS * (root - speed) / acceleration;
}

/* Whether the light or sign at the end of a lane holds a car back for now. */
static bool HeldAtLine(const FoxNetwork *network, Vehicle *vehicle, const FoxPath *lane,
                       int64_t distance)
{
    const uint32_t end = lane->direction == 0u ? 1u : 0u;
    const FoxLight light = FoxSignalsLight(network, lane->road, end);
    if (light == FOX_LIGHT_RED)
    {
        /* Who waited at the line through the green to turn across the
         * oncoming traffic finishes the turn while the junction clears. */
        return vehicle->arrival == 0u || distance > CLEARING_REACH ||
               !FoxSignalsClearing(network, lane->road, end);
    }
    if (light == FOX_LIGHT_YELLOW)
    {
        /* Whoever can still stop comfortably does. */
        return distance > BrakingDistance(vehicle) + FOX_METRES(2);
    }
    if (light == FOX_LIGHT_NONE && network->roads[lane->road].sign[end] == FOX_SIGN_STOP &&
        !vehicle->stopped)
    {
        vehicle->stopped = vehicle->speed <= STANDING_SPEED && distance <= STOP_LINE_REACH;
        return !vehicle->stopped;
    }
    return false;
}

/* Right of way from the sign at the end of a lane: 2 priority, 1 none, 0
 * give way or stop. Under lights every approach is equal. */
static int64_t SignClass(const FoxNetwork *network, const FoxPath *lane)
{
    const uint8_t sign = network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u];
    return sign == FOX_SIGN_PRIORITY ? 2 : (sign == FOX_SIGN_NONE ? 1 : 0);
}

/* A car that holds its way but stands before its line with no room beyond
 * gives the way back: holding it would only block the cars crossing, and it
 * takes its turn again once there is room. */
static bool GiveBack(const FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle)
{
    if (vehicle->speed > STANDING_SPEED || vehicle->through != NONE ||
        vehicle->claim != vehicle->connection)
    {
        return false;
    }
    const uint32_t beyond = FoxNetworkPathRecord(network, vehicle->claim)->to;
    Arrive(network, vehicle->claim, vehicle);
    if (RoomOn(traffic, network, beyond, DrivingOf(vehicle)->length))
    {
        /* There is room once its own share is not counted: it keeps the way. */
        PathOf(network, beyond)->incoming += (uint32_t)DrivingOf(vehicle)->length;
        return false;
    }
    Release(network, &vehicle->claim);
    vehicle->permitted = false;
    vehicle->clear = false;
    return true;
}

/* Each intersection hands out its way in, one round per tick. The cars at
 * the front of its lanes that are close enough take part, in the order they
 * came, a bigger road and going straight counting as having come earlier.
 * Each in turn may go when nothing claimed or given away before it conflicts
 * and the lane beyond has room. A car held up by a conflict keeps its turn:
 * the cars after it may not take a way that crosses its own, so no car
 * waits forever and no ring of cars can wait for each other. A car claims
 * its way only once it could no longer stop comfortably; until then a car
 * that came earlier can still take its turn back. */
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
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        if (lane->kind != PATH_LANE || lane->front != slot ||
            (vehicle->permitted && !GiveBack(traffic, network, vehicle)))
        {
            continue;
        }
        const int64_t distance = lane->length - vehicle->s;
        const int64_t zone =
            BrakingDistance(vehicle) + (int64_t)vehicle->speed * 2 + DECISION_MARGIN;
        if (distance > zone)
        {
            /* Far from the line: drive on, decide later. */
            vehicle->clear = true;
            vehicle->arrival = 0u;
            continue;
        }
        if (HeldAtLine(network, vehicle, lane, distance))
        {
            vehicle->clear = false;
            vehicle->arrival = 0u;
            continue;
        }
        if (vehicle->arrival == 0u)
        {
            vehicle->arrival = traffic->tick;
        }
        if (count == traffic->candidateCapacity)
        {
            const uint32_t capacity = count < 64u ? 64u : count * 2u;
            Candidate *grown = realloc(traffic->candidates, (size_t)capacity * sizeof(*grown));
            TakenWay *taken = realloc(traffic->taken, (size_t)capacity * 2u * sizeof(*taken));
            if (grown != NULL)
            {
                traffic->candidates = grown;
            }
            if (taken != NULL)
            {
                traffic->taken = taken;
            }
            if (grown == NULL || taken == NULL)
            {
                /* Without memory the car simply waits at its line. */
                vehicle->clear = false;
                continue;
            }
            traffic->candidateCapacity = capacity;
        }
        const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->connection);
        int64_t key = (int64_t)vehicle->arrival - SignClass(network, lane) * SIGN_PRIORITY_TICKS;
        key -= connection->movement == MOVE_STRAIGHT ? STRAIGHT_PRIORITY_TICKS : 0;
        key += connection->movement == MOVE_FAR ? STRAIGHT_PRIORITY_TICKS / 2 : 0;
        key += connection->movement == MOVE_U_TURN ? STRAIGHT_PRIORITY_TICKS : 0;
        traffic->candidates[count++] = (Candidate){
            .node = connection->node,
            .key = key,
            .serial = vehicle->serial,
            .slot = slot,
            .eta = TravelTicks(vehicle, distance),
            .distance = distance,
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
        const FoxPath *beyond = FoxNetworkPathRecord(network, connection->to);
        const int64_t length = DrivingOf(vehicle)->length;
        const TakenWay way = {.connection = vehicle->connection, .eta = candidate->eta};
        uint32_t through = NONE;
        bool clear = false;
        if (Blocked(traffic, network, vehicle->connection, traffic->taken, takenCount, vehicle,
                    candidate->distance))
        {
            traffic->taken[takenCount++] = way;
        }
        else if (beyond->length < length + 2 * MIN_GAP &&
                 PlanThrough(network, vehicle, connection->to) != NONE)
        {
            through = PlanThrough(network, vehicle, connection->to);
            /* Beyond a lane too short to stop on the next intersection must
             * be free as well. */
            if (beyond->back == NONE && beyond->incoming == 0u)
            {
                if (Blocked(traffic, network, through, NULL, 0u, NULL, 0))
                {
                    traffic->taken[takenCount++] = way;
                }
                else
                {
                    clear = RoomOn(traffic, network, FoxNetworkPathRecord(network, through)->to,
                                   length);
                }
            }
        }
        else
        {
            through = NONE;
            clear = RoomOn(traffic, network, connection->to, length);
        }
        vehicle->clear = clear;
        if (!clear)
        {
            continue;
        }
        traffic->taken[takenCount++] = way;
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        if (lane->length - vehicle->s <= BrakingDistance(vehicle) + FOX_METRES(4))
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
    if (path->kind == PATH_LANE)
    {
        if (vehicle->connection == NONE || (!vehicle->permitted && !vehicle->clear))
        {
            /* The stop line, or the end of the road. */
            if (!hasLeader)
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
        travelled += upcoming->length;
        next = upcoming->kind == PATH_CONNECTION ? upcoming->to : NONE;
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
                FoxClamp(((int64_t)road->averageSpeed[direction] * 3 + sample) / 4, FOX_METRES(1) / 2,
                         limit);
            road->averageSpeed[direction] = (int32_t)average;
            road->speedSum[direction] = 0;
            road->speedSamples[direction] = 0u;
            const uint32_t cost = (uint32_t)FoxClamp(road->length * 1000 / average, 1, UINT32_MAX - 1u);
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
    return lane->lane < record->laneCount[lane->direction] ? record->lanes[lane->direction][lane->lane]
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
            *outS = FoxClamp(s * candidate->length / FoxMax(1, old->length), 0, candidate->length - 1);
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
