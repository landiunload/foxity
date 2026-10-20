#include "sim/traffic.h"

#include "sim/network_internal.h"
#include "sim/random.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define NONE FOX_NETWORK_NONE
#define TICKS FOX_TRAFFIC_TICKS_PER_SECOND
#define ROUTE_CAPACITY 128u
/* The driver model: acceleration, comfortable and emergency braking, the gap
 * kept when standing and the time gap kept when moving. */
#define MAX_ACCELERATION (FOX_METRES(16) / 10)
#define COMFORT_BRAKE (FOX_METRES(23) / 10)
#define MAX_BRAKE FOX_METRES(9)
#define MIN_GAP FOX_METRES(2)
#define HEADWAY_MS 1300
/* How far ahead a car looks for leaders and slower paths. */
#define LOOKAHEAD FOX_METRES(150)
#define STANDING_SPEED (FOX_METRES(1) / 2)
#define MOVING_SPEED FOX_METRES(2)
/* A lane whose last car moves this fast is not filling up. */
#define FREE_FLOW_SPEED FOX_METRES(5)
/* A lane shorter than a car and its gaps cannot be stopped on. */
#define SHORT_LANE (FOX_VEHICLE_LENGTH + 2 * MIN_GAP)
/* Cars this close to their line, beyond their braking distance and two
 * seconds of driving, take part in their intersection's rounds. */
#define DECISION_MARGIN FOX_METRES(15)
/* Head starts in an intersection's order: per rank of the road, and for
 * going straight. */
#define RANK_PRIORITY_TICKS (INT64_C(6) * TICKS)
#define STRAIGHT_PRIORITY_TICKS (INT64_C(3) * TICKS / 2)
/* A car standing this long is taken off the road, as city builders do: an
 * overloaded network can always tie itself into a knot, and this undoes it
 * while the spawner brings the car back elsewhere. */
#define STUCK_TICKS (60u * TICKS)
#define STATS_TICKS (UINT64_C(2) * TICKS)
#define REROUTE_TICKS (UINT64_C(30) * TICKS)
#define SPAWN_PER_TICK 2u
#define SPAWN_CLEARANCE FOX_METRES(14)
#define COLORS 8u

typedef struct Vehicle
{
    bool alive;
    bool permitted;
    bool clear;
    bool parking;
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
    FoxRandom random;
} Vehicle;

typedef struct Candidate Candidate;

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
    uint32_t *taken;
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

/* The lane a claimed connection leads into stops counting the car. */
static void Arrive(FoxNetwork *network, uint32_t claim)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, claim);
    FoxPath *lane = connection == NULL ? NULL : PathOf(network, connection->to);
    if (lane != NULL && lane->incoming > 0u)
    {
        --lane->incoming;
    }
}

static void ReleaseClaims(FoxNetwork *network, Vehicle *vehicle)
{
    if (vehicle->claim != NONE)
    {
        Arrive(network, vehicle->claim);
    }
    if (vehicle->through != NONE)
    {
        Arrive(network, vehicle->through);
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

static void PlanRoute(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle,
                      uint32_t from, bool keepDestination)
{
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
    if (vehicle->destination == NONE)
    {
        return;
    }
    vehicle->routeLength = network->router->findRoute(network->router->context, from,
                                                      vehicle->destination, vehicle->route,
                                                      ROUTE_CAPACITY);
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
        const uint32_t load = to->vehicleCount + to->incoming;
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
    vehicle->arrival = 0u;
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    if (lane == NULL || lane->kind != PATH_LANE || lane->outgoingCount == 0u)
    {
        return;
    }
    const uint32_t endNode = EndNode(network, lane);
    if (network->router != NULL && !vehicle->parking && vehicle->routeLength != 0u &&
        (vehicle->routeRevision != network->revision ||
         traffic->tick - vehicle->routeTick > REROUTE_TICKS))
    {
        PlanRoute(traffic, network, vehicle, endNode, true);
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
            PlanRoute(traffic, network, vehicle, endNode, true);
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
            PlanRoute(traffic, network, vehicle, EndNode(network, lane), false);
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
        vehicle->color = (uint8_t)FoxRandomBelow(&vehicle->random, COLORS);
        vehicle->desire = (int32_t)FoxRandomRange(&vehicle->random, FOX_UNIT * 9 / 10,
                                                   FOX_UNIT * 11 / 10);
        Link(traffic, network, slot, laneIndex, s);
        vehicle->prevPath = laneIndex;
        vehicle->prevGeneration = vehicle->pathGeneration;
        vehicle->prevS = s;
        ++traffic->count;
        ++traffic->spawned;
        PlanRoute(traffic, network, vehicle, EndNode(network, lane), false);
        ChooseConnection(traffic, network, vehicle);
    }
}

/* ---- Intersections --------------------------------------------------------------- */

/* Whether a lane has room for one more car beyond every car already on its
 * way there, or moves freely enough not to need it. */
static bool RoomOn(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t laneIndex)
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
    const int64_t room = (tail != NULL ? tail->s - FOX_VEHICLE_LENGTH : lane->length) -
                         (int64_t)lane->incoming * (FOX_VEHICLE_LENGTH + MIN_GAP);
    return room >= FOX_VEHICLE_LENGTH + MIN_GAP;
}

static void Claim(FoxNetwork *network, uint32_t connectionIndex)
{
    FoxPath *connection = PathOf(network, connectionIndex);
    ++connection->claims;
    ++PathOf(network, connection->to)->incoming;
}

/* Whether a connection crosses a claimed one, or one given away or reserved
 * earlier in this round of its intersection. */
static bool Blocked(const FoxNetwork *network, uint32_t connectionIndex, const uint32_t *taken,
                    uint32_t takenCount)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxPath *other = FoxNetworkPathRecord(network, connection->conflicts[index].path);
        if (other == NULL)
        {
            continue;
        }
        if (other->claims > 0u)
        {
            return true;
        }
        for (uint32_t slot = 0u; slot < takenCount; ++slot)
        {
            if (taken[slot] == connection->conflicts[index].path)
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

static int64_t BrakingDistance(int64_t speed)
{
    return speed * speed / (2 * COMFORT_BRAKE);
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
        if (!vehicle->alive || vehicle->permitted || vehicle->connection == NONE)
        {
            continue;
        }
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        if (lane->kind != PATH_LANE || lane->front != slot)
        {
            continue;
        }
        const int64_t distance = lane->length - vehicle->s;
        const int64_t zone =
            BrakingDistance(vehicle->speed) + (int64_t)vehicle->speed * 2 + DECISION_MARGIN;
        if (distance > zone)
        {
            /* Far from the line: drive on, decide later. */
            vehicle->clear = true;
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
            uint32_t *taken = realloc(traffic->taken, (size_t)capacity * 2u * sizeof(*taken));
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
        const uint8_t rank = FoxRoadTypeGet((FoxRoadType)network->roads[lane->road].type)->rank;
        int64_t key = (int64_t)vehicle->arrival - (int64_t)rank * RANK_PRIORITY_TICKS;
        key -= connection->movement == MOVE_STRAIGHT ? STRAIGHT_PRIORITY_TICKS : 0;
        key += connection->movement == MOVE_U_TURN ? STRAIGHT_PRIORITY_TICKS : 0;
        traffic->candidates[count++] = (Candidate){
            .node = connection->node,
            .key = key,
            .serial = vehicle->serial,
            .slot = slot,
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
        uint32_t through = NONE;
        bool clear = false;
        if (Blocked(network, vehicle->connection, traffic->taken, takenCount))
        {
            traffic->taken[takenCount++] = vehicle->connection;
        }
        else if (beyond->length < SHORT_LANE &&
                 PlanThrough(network, vehicle, connection->to) != NONE)
        {
            through = PlanThrough(network, vehicle, connection->to);
            /* Beyond a lane too short to stop on the next intersection must
             * be free as well. */
            if (beyond->back == NONE && beyond->incoming == 0u)
            {
                if (Blocked(network, through, NULL, 0u))
                {
                    traffic->taken[takenCount++] = vehicle->connection;
                }
                else
                {
                    clear = RoomOn(traffic, network, FoxNetworkPathRecord(network, through)->to);
                }
            }
        }
        else
        {
            through = NONE;
            clear = RoomOn(traffic, network, connection->to);
        }
        vehicle->clear = clear;
        if (!clear)
        {
            continue;
        }
        traffic->taken[takenCount++] = vehicle->connection;
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        if (lane->length - vehicle->s <= BrakingDistance(vehicle->speed) + FOX_METRES(4))
        {
            Claim(network, vehicle->connection);
            vehicle->claim = vehicle->connection;
            vehicle->permitted = true;
            vehicle->arrival = 0u;
            if (through != NONE)
            {
                Claim(network, through);
                vehicle->through = through;
            }
        }
    }
}
/* ---- Driving ------------------------------------------------------------------- */

static int32_t ScaledLimit(const Vehicle *vehicle, int32_t limit)
{
    return (int32_t)((int64_t)limit * vehicle->desire / FOX_UNIT);
}

/* The intelligent driver model in integers: free-road acceleration towards
 * the desired speed minus the interaction with the leader. */
static int32_t DriverModel(int64_t speed, int64_t desired, int64_t gap, int64_t leaderSpeed,
                           bool hasLeader)
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
        static const int64_t rootAB = 1918;
        const int64_t closing = speed * (speed - leaderSpeed) / (2 * rootAB);
        const int64_t wanted = MIN_GAP + FoxMax(0, speed * HEADWAY_MS / 1000 + closing);
        const int64_t ratio = FoxMin(wanted * FOX_UNIT / gap, 16 * FOX_UNIT);
        interaction = ratio * ratio / FOX_UNIT;
    }
    const int64_t acceleration =
        MAX_ACCELERATION * (FOX_UNIT - freeTerm - interaction) / FOX_UNIT;
    return (int32_t)FoxClamp(acceleration, -MAX_BRAKE, MAX_ACCELERATION);
}

static void Accelerate(const FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    int64_t desired = ScaledLimit(vehicle, path->speedLimit);
    int64_t gap = 0;
    int64_t leaderSpeed = 0;
    bool hasLeader = false;
    if (vehicle->ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[vehicle->ahead];
        gap = leader->s - FOX_VEHICLE_LENGTH - vehicle->s;
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
        /* Slow down in time for a slower path ahead. */
        const int64_t limit = ScaledLimit(vehicle, upcoming->speedLimit);
        const int64_t reachable = (int64_t)FoxIntegerSqrt(
            (uint64_t)(limit * limit + 2 * COMFORT_BRAKE * FoxMax(0, travelled)));
        desired = FoxMin(desired, reachable);
        if (!hasLeader && upcoming->back != NONE)
        {
            const Vehicle *leader = &traffic->vehicles[upcoming->back];
            gap = travelled + leader->s - FOX_VEHICLE_LENGTH;
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
    vehicle->acceleration = DriverModel(vehicle->speed, desired, gap, leaderSpeed, hasLeader);
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
        room = leader->s - FOX_VEHICLE_LENGTH - FOX_METRES(1) / 2 - vehicle->s;
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
            room = path->length - vehicle->s + leader->s - FOX_VEHICLE_LENGTH - FOX_METRES(1) / 2;
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
            Unlink(traffic, network, slot);
            Link(traffic, network, slot, connection, over);
        }
        else
        {
            const uint32_t to = path->to;
            Unlink(traffic, network, slot);
            Link(traffic, network, slot, to, over);
            /* The claim now covers the connection being left behind. */
            Release(network, &vehicle->clearing);
            if (vehicle->claim != NONE)
            {
                Arrive(network, vehicle->claim);
            }
            vehicle->clearing = vehicle->claim;
            vehicle->claim = NONE;
            vehicle->permitted = false;
            EnterLane(traffic, network, vehicle);
        }
    }
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    /* The connection behind is free once the car's rear has left it. */
    if (vehicle->clearing != NONE && path->kind == PATH_LANE && vehicle->s >= FOX_VEHICLE_LENGTH)
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
        const int32_t limit = FoxRoadTypeGet((FoxRoadType)road->type)->speedLimit;
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
        if ((ahead != NONE && traffic->vehicles[ahead].s - s < FOX_VEHICLE_LENGTH) ||
            (behind != NONE && s - traffic->vehicles[behind].s < FOX_VEHICLE_LENGTH))
        {
            Kill(traffic, network, slot, false);
            continue;
        }
        Link(traffic, network, slot, moved, s);
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
        if (vehicle->claim != NONE && FoxNetworkPathLive(network, vehicle->claim))
        {
            ++network->paths[vehicle->claim].claims;
            ++network->paths[network->paths[vehicle->claim].to].incoming;
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
            ++network->paths[network->paths[vehicle->through].to].incoming;
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
        .color = vehicle->color,
        .waitTicks = vehicle->waitTicks,
    };
    return true;
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
    FoxVec current;
    FoxVec forward;
    if (!FoxNetworkPathPoint(network, vehicle->path, vehicle->s, &current, &forward))
    {
        return false;
    }
    FoxVec previous = current;
    const FoxPath *before = FoxNetworkPathRecord(network, vehicle->prevPath);
    if (before != NULL && before->generation == vehicle->prevGeneration)
    {
        (void)FoxNetworkPathPoint(network, vehicle->prevPath, vehicle->prevS, &previous, NULL);
    }
    if (outPosition != NULL)
    {
        *outPosition = FoxVecLerp(previous, current, FoxClamp(blend, 0, FOX_UNIT));
    }
    if (outForward != NULL)
    {
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
    }
    return hash;
}
