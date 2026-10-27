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
#define FIRM_BRAKE (FOX_METRES(7) / 2)
#define MIN_GAP FOX_METRES(2)
/* How far short of a line a driver means to come to rest. */
#define STOP_SHORT FOX_METRES(1)
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
 * safety margin after waiting the patience, as one passing first must be
 * through that before the other comes itself. Out of patience a driver also
 * goes ahead of those that can still stop for it comfortably with room to
 * spare, at first leaving them time to react, none after twice the
 * patience. */
#define REACTION_TICKS (INT64_C(3) * TICKS / 2)
/* How many lanes back drivers look for those they give way to. */
#define UPSTREAM_LANES 3u
#define UPSTREAM_PENDING 64u
#define SPARE_ROOM FOX_METRES(5)
#define GAP_TICKS (INT64_C(2) * TICKS)
#define PATIENCE_TICKS (UINT64_C(20) * TICKS)
#define SAFETY_TICKS (INT64_C(1) * TICKS)
#define MIN_GAP_TICKS SAFETY_TICKS
/* A car turning across the oncoming lanes drives in to wait when it gets at
 * least this far past its line. */
#define MIN_HOLD FOX_METRES(4)
/* A car this close behind another one in an intersection goes no faster. */
#define LEADER_REACH FOX_METRES(20)
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
 * red lights takes less, and a car creeping on in a queue this far at a
 * time does not stand. */
#define PROGRESS FOX_METRES(5)
/* A driver refused a turn this long while the light or sign lets it go, or
 * a quarter of it standing at the line this long, takes another way there
 * and finds a route on from it. */
#define GIVE_UP_TICKS (40u * TICKS)
#define GIVE_UP_STANDING (80u * TICKS)
#define STATS_TICKS (UINT64_C(2) * TICKS)
#define REROUTE_TICKS (UINT64_C(30) * TICKS)
/* A car a change of the network leaves without a place looks this many car
 * lengths either way for one, and for a lane this far from where it was. */
#define REJOIN_STEPS 9u
#define REJOIN_REACH FOX_METRES(60)
#define SPAWN_PER_TICK 2u
#define SPAWN_CLEARANCE FOX_METRES(14)
/* Turning round to and fro, a driver stands this long at each change of
 * direction; reversing it goes at two thirds of the pace. */
#define CUSP_PAUSE_TICKS (TICKS / 2u)
/* Changing lanes: drivers think about it this often and not again for a
 * while after a change, which takes them this long at speed, never less
 * than the shortest stretch. Lane lines turn solid this close to an
 * intersection, and nobody changes over them. A change must let a driver
 * accelerate this much harder, and nobody may have to brake harder than
 * this for it, unless the driver's route needs the other lane. On a
 * highway drivers keep to the curb side unless they gain this much more. */
#define CHANGE_EVERY_TICKS 10u
#define CHANGE_WAIT_TICKS (4u * TICKS)
#define CHANGE_TICKS (INT64_C(3) * TICKS)
#define CHANGE_MIN_SPAN FOX_METRES(8)
#define SOLID_NEAR FOX_METRES(15)
#define CHANGE_GAIN (FOX_METRES(1) / 4)
#define CHANGE_SAFE_BRAKE FOX_METRES(2)
#define CHANGE_URGENT_BRAKE FOX_METRES(3)
#define KEEP_CURB (FOX_METRES(1) / 5)
/* Overtaking through the oncoming lane: only one this many percent slower
 * than the driver would go, moving at least this fast and at most this far
 * ahead, with room this much bigger than needed to pull in ahead of it, and
 * only gaining this fast at the limit. The oncoming lane must
 * be clear this much further than the manoeuvre takes; pulling out and in
 * takes this stretch, and the overtaker pulls in this long ahead of the
 * other at its speed. */
#define PASS_NONE INT64_MIN
#define PASS_SLOWER 90
#define PASS_MIN_SPEED FOX_METRES(6)
#define PASS_STEADY (FOX_METRES(1) / 4)
#define PASS_REACH FOX_METRES(25)
#define PASS_ROOM FOX_METRES(20)
#define PASS_MIN_GAIN (FOX_METRES(3) / 2)
#define PASS_MARGIN FOX_METRES(60)
#define PASS_SPAN FOX_METRES(14)
#define PASS_RETURN_MS 800
#define CUSP_REACH (FOX_METRES(1) / 20)
#define COLORS 8u

/* How each kind of vehicle drives: its length, acceleration, comfortable
 * braking, time gap and top speed, and the share of the limit its drivers
 * aim for, in FOX_UNIT: nobody goes over it, and trucks keep well below. */
/* `slowest` and `fastest` bound the share of the limit a vehicle keeps to
 * where the road is free: a car the limit itself, a loaded truck a little
 * under it. */
typedef struct Driving
{
    int64_t length;
    int64_t acceleration;
    int64_t brake;
    int64_t headwayMs;
    int64_t topSpeed;
    int64_t slowest;
    int64_t fastest;
} Driving;

static const Driving drivings[FOX_VEHICLE_KIND_COUNT] = {
    [FOX_VEHICLE_CAR] = {FOX_CAR_LENGTH, FOX_METRES(5) / 2, FOX_METRES(5) / 2, 1200, FOX_METRES(60),
                         FOX_UNIT, FOX_UNIT},
    [FOX_VEHICLE_TRUCK] = {FOX_TRUCK_LENGTH, FOX_METRES(6) / 5, FOX_METRES(8) / 5, 1800,
                           FOX_KMH(80), FOX_UNIT * 80 / 100, FOX_UNIT * 90 / 100},
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
    /* On a dead end's turn: the leg driven, and the ticks stood at the
     * change of direction ending the last one. */
    uint8_t leg;
    uint8_t pause;
    /* Ticks before the driver thinks of changing lanes again. */
    uint16_t changeWait;
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
    /* How far the car has crept on since it last stood still or went at a
     * pace, and the ticks its turn was refused while its light or sign let
     * it go. */
    int64_t crept;
    uint32_t refused;
    /* The tick the car joined its intersection's rounds; zero outside them. */
    uint64_t arrival;
    /* Changing lanes: the lane left, NONE once the change is through, its
     * generation, and the stretch of the new lane the change takes. */
    uint32_t changeFrom;
    uint32_t changeGeneration;
    int64_t changeAt;
    int64_t changeSpan;
    /* Overtaking through the oncoming lane: where along its lane the car
     * pulled out and where it began to pull in again, PASS_NONE before, and
     * the serial of the one it overtakes. */
    int64_t passOut;
    int64_t passBack;
    uint64_t passed;
    /* Level with the one it overtakes but unable to get past it before the
     * oncoming traffic comes: falling back behind it, or, more than half
     * past it, having it hold back to let this one in. */
    bool fallingBack;
    bool lettingIn;
    /* Q16 share of the speed limit this driver aims for. */
    int32_t desire;
    uint32_t destination;
    uint32_t route[ROUTE_CAPACITY];
    uint32_t routeLength;
    uint32_t routeIndex;
    /* Where along the route the trip to the destination ends when the next
     * one is planned on beyond it already; zero otherwise. */
    uint32_t tripEnd;
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
    uint32_t manner;
    Vehicle *vehicles;
    uint32_t capacity;
    uint32_t count;
    uint64_t tick;
    uint64_t nextSerial;
    uint64_t spawned;
    uint64_t arrived;
    uint64_t removed;
    uint64_t laneChanges;
    uint64_t overtakes;
    FoxRandom random;
    Candidate *candidates;
    TakenWay *taken;
    uint32_t candidateCapacity;
    /* The cars changing lanes this tick, and those overtaking. */
    uint32_t *changers;
    uint32_t changerCount;
    uint32_t changerCapacity;
    uint32_t *passers;
    uint32_t passerCount;
    uint32_t passerCapacity;
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
    traffic->manner =
        desc->mannerPercent < FOX_TRAFFIC_MAX_MANNER ? desc->mannerPercent : FOX_TRAFFIC_MAX_MANNER;
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
        free(traffic->changers);
        free(traffic->passers);
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

/* The distance along the lane `to` level with `s` along the lane `from`
 * beside it. */
static int64_t Across(const FoxNetwork *network, uint32_t from, uint32_t to, int64_t s)
{
    const FoxPath *source = FoxNetworkPathRecord(network, from);
    const FoxPath *target = FoxNetworkPathRecord(network, to);
    return FoxClamp(FoxCentreToLane(target, FoxLaneToCentre(source, s)), 0, target->length);
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

/* Whether a way along a road, `edge`, ends in a dead end, through any bends
 * where only one road goes on: whoever takes it can only turn round, which
 * trucks, too long for it, never have to. */
static bool LeadsToDeadEnd(const FoxNetwork *network, uint32_t edge)
{
    for (uint32_t step = 0u; step < 64u; ++step)
    {
        const FoxRoad *road = &network->roads[edge / 2u];
        const uint32_t node = road->nodes[edge % 2u == 0u ? 1u : 0u];
        const FoxNode *record = &network->nodes[node];
        if (record->roadCount != 2u)
        {
            return record->roadCount == 1u;
        }
        const uint32_t next = record->roads[record->roads[0] == edge / 2u ? 1u : 0u];
        edge = next * 2u + (network->roads[next].nodes[0] == node ? 0u : 1u);
    }
    return false;
}

/* Whether a node is fit for a truck to head for: not one from which it
 * could only go on into a dead end. */
static bool TruckEnd(const FoxNetwork *network, uint32_t node)
{
    const FoxNode *record = &network->nodes[node];
    for (uint32_t index = 0u; index < record->roadCount; ++index)
    {
        const uint32_t road = record->roads[index];
        const uint32_t edge = road * 2u + (network->roads[road].nodes[0] == node ? 0u : 1u);
        if (network->roads[road].laneCount[edge % 2u] != 0u && !LeadsToDeadEnd(network, edge))
        {
            return record->roadCount >= 3u;
        }
    }
    return false;
}

static uint32_t RandomNode(FoxTraffic *traffic, const FoxNetwork *network, uint32_t avoid,
                           bool truck)
{
    if (network->nodeCapacity == 0u)
    {
        return NONE;
    }
    for (uint32_t attempt = 0u; attempt < 16u; ++attempt)
    {
        const uint32_t node = FoxRandomBelow(&traffic->random, network->nodeCapacity);
        if (network->nodes[node].alive && node != avoid && !(truck && !TruckEnd(network, node)))
        {
            return node;
        }
    }
    return NONE;
}

/* Whether a way leads onto a road that is closing: nobody takes it any more
 * but the cars that had leave to before it closed. */
static bool Closed(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    const FoxPath *to = connection == NULL ? NULL : FoxNetworkPathRecord(network, connection->to);
    return to != NULL && network->roads[to->road].closing;
}

static bool LaneLeadsTo(const FoxNetwork *network, const FoxPath *lane, uint32_t edge)
{
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        const FoxPath *to =
            connection == NULL ? NULL : FoxNetworkPathRecord(network, connection->to);
        if (to != NULL && EdgeOf(to) == edge && !Closed(network, lane->outgoing[index]))
        {
            return true;
        }
    }
    return false;
}

/* Whether a vehicle may take every road of a route: a truck none towards a
 * dead end, where the router may well have it turn round. */
static bool RouteFor(const FoxNetwork *network, const Vehicle *vehicle, const uint32_t *route,
                     uint32_t length)
{
    for (uint32_t index = 0u; index < length && vehicle->kind == FOX_VEHICLE_TRUCK; ++index)
    {
        if (LeadsToDeadEnd(network, route[index]))
        {
            return false;
        }
    }
    return true;
}

/* What the router reckons an edge costs. */
static uint64_t EdgeCost(const FoxNetwork *network, uint32_t edge)
{
    return network->roads[edge / 2u].routedCost[edge % 2u];
}

/* A route from the end of a lane to `destination` into `route`, at most
 * `capacity` roads, and its length; zero without one. The router knows
 * roads, not lanes, and would as soon begin by turning back down the road
 * the car is on, or have a truck turn round in a dead end; a route must
 * begin with a turn the lane really has, and suit the vehicle. When the
 * cheapest one does not, each way the lane leads is tried, with the
 * cheapest route on from there. */
static uint32_t RouteFrom(const FoxNetwork *network, const Vehicle *vehicle, const FoxPath *lane,
                          uint32_t destination, uint32_t *route, uint32_t capacity)
{
    const FoxRouter *router = network->router;
    uint32_t found =
        router->findRoute(router->context, EndNode(network, lane), destination, route, capacity);
    if (found != 0u && LaneLeadsTo(network, lane, route[0]) &&
        RouteFor(network, vehicle, route, found))
    {
        return found;
    }
    found = 0u;
    uint32_t onward[ROUTE_CAPACITY];
    uint64_t best = UINT64_MAX;
    for (uint32_t index = 0u; index < lane->outgoingCount && capacity > 1u; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        const FoxPath *next = FoxNetworkPathRecord(network, connection->to);
        const uint32_t edge = EdgeOf(next);
        bool tried = (vehicle->kind == FOX_VEHICLE_TRUCK && LeadsToDeadEnd(network, edge)) ||
                     Closed(network, lane->outgoing[index]);
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
            length = router->findRoute(router->context, beyond, destination, onward, capacity - 1u);
            if (length == 0u || !RouteFor(network, vehicle, onward, length))
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
            route[0] = edge;
            memcpy(route + 1, onward, (size_t)length * sizeof(*onward));
            found = length + 1u;
        }
    }
    return found;
}

/* A new route from the end of a lane, to the destination the car has or to
 * a new one. */
static void PlanRoute(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle,
                      uint32_t laneIndex, bool keepDestination)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const uint32_t from = EndNode(network, lane);
    vehicle->routeLength = 0u;
    vehicle->routeIndex = 0u;
    vehicle->tripEnd = 0u;
    vehicle->routeRevision = network->revision;
    vehicle->routeTick = traffic->tick;
    if (network->router == NULL)
    {
        return;
    }
    if (!keepDestination || vehicle->destination >= network->nodeCapacity ||
        !network->nodes[vehicle->destination].alive || vehicle->destination == from)
    {
        vehicle->destination =
            RandomNode(traffic, network, from, vehicle->kind == FOX_VEHICLE_TRUCK);
    }
    if (vehicle->destination != NONE)
    {
        vehicle->routeLength =
            RouteFrom(network, vehicle, lane, vehicle->destination, vehicle->route, ROUTE_CAPACITY);
    }
}

/* With one road of its trip left, a car picks where to go next and plans
 * the route on from the lane it takes there, so that it knows the turn at
 * its end in time to slow down for it. */
static void PlanBeyond(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->connection);
    if (network->router == NULL || vehicle->parking || vehicle->tripEnd != 0u ||
        vehicle->routeLength == 0u || vehicle->routeIndex + 1u != vehicle->routeLength ||
        vehicle->routeLength + 1u >= ROUTE_CAPACITY || connection == NULL)
    {
        return;
    }
    const FoxPath *last = FoxNetworkPathRecord(network, connection->to);
    if (EdgeOf(last) != vehicle->route[vehicle->routeIndex])
    {
        return;
    }
    const uint32_t from = EndNode(network, last);
    const uint32_t destination =
        RandomNode(traffic, network, from, vehicle->kind == FOX_VEHICLE_TRUCK);
    if (destination == NONE)
    {
        return;
    }
    const uint32_t length =
        RouteFrom(network, vehicle, last, destination, vehicle->route + vehicle->routeLength,
                  ROUTE_CAPACITY - vehicle->routeLength);
    if (length != 0u)
    {
        vehicle->tripEnd = vehicle->routeLength;
        vehicle->routeLength += length;
        vehicle->destination = destination;
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
        if (EdgeOf(to) != want || Closed(network, candidate))
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

/* Whether a way through an intersection runs into a lane that a way from a
 * lane of the same road further from the curb runs into as well: the lane
 * it comes from ends there, and drivers move over before. */
static bool LaneEnds(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    if (connection == NULL || connection->kind != PATH_CONNECTION)
    {
        return false;
    }
    const FoxPath *from = FoxNetworkPathRecord(network, connection->from);
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxPath *other = FoxNetworkPathRecord(network, connection->conflicts[index].path);
        const FoxPath *start = other != NULL ? FoxNetworkPathRecord(network, other->from) : NULL;
        if (start != NULL && other->to == connection->to && EdgeOf(start) == EdgeOf(from) &&
            start->lane > from->lane)
        {
            return true;
        }
    }
    return false;
}

/* Whether a driver without a destination would rather not take a
 * connection: turning round, towards a dead end, where it would only have
 * to turn round, or into a merge where its lane ends. */
static bool Shunned(const FoxNetwork *network, uint32_t connectionIndex)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, connectionIndex);
    return connection->movement == MOVE_U_TURN ||
           LeadsToDeadEnd(network, EdgeOf(FoxNetworkPathRecord(network, connection->to))) ||
           LaneEnds(network, connectionIndex);
}

/* A random turn, of those not shunned where there are any, never onto a
 * closing road; NONE where every way leads onto one. */
static uint32_t RandomConnection(const FoxNetwork *network, const FoxPath *lane, uint64_t seed)
{
    uint32_t turns = 0u;
    uint32_t open = 0u;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const bool closed = Closed(network, lane->outgoing[index]);
        open += closed ? 0u : 1u;
        turns += closed || Shunned(network, lane->outgoing[index]) ? 0u : 1u;
    }
    const uint32_t choices = turns != 0u ? turns : open;
    if (choices == 0u)
    {
        return NONE;
    }
    const uint32_t pick = (uint32_t)(FoxHash64(seed) % choices);
    uint32_t seen = 0u;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        if (Closed(network, lane->outgoing[index]) ||
            (turns != 0u && Shunned(network, lane->outgoing[index])))
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
        /* The pick the car will make there, the same every tick, so the
         * claim and the speed it slows down to match the turn taken. */
        return RandomConnection(network, lane, FoxHashCombine(vehicle->random.state, laneIndex));
    }
    const uint32_t index = vehicle->routeIndex + 1u;
    if (index >= vehicle->routeLength)
    {
        return NONE;
    }
    return RouteConnection(network, lane, vehicle->route[index],
                           index + 1u < vehicle->routeLength ? vehicle->route[index + 1u] : NONE);
}

/* Whether another lane beside a lane, in the same direction, leads to the
 * road `want`. */
static bool OtherLaneLeads(const FoxNetwork *network, const FoxPath *lane, uint32_t want,
                           uint32_t after)
{
    const FoxRoad *road = &network->roads[lane->road];
    for (uint32_t index = 0u; index < road->laneCount[lane->direction]; ++index)
    {
        const FoxPath *other = FoxNetworkPathRecord(network, road->lanes[lane->direction][index]);
        if (other != lane && RouteConnection(network, other, want, after) != NONE)
        {
            return true;
        }
    }
    return false;
}

/* The way a car will take at the end of a lane it has not reached yet from
 * whichever lane beside it leads on along its route, where it changes over
 * before; NONE when not known. */
static uint32_t TurnAhead(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex)
{
    const uint32_t index = vehicle->routeIndex + 1u;
    if (network->router == NULL || vehicle->parking || index >= vehicle->routeLength)
    {
        return NONE;
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const FoxRoad *road = &network->roads[lane->road];
    const uint32_t after = index + 1u < vehicle->routeLength ? vehicle->route[index + 1u] : NONE;
    for (uint32_t side = 0u; side < road->laneCount[lane->direction] && side < FOX_ROAD_MAX_LANES;
         ++side)
    {
        const uint32_t turn = RouteConnection(
            network, FoxNetworkPathRecord(network, road->lanes[lane->direction][side]),
            vehicle->route[index], after);
        if (turn != NONE)
        {
            return turn;
        }
    }
    return NONE;
}

/* What a car takes at the end of a lane it means to leave for another one
 * before it gets there: any turn but round, one not towards a dead end
 * first, straight on first of those. */
static uint32_t StandIn(const FoxNetwork *network, const FoxPath *lane)
{
    uint32_t found = lane->outgoing[0];
    int32_t best = -1;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, lane->outgoing[index]);
        const int32_t score =
            (LeadsToDeadEnd(network, EdgeOf(FoxNetworkPathRecord(network, connection->to))) ? 0
                                                                                            : 2) +
            (connection->movement == MOVE_STRAIGHT ? 1 : 0);
        if (connection->movement != MOVE_U_TURN && score > best &&
            !Closed(network, lane->outgoing[index]))
        {
            best = score;
            found = lane->outgoing[index];
        }
    }
    return found;
}

/* The connection a car on a lane takes at its end: towards the next road of
 * its route, into a lane that leads on to the road after it, the emptiest
 * such lane first. Where another lane beside it leads there and `mayChange`
 * allows, the car keeps its route and means to change lanes. Without a
 * usable route, a random turn. */
static void PickConnection(FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle,
                           bool mayChange)
{
    vehicle->connection = NONE;
    vehicle->permitted = false;
    vehicle->clear = false;
    vehicle->stopped = false;
    vehicle->holdAt = HOLD_NONE;
    vehicle->arrival = 0u;
    vehicle->refused = 0u;
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    if (lane == NULL || lane->kind != PATH_LANE || lane->outgoingCount == 0u)
    {
        return;
    }
    if (network->router != NULL && !vehicle->parking && vehicle->routeLength != 0u &&
        (vehicle->routeRevision != network->revision ||
         (traffic->tick - vehicle->routeTick > REROUTE_TICKS && vehicle->tripEnd == 0u)))
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
        if (attempt == 0u && mayChange && OtherLaneLeads(network, lane, want, after))
        {
            vehicle->connection = StandIn(network, lane);
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
    vehicle->tripEnd = 0u;
    vehicle->connection =
        RandomConnection(network, lane, FoxHashCombine(vehicle->random.state, vehicle->path));
    (void)FoxRandomNext(&vehicle->random);
}

/* The same; with the last road of its trip ahead, a car plans the next one
 * on. */
static void ChooseConnection(FoxTraffic *traffic, FoxNetwork *network, Vehicle *vehicle,
                             bool mayChange)
{
    PickConnection(traffic, network, vehicle, mayChange);
    PlanBeyond(traffic, network, vehicle);
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
        vehicle->tripEnd = 0u;
    }
    if (vehicle->tripEnd != 0u && vehicle->routeIndex >= vehicle->tripEnd)
    {
        /* At the end of a trip, with the next one planned on already. */
        vehicle->tripEnd = 0u;
        ++traffic->arrived;
        if (traffic->count > traffic->target)
        {
            vehicle->parking = true;
        }
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
    ChooseConnection(traffic, network, vehicle, true);
}

/* ---- Intersections --------------------------------------------------------------- */

/* Whether a lane has room for one more vehicle of `length` beyond every one
 * already on its way there, or moves freely enough not to need it. The cars
 * on it take the room they end up in once they have closed up behind those
 * ahead, and a car changing out of the lane still takes the room its body
 * is in. */
static bool RoomOn(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t laneIndex,
                   int64_t length)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    if (lane == NULL)
    {
        return false;
    }
    if (traffic->vehicles == NULL)
    {
        /* Nobody on the roads yet. */
        return true;
    }
    const Vehicle *tail = lane->back != NONE ? &traffic->vehicles[lane->back] : NULL;
    int64_t last = lane->length;
    for (uint32_t slot = lane->front; slot != NONE; slot = traffic->vehicles[slot].behind)
    {
        const Vehicle *vehicle = &traffic->vehicles[slot];
        last = FoxMax(vehicle->s, slot == lane->front ? vehicle->s : last - MIN_GAP) -
               DrivingOf(vehicle)->length;
    }
    bool moving = tail == NULL || tail->speed > FREE_FLOW_SPEED;
    for (uint32_t index = 0u; index < traffic->changerCount; ++index)
    {
        const Vehicle *changer = &traffic->vehicles[traffic->changers[index]];
        if (changer->changeFrom != laneIndex)
        {
            continue;
        }
        const int64_t rear =
            Across(network, changer->path, laneIndex, changer->s) - DrivingOf(changer)->length;
        if (rear < last)
        {
            last = rear;
            moving = changer->speed > FREE_FLOW_SPEED;
        }
    }
    if (moving && (tail != NULL || last < lane->length || lane->incoming == 0u))
    {
        return true;
    }
    /* Each vehicle on its way keeps a gap besides its length. */
    const int64_t coming = (int64_t)lane->incoming * (FOX_CAR_LENGTH + MIN_GAP) / FOX_CAR_LENGTH;
    return last - coming >= length + MIN_GAP;
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

/* How far a vehicle needs to stop braking firmly, as drivers do when the
 * light turns: a yellow is long enough to get through from anywhere closer
 * than this. */
static int64_t FirmBrakingDistance(const Vehicle *vehicle)
{
    const int64_t speed = vehicle->speed;
    return speed * speed / (2 * FIRM_BRAKE);
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
 * two meet, `meet` along the way, keeping `room` from it. It does not when
 * it is through a margin before, waits short of the meeting point, or comes
 * a gap after. */
static bool HolderInWay(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *holder,
                        const FoxPath *way, int64_t position, int64_t meet, int64_t room,
                        Window mine)
{
    const int64_t length = DrivingOf(holder)->length;
    if (position - length > meet + room)
    {
        return false;
    }
    const int64_t hold = HoldOf(network, holder);
    if (hold != HOLD_NONE && hold <= meet - room)
    {
        return false;
    }
    const int64_t arrive = EarliestTicks(holder, way->speedLimit, meet - room - position);
    const int64_t clear =
        hold != HOLD_NONE && hold - length < meet + room
            ? INT64_MAX
            : LatestTicks(holder,
                          FoxMin(way->speedLimit, LeaderPace(traffic, network, holder, way)),
                          meet + length + room - position);
    const bool before = clear != INT64_MAX && clear + SAFETY_TICKS <= mine.arrive;
    return !before && arrive < mine.clear + mine.gap;
}

/* Whether the cars holding a way stand in the way of one meeting it: those
 * on it, those that have left it until their rear is past, and the one about
 * to enter it. Claims of cars not found among them come through a lane too
 * short to stop on and always do. */
static bool HoldersInWay(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t wayIndex,
                         const FoxPath *way, int64_t meet, int64_t room, Window mine)
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
        if (HolderInWay(traffic, network, holder, way, holder->s, meet, room, mine))
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
        if (HolderInWay(traffic, network, leaving, way, way->length + leaving->s, meet, room, mine))
        {
            return true;
        }
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, way->from);
    if (lane->front != NONE && traffic->vehicles[lane->front].claim == wayIndex)
    {
        const Vehicle *coming = &traffic->vehicles[lane->front];
        ++holders;
        if (HolderInWay(traffic, network, coming, way, coming->s - lane->length, meet, room, mine))
        {
            return true;
        }
    }
    return way->claims > holders;
}

/* A way given away earlier in this round of its intersection, to whom, and
 * whether that car had been let through before and stands held by
 * something ahead of it: it is in nobody's way until it moves. */
struct TakenWay
{
    uint32_t connection;
    uint32_t slot;
    bool held;
};

static bool TakenInWay(const FoxTraffic *traffic, const FoxNetwork *network, const TakenWay *taken,
                       uint32_t takenCount, uint32_t wayIndex, const FoxPath *way, int64_t meet,
                       int64_t room, Window mine)
{
    for (uint32_t index = 0u; index < takenCount; ++index)
    {
        if (taken[index].connection != wayIndex || taken[index].held)
        {
            continue;
        }
        const Vehicle *holder = &traffic->vehicles[taken[index].slot];
        const FoxPath *path = FoxNetworkPathRecord(network, holder->path);
        const int64_t position = path->kind == PATH_LANE ? holder->s - path->length : holder->s;
        if (HolderInWay(traffic, network, holder, way, position, meet, room, mine))
        {
            return true;
        }
    }
    return false;
}

/* Whether a yellow at the end of a lane stops a car `distance` before it:
 * whoever can still stop braking firmly does, and so does whoever could
 * stop braking hard but would not get to the line half a second before the
 * light turns red, slowing as it must for its way. */
static bool YellowStops(const FoxNetwork *network, const Vehicle *vehicle, const FoxPath *lane,
                        int64_t distance)
{
    if (distance > FirmBrakingDistance(vehicle) + FOX_METRES(1))
    {
        return true;
    }
    const int64_t speed = vehicle->speed;
    if (distance <= speed * speed / (4 * FIRM_BRAKE))
    {
        return false;
    }
    const FoxPath *way = FoxNetworkPathRecord(network, vehicle->connection);
    const int64_t through = way != NULL ? FoxMin(speed, (int64_t)way->speedLimit) : speed;
    const int64_t ticks = 2 * distance * (int64_t)TICKS / FoxMax(1, speed + through);
    return ticks + (int64_t)TICKS / 2 >
           (int64_t)FoxSignalsYellowLeft(network, EndNode(network, lane));
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
        return YellowStops(network, vehicle, lane, distance);
    }
    return light == FOX_LIGHT_NONE &&
           network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u] == FOX_SIGN_STOP &&
           !vehicle->stopped;
}

/* Whether the light or sign at the end of a lane further ahead will stop a
 * car `distance` before it, taking the way `onward` there if known. */
static bool StopsAhead(const FoxNetwork *network, const Vehicle *vehicle, const FoxPath *lane,
                       uint32_t onward, int64_t distance)
{
    const FoxLight light = onward != NONE ? LightFor(network, onward) : LightAt(network, lane);
    if (light == FOX_LIGHT_RED)
    {
        return true;
    }
    if (light == FOX_LIGHT_YELLOW)
    {
        return distance > FirmBrakingDistance(vehicle) + FOX_METRES(1);
    }
    return light == FOX_LIGHT_NONE &&
           network->roads[lane->road].sign[lane->direction == 0u ? 1u : 0u] == FOX_SIGN_STOP;
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

/* Whether a car stands at its line waiting for its turn there: it does not
 * come before it is given one, and then it starts from standing. */
static bool Waiting(const Vehicle *vehicle)
{
    return vehicle->speed <= STANDING_SPEED && vehicle->arrival != 0u && !vehicle->clear &&
           !vehicle->permitted;
}

/* Whether a car let through that has not claimed its way yet stands held by
 * something ahead of it, a car in the lane it is leaving say: it does not
 * come for now, and starting later it finds those that went meanwhile in
 * its way. */
static bool Held(const Vehicle *vehicle)
{
    return vehicle->speed <= STANDING_SPEED && vehicle->clear && !vehicle->permitted &&
           vehicle->acceleration <= 0;
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
 * of a lane a light or sign holds, that waits for its turn at the
 * intersection there or stands held. Where there are more ways in than can be
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
                       (WillStop(network, coming, previous, previous->length - coming->s) ||
                        Waiting(coming) || Held(coming));
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

/* Whether a car came to its intersection's round after another one, by the
 * tick, and by its serial in the same tick. */
static bool CameAfter(const Vehicle *vehicle, const Vehicle *other)
{
    return vehicle->arrival > other->arrival ||
           (vehicle->arrival == other->arrival && vehicle->serial > other->serial);
}

/* Whether a car that has not taken a way yet could get to where it meets
 * another one, `meet` along it, within `window` ticks: from the lane before
 * it, or from further back. Nobody gets past the first car of a lane while
 * a light or sign holds that one. One standing at its line waiting for its
 * own turn that came after the driver who asks, `asking`, lets that one go
 * first, and so does any for a driver out of patience: no ring of cars
 * waits for each other, and turns that cross under the same green take
 * turns rather than one side going on and on. */
static bool Approaching(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t wayIndex,
                        const FoxPath *way, int64_t meet, int64_t room, int64_t window,
                        int64_t reaction, const Vehicle *asking)
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
            (WillStop(network, coming, lane, distance) || Held(coming) ||
             (coming->speed <= STANDING_SPEED && !coming->clear && !coming->permitted &&
              (reaction >= 0 || (coming->arrival != 0u && CameAfter(coming, asking))))))
        {
            return false;
        }
        if (coming->connection == wayIndex && coming->claim != wayIndex &&
            Coming(coming, lane->speedLimit, distance, distance + meet - room, window, reaction))
        {
            return true;
        }
    }
    const FoxPath *beyond = FoxNetworkPathRecord(network, way->to);
    return Upstream(traffic, network, way->from, lane->length, lane->length + meet - room,
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

/* How much more of a gap a car merging into the lane of another way, its
 * rear through `clear` from now, leaves a car coming along that way: what
 * the one behind, a car, closes in slowing down comfortably from the limit
 * of its way to the speed the merging one reaches as in LatestTicks. */
static int64_t MergeLead(const Vehicle *vehicle, const FoxPath *connection, const FoxPath *other,
                         int64_t clear)
{
    const Driving *driving = DrivingOf(vehicle);
    const int64_t speed = vehicle->speed;
    const int64_t reached = (int64_t)FoxIntegerSqrt(
        (uint64_t)(speed * speed + 2 * driving->acceleration * 7 / 10 * FoxMax(0, clear)));
    const int64_t merging =
        FoxMax(speed, FoxMin((int64_t)connection->speedLimit * 9 / 10, reached));
    const int64_t closing = FoxMax(0, (int64_t)other->speedLimit - merging);
    return closing * (int64_t)TICKS / (2 * drivings[FOX_VEHICLE_CAR].brake);
}

/* How far along its connection a car `toLine` before its line (negative
 * once past it) may go: up to the room it keeps from the first way a car
 * holds or was given in this round and stands in its way there, or one a
 * car it gives way to is coming along; HOLD_NONE when it is free
 * throughout. `outMerge` tells whether that way runs into the same lane. */
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
        const int64_t room = conflict->room;
        const int64_t toMeet = toLine + meet;
        if (meet - room >= first || toMeet + length + room <= 0)
        {
            continue;
        }
        const bool merge = other->to == connection->to;
        /* No faster through than a car ahead on the way lets it. */
        const int64_t limit = connection->speedLimit;
        const int64_t pace = FoxMin(
            limit, FoxMin(limit, LeaderPace(traffic, network, vehicle, connection)) * 10 / 9);
        const Window mine = {
            .arrive = EarliestTicks(vehicle, connection->speedLimit, toMeet - room),
            .clear = LatestTicks(vehicle, pace, toMeet + length + room),
            /* The lead shrinks with the driver's patience as the gap does. */
            .gap = gap + (merge ? MergeLead(vehicle, connection, other, toMeet + length + room) *
                                      gap / GAP_TICKS
                                : 0),
        };
        if (HoldersInWay(traffic, network, conflict->path, other, conflict->otherAt, room, mine) ||
            TakenInWay(traffic, network, taken, takenCount, conflict->path, other,
                       conflict->otherAt, room, mine) ||
            (giveWay && conflict->yields &&
             Approaching(traffic, network, conflict->path, other, conflict->otherAt, room,
                         mine.clear + mine.gap, reaction, vehicle)))
        {
            first = meet - room;
            *outMerge = merge;
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
            .arrive = EarliestTicks(vehicle, connection->speedLimit, toMeet - conflict->room),
            .clear = LatestTicks(vehicle, connection->speedLimit, toMeet + length + conflict->room),
            .gap = GAP_TICKS,
        };
        if (HoldersInWay(traffic, network, conflict->path, other, conflict->otherAt, conflict->room,
                         mine))
        {
            return true;
        }
    }
    return false;
}

/* Whether a car taking the way `onward` at the end of a lane without a
 * light, or any way there when that is not known, may have to give way. */
static bool GivesWay(const FoxNetwork *network, const FoxPath *lane, uint32_t onward)
{
    if (LightAt(network, lane) != FOX_LIGHT_NONE)
    {
        return false;
    }
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *way = FoxNetworkPathRecord(network, lane->outgoing[index]);
        if (onward != NONE && lane->outgoing[index] != onward)
        {
            continue;
        }
        for (uint32_t conflict = 0u; conflict < way->conflictCount; ++conflict)
        {
            if (way->conflicts[conflict].yields)
            {
                return true;
            }
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

/* The rearmost car whose body is still on a way through an intersection:
 * on it, or gone on beyond but not yet clear of it; NONE without one.
 * `outRear` is how far along the way its rear is, negative while the rear
 * is still before the way. */
static uint32_t LastOnWay(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t wayIndex,
                          int64_t *outRear)
{
    const FoxPath *way = FoxNetworkPathRecord(network, wayIndex);
    if (way == NULL)
    {
        return NONE;
    }
    if (way->back != NONE)
    {
        const Vehicle *last = &traffic->vehicles[way->back];
        *outRear = last->s - DrivingOf(last)->length;
        return way->back;
    }
    const FoxPath *beyond = FoxNetworkPathRecord(network, way->to);
    if (beyond->back != NONE && traffic->vehicles[beyond->back].clearing == wayIndex)
    {
        const Vehicle *last = &traffic->vehicles[beyond->back];
        *outRear = way->length + last->s - DrivingOf(last)->length;
        return beyond->back;
    }
    return NONE;
}

/* How far along a way out of a lane the rear of a car on it must be before
 * one taking the way `other` from the same lane can no longer touch it: the
 * furthest of them all when that way is not known. */
static int64_t PartingOf(const FoxPath *way, uint32_t other)
{
    int64_t parting = 0;
    for (uint32_t index = 0u; index < way->siblingCount; ++index)
    {
        if (way->siblings[index].path == other)
        {
            return way->siblings[index].parting;
        }
        parting = FoxMax(parting, way->siblings[index].parting);
    }
    return parting;
}

/* The car that left a lane by another way than `except` and whose rear is
 * not yet past where the ways part, the rearmost one; NONE without one.
 * `outRear` is how far past the line its rear is, and `outLeft` how far it
 * still has to go to where they part. */
static uint32_t Departing(const FoxTraffic *traffic, const FoxNetwork *network, const FoxPath *lane,
                          uint32_t except, int64_t *outRear, int64_t *outLeft)
{
    uint32_t found = NONE;
    *outRear = 0;
    *outLeft = 0;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const uint32_t wayIndex = lane->outgoing[index];
        int64_t rear = 0;
        const uint32_t last =
            wayIndex == except ? NONE : LastOnWay(traffic, network, wayIndex, &rear);
        const int64_t left =
            last == NONE ? 0 : PartingOf(FoxNetworkPathRecord(network, wayIndex), except) - rear;
        if (left > 0 && (found == NONE || rear < *outRear))
        {
            *outRear = rear;
            *outLeft = left;
            found = last;
        }
    }
    return found;
}

/* Whether a car that left a lane by another way, `left` short of where the
 * ways part, will still be there when a car `toLine` before the line gets
 * there, a second early; the one coming follows it until then. */
static bool Lingers(const Vehicle *departing, int64_t left, const Vehicle *vehicle, int64_t toLine)
{
    const int64_t speed = FoxMax((int64_t)vehicle->speed, FOX_METRES(1));
    return left * speed > FoxMax(0, toLine - speed) * departing->speed;
}

/* How far along a connection a car must stop to keep clear of the ways it
 * gives way to that may be driven now: a car waiting inside stops short of
 * all of them, not between them. */
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
            first = FoxMin(first, MeetingOn(network, connectionIndex, other) - conflict->room);
        }
    }
    return first;
}

/* Whether a car may drive into the intersection to wait for a gap in the
 * oncoming traffic, its front at `hold` along its connection: when it turns
 * across that traffic on a green, nobody waits on its way already, and
 * standing there it is in the way of no way that may be driven while it
 * waits, which only a red light makes sure of. A truck is too long to stand
 * inside and waits at its line. */
static bool MayWaitInside(const FoxTraffic *traffic, const FoxNetwork *network,
                          const Vehicle *vehicle, const FoxPath *lane, const FoxPath *connection,
                          int64_t hold)
{
    if (connection->movement != MOVE_FAR || LightAt(network, lane) != FOX_LIGHT_GREEN ||
        vehicle->kind == FOX_VEHICLE_TRUCK)
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
    const int64_t rear = hold - DrivingOf(vehicle)->length;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxConflict *conflict = &connection->conflicts[index];
        const FoxPath *other = FoxNetworkPathRecord(network, conflict->path);
        if (other == NULL)
        {
            continue;
        }
        const int64_t meet = MeetingOn(network, vehicle->connection, other);
        if (meet + conflict->room > rear && meet - conflict->room < hold &&
            LightFor(network, connection->conflicts[index].path) != FOX_LIGHT_RED)
        {
            return false;
        }
    }
    return true;
}

/* Whether a dead end is busy: a car turns round there, or has been let to
 * in this round. One at a time, as the turn fills the road. */
static bool TurnBusy(const FoxNetwork *network, uint32_t nodeIndex, uint32_t takenCount)
{
    const FoxNode *node = &network->nodes[nodeIndex];
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        const FoxPath *connection = FoxNetworkPathRecord(network, node->connections[index]);
        if (connection->claims != 0u || connection->front != NONE)
        {
            return true;
        }
    }
    return takenCount != 0u;
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
    /* Let through before, it stands held by something ahead of it. */
    bool held;
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
 * when its light turns yellow or red and it can still stop, and
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
    const int64_t toLine = lane->length - vehicle->s;
    const bool stop =
        (light == FOX_LIGHT_RED &&
         toLine >= FirmBrakingDistance(vehicle) + (standing ? 0 : FOX_METRES(1))) ||
        (light == FOX_LIGHT_YELLOW && (standing || YellowStops(network, vehicle, lane, toLine)));
    if (!stop && !standing)
    {
        return false;
    }
    const uint32_t beyond = FoxNetworkPathRecord(network, vehicle->claim)->to;
    int64_t rear = 0;
    Arrive(network, vehicle->claim, vehicle);
    int64_t left = 0;
    if (!stop && Departing(traffic, network, lane, vehicle->connection, &rear, &left) == NONE &&
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

/* How readily a driver giving up a turn takes another way: towards the curb
 * first, crossing nobody's lanes, then straight on, then across. */
static int32_t DetourOrder(const FoxPath *connection)
{
    return connection->movement == MOVE_CURB ? 0 : (connection->movement == MOVE_STRAIGHT ? 1 : 2);
}

/* A driver refused its turn for long takes another way from its lane, with
 * room beyond, never turning round nor a truck towards a dead end; with a
 * destination, only one it finds a route on from. False when there is none. */
static bool Detour(FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle,
                   const FoxPath *lane)
{
    const bool routed = network->router != NULL && vehicle->routeLength != 0u && !vehicle->parking;
    uint32_t found = NONE;
    int32_t best = INT32_MAX;
    uint32_t onward[ROUTE_CAPACITY];
    uint32_t onwardLength = 0u;
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const uint32_t candidate = lane->outgoing[index];
        const FoxPath *connection = FoxNetworkPathRecord(network, candidate);
        const FoxPath *next = FoxNetworkPathRecord(network, connection->to);
        const int32_t order = DetourOrder(connection) + (Shunned(network, candidate) ? 3 : 0);
        if (candidate == vehicle->connection || connection->movement == MOVE_U_TURN ||
            Closed(network, candidate) || order >= best ||
            (vehicle->kind == FOX_VEHICLE_TRUCK && LeadsToDeadEnd(network, EdgeOf(next))) ||
            !RoomOn(traffic, network, connection->to, DrivingOf(vehicle)->length))
        {
            continue;
        }
        uint32_t length = 0u;
        const uint32_t beyond = EndNode(network, next);
        if (routed && beyond != vehicle->destination)
        {
            length = RouteFrom(network, vehicle, next, vehicle->destination, onward,
                               ROUTE_CAPACITY - 1u);
            if (length == 0u)
            {
                continue;
            }
        }
        found = candidate;
        best = order;
        onwardLength = length;
        if (routed)
        {
            vehicle->route[0] = EdgeOf(next);
            memcpy(vehicle->route + 1, onward, (size_t)length * sizeof(*onward));
        }
    }
    if (found == NONE)
    {
        return false;
    }
    if (routed)
    {
        vehicle->routeLength = onwardLength + 1u;
        vehicle->routeIndex = 0u;
        vehicle->tripEnd = 0u;
        vehicle->routeRevision = network->revision;
        vehicle->routeTick = traffic->tick;
    }
    vehicle->connection = found;
    vehicle->refused = 0u;
    vehicle->clear = false;
    vehicle->holdAt = HOLD_NONE;
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
                vehicle->refused = 0u;
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
            if ((vehicle->refused >= GIVE_UP_TICKS || (vehicle->refused >= GIVE_UP_TICKS / 4u &&
                                                       vehicle->waitTicks >= GIVE_UP_STANDING)) &&
                !Detour(traffic, network, vehicle, path))
            {
                /* Nowhere else to go: try again a while later. */
                vehicle->refused = 0u;
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
            .held = Held(vehicle),
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
                traffic->taken[takenCount++] = (TakenWay){.connection = vehicle->connection,
                                                          .slot = candidate->slot,
                                                          .held = candidate->held};
            }
            continue;
        }
        if (connection->movement == MOVE_U_TURN && TurnBusy(network, connection->node, takenCount))
        {
            vehicle->clear = false;
            continue;
        }
        int64_t hold = HOLD_NONE;
        int64_t rear = 0;
        if (blocked != HOLD_NONE)
        {
            hold = FoxMin(blocked, FirstGiveWay(network, vehicle->connection));
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
        int64_t left = 0;
        const uint32_t departing =
            Departing(traffic, network, lane, vehicle->connection, &rear, &left);
        if (departing != NONE &&
            Lingers(&traffic->vehicles[departing], left, vehicle, candidate->toLine))
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
        traffic->taken[takenCount++] = (TakenWay){
            .connection = vehicle->connection, .slot = candidate->slot, .held = candidate->held};
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
    for (uint32_t index = 0u; index < count; ++index)
    {
        Vehicle *vehicle = &traffic->vehicles[traffic->candidates[index].slot];
        if (FoxNetworkPathRecord(network, vehicle->path)->kind == PATH_LANE)
        {
            vehicle->refused = vehicle->clear ? 0u : vehicle->refused + 1u;
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
 * the desired speed minus the interaction with the leader. Over the speed it
 * aims for a driver eases off and brakes, comfortably at most once a tenth
 * over: braking harder is for what it has to stop or slow down for in
 * time, which the caller asks for itself. */
static int32_t DriverModel(const Driving *driving, int64_t speed, int64_t desired, int64_t gap,
                           int64_t leaderSpeed, bool hasLeader)
{
    int64_t freeTerm = FOX_UNIT + FOX_UNIT * driving->brake / driving->acceleration;
    if (desired > 0 && speed <= desired)
    {
        const int64_t ratio = speed * FOX_UNIT / desired;
        const int64_t square = ratio * ratio / FOX_UNIT;
        freeTerm = square * square / FOX_UNIT;
    }
    else if (desired > 0)
    {
        const int64_t tenth = FoxMax(1, desired / 10);
        freeTerm = FOX_UNIT + FOX_UNIT * driving->brake / driving->acceleration *
                                  FoxMin(speed - desired, tenth) / tenth;
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

/* How a driver brakes for where it has to stop `stop` ahead, a line or a
 * place to wait: as the driver model would for a car standing there, but
 * no harder than a quarter over the steady braking that stops it a little
 * short of the place, or halfway there once that close. Braking so, the
 * driver eases off as the car slows and comes to rest smoothly. */
static int32_t StopModel(const Driving *driving, int64_t speed, int64_t desired, int64_t stop)
{
    const int32_t model = DriverModel(driving, speed, desired, stop, 0, true);
    const int64_t room = FoxMax(stop - STOP_SHORT, stop / 2);
    if (room <= 0)
    {
        return model;
    }
    const int64_t steady = speed * speed / (2 * room);
    return (int32_t)FoxMax((int64_t)model, -FoxMin(steady * 5 / 4, MAX_BRAKE));
}

/* ---- Changing lanes ------------------------------------------------------------- */

/* The lane beside a lane, a lane further from the curb for `side` 1 and
 * nearer it for -1; NONE where there is none. */
static uint32_t Neighbour(const FoxNetwork *network, const FoxPath *lane, int32_t side)
{
    const FoxRoad *road = &network->roads[lane->road];
    const int32_t index = (int32_t)lane->lane + side;
    return index >= 0 && index < (int32_t)road->laneCount[lane->direction] &&
                   index < (int32_t)FOX_ROAD_MAX_LANES
               ? road->lanes[lane->direction][index]
               : NONE;
}

/* Whether a car changing lanes is still partly in the lane it left. */
static bool Changing(const FoxNetwork *network, const Vehicle *vehicle)
{
    const FoxPath *from = FoxNetworkPathRecord(network, vehicle->changeFrom);
    return from != NULL && from->generation == vehicle->changeGeneration;
}

/* The leader a car minds most of those it has, `gap` to its rear and its
 * speed: for how hard to brake, the one it would come up to first were
 * both to brake comfortably, a further one where that is slower enough;
 * for never running into one, the nearest. */
typedef struct Leader
{
    bool nearest;
    bool found;
    int64_t gap;
    int64_t speed;
} Leader;

/* How far a car could go behind a leader `gap` ahead at `speed`, both of
 * them braking comfortably, or just the gap when minding the nearest. */
static int64_t LeaderRoom(const Leader *leader, int64_t gap, int64_t speed)
{
    return leader->nearest ? gap : gap + speed * speed / (2 * drivings[FOX_VEHICLE_CAR].brake);
}

/* Takes a leader where it is the one to mind. */
static void Mind(Leader *leader, int64_t gap, int64_t speed)
{
    if (!leader->found ||
        LeaderRoom(leader, gap, speed) < LeaderRoom(leader, leader->gap, leader->speed))
    {
        leader->gap = gap;
        leader->speed = speed;
        leader->found = true;
    }
}

/* Leaders a car `s` along a lane has besides those on the lane's own list:
 * cars changing out of the lane, still partly in it, and for a car changing
 * lanes itself those ahead in the lane it is leaving, on its list or
 * changing out of it too. */
static void SideLeaders(const FoxTraffic *traffic, const FoxNetwork *network,
                        const Vehicle *vehicle, uint32_t laneIndex, int64_t s, Leader *lead)
{
    const bool changing =
        vehicle->path == laneIndex && vehicle->changeFrom != NONE && Changing(network, vehicle);
    const int64_t left = changing ? Across(network, laneIndex, vehicle->changeFrom, s) : 0;
    for (uint32_t index = 0u; index < traffic->changerCount; ++index)
    {
        const Vehicle *changer = &traffic->vehicles[traffic->changers[index]];
        if (changer == vehicle)
        {
            continue;
        }
        if (changer->changeFrom == laneIndex)
        {
            const int64_t at = Across(network, changer->path, laneIndex, changer->s);
            if (at > s)
            {
                Mind(lead, at - DrivingOf(changer)->length - s, changer->speed);
            }
        }
        else if (changing && changer->changeFrom == vehicle->changeFrom)
        {
            const int64_t at = Across(network, changer->path, vehicle->changeFrom, changer->s);
            if (at > left)
            {
                Mind(lead, at - DrivingOf(changer)->length - left, changer->speed);
            }
        }
    }
    if (changing && vehicle->s < vehicle->changeAt + vehicle->changeSpan * 3 / 4)
    {
        /* Until its front is over, the one ahead in the lane it leaves. */
        uint32_t ahead = NONE;
        uint32_t behind = NONE;
        FindPlace(traffic, FoxNetworkPathRecord(network, vehicle->changeFrom), left, &ahead,
                  &behind);
        if (ahead != NONE)
        {
            const Vehicle *leader = &traffic->vehicles[ahead];
            Mind(lead, leader->s - DrivingOf(leader)->length - left, leader->speed);
        }
    }
}

/* Whether a car's route can go on from a lane: towards the next road it
 * wants; any lane will do without a route. */
static bool Fits(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex)
{
    if (vehicle->parking || vehicle->routeIndex >= vehicle->routeLength)
    {
        return true;
    }
    const uint32_t after = vehicle->routeIndex + 1u < vehicle->routeLength
                               ? vehicle->route[vehicle->routeIndex + 1u]
                               : NONE;
    return RouteConnection(network, FoxNetworkPathRecord(network, laneIndex),
                           vehicle->route[vehicle->routeIndex], after) != NONE;
}

/* The way a car would take at the end of a lane: its own on its lane, the
 * one its route asks for on another. */
static uint32_t WayOn(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex)
{
    if (laneIndex == vehicle->path || vehicle->routeIndex >= vehicle->routeLength)
    {
        return laneIndex == vehicle->path ? vehicle->connection : NONE;
    }
    const uint32_t after = vehicle->routeIndex + 1u < vehicle->routeLength
                               ? vehicle->route[vehicle->routeIndex + 1u]
                               : NONE;
    return RouteConnection(network, FoxNetworkPathRecord(network, laneIndex),
                           vehicle->route[vehicle->routeIndex], after);
}

/* How hard a car `s` along a lane could accelerate there: behind the car
 * ahead, the one ahead on its way out of the lane or the one still where
 * the ways part, or a light holding the lane. */
static int64_t LaneAcceleration(const FoxTraffic *traffic, const FoxNetwork *network,
                                const Vehicle *vehicle, uint32_t laneIndex, int64_t s)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    Leader lead = {0};
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, lane, s, &ahead, &behind);
    if (ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[ahead];
        Mind(&lead, leader->s - DrivingOf(leader)->length - s, leader->speed);
    }
    SideLeaders(traffic, network, vehicle, laneIndex, s, &lead);
    const int64_t toEnd = lane->length - s;
    const uint32_t way = WayOn(network, vehicle, laneIndex);
    int64_t rear = 0;
    const uint32_t last = lead.found ? NONE : LastOnWay(traffic, network, way, &rear);
    if (last != NONE)
    {
        Mind(&lead, toEnd + rear, traffic->vehicles[last].speed);
    }
    int64_t left = 0;
    const uint32_t departing = Departing(traffic, network, lane, way, &rear, &left);
    if (departing != NONE)
    {
        Mind(&lead, toEnd + rear, traffic->vehicles[departing].speed);
    }
    const FoxLight light = LightAt(network, lane);
    if (light == FOX_LIGHT_RED || light == FOX_LIGHT_YELLOW)
    {
        Mind(&lead, toEnd, 0);
    }
    return DriverModel(DrivingOf(vehicle), vehicle->speed, ScaledLimit(vehicle, lane->speedLimit),
                       lead.gap, lead.speed, lead.found);
}

/* Whether the car coming up behind `s` along a lane from the intersection
 * before it, on a way into the lane or about to take one, would have to
 * brake harder than `brake` for a vehicle joining there. */
static bool CrowdsBehind(const FoxTraffic *traffic, const FoxNetwork *network,
                         const Vehicle *joining, uint32_t laneIndex, int64_t s, int64_t brake)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const FoxRoad *road = &network->roads[lane->road];
    const FoxNode *node = &network->nodes[road->nodes[lane->direction == 0u ? 0u : 1u]];
    const int64_t rear = s - DrivingOf(joining)->length;
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        const FoxPath *way = FoxNetworkPathRecord(network, node->connections[index]);
        if (way->to != laneIndex)
        {
            continue;
        }
        const FoxPath *from = FoxNetworkPathRecord(network, way->from);
        const Vehicle *follower = NULL;
        int64_t gap = 0;
        if (way->front != NONE)
        {
            follower = &traffic->vehicles[way->front];
            gap = rear + way->length - follower->s;
        }
        else if (from->front != NONE &&
                 traffic->vehicles[from->front].connection == node->connections[index])
        {
            follower = &traffic->vehicles[from->front];
            gap = rear + way->length + from->length - follower->s;
        }
        if (follower != NULL &&
            (gap < MIN_GAP || DriverModel(DrivingOf(follower), follower->speed,
                                          ScaledLimit(follower, lane->speedLimit), gap,
                                          joining->speed, true) < -brake))
        {
            return true;
        }
    }
    return false;
}

/* Whether a car may move over to `s` along a lane: with room to the cars
 * there, and neither it nor the one behind, there or coming onto the lane,
 * braking harder than `brake` for the other. Nobody may be partly in the
 * lane beside it, changing out. */
static bool SafeChange(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *vehicle,
                       uint32_t laneIndex, int64_t s, int64_t brake)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const int64_t length = DrivingOf(vehicle)->length;
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, lane, s, &ahead, &behind);
    if (ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[ahead];
        const int64_t gap = leader->s - DrivingOf(leader)->length - s;
        if (gap < MIN_GAP ||
            DriverModel(DrivingOf(vehicle), vehicle->speed, ScaledLimit(vehicle, lane->speedLimit),
                        gap, leader->speed, true) < -brake)
        {
            return false;
        }
    }
    if (behind == NONE && CrowdsBehind(traffic, network, vehicle, laneIndex, s, brake))
    {
        return false;
    }
    if (behind != NONE)
    {
        const Vehicle *follower = &traffic->vehicles[behind];
        const int64_t gap = s - length - follower->s;
        if (gap < MIN_GAP || DriverModel(DrivingOf(follower), follower->speed,
                                         ScaledLimit(follower, lane->speedLimit), gap,
                                         vehicle->speed, true) < -brake)
        {
            return false;
        }
    }
    for (uint32_t index = 0u; index < traffic->changerCount; ++index)
    {
        const Vehicle *changer = &traffic->vehicles[traffic->changers[index]];
        if (changer->changeFrom != laneIndex && changer->path != laneIndex)
        {
            continue;
        }
        const int64_t at = Across(network, changer->path, laneIndex, changer->s);
        const int64_t rear = at - DrivingOf(changer)->length;
        if (at + MIN_GAP > s - length && rear - MIN_GAP < s)
        {
            return false;
        }
        /* Partly in the lane, it follows or leads there as well. */
        if (at <= s - length ? DriverModel(DrivingOf(changer), changer->speed,
                                           ScaledLimit(changer, lane->speedLimit), s - length - at,
                                           vehicle->speed, true) < -brake
                             : DriverModel(DrivingOf(vehicle), vehicle->speed,
                                           ScaledLimit(vehicle, lane->speedLimit), rear - s,
                                           changer->speed, true) < -brake)
        {
            return false;
        }
    }
    return true;
}

/* How far from either end of a lane its lines are solid: near an
 * intersection, where nobody changes lanes. */
static int64_t SolidAt(const FoxNetwork *network, const FoxPath *lane, bool atEnd)
{
    const FoxRoad *road = &network->roads[lane->road];
    const uint32_t node = road->nodes[(lane->direction == 0u) == atEnd ? 1u : 0u];
    return network->nodes[node].roadCount >= 3u ? SOLID_NEAR : 0;
}

/* Whether a car would keep to a lane: its route goes on from there, and
 * the way it takes does not end in a merge. */
static bool Suits(const FoxNetwork *network, const Vehicle *vehicle, uint32_t laneIndex)
{
    return Fits(network, vehicle, laneIndex) &&
           !LaneEnds(network, WayOn(network, vehicle, laneIndex));
}

/* Drivers move over to a lane beside theirs where they can go faster, or
 * where their route needs them or their lane ends, when there is room and
 * away from the solid lines near intersections, and not while the car ahead
 * makes them brake hard. On a highway they keep to the curb unless they
 * pass. A driver who could not get over in time plans its way on from the
 * lane it is in. */
static void ChangeLanes(FoxTraffic *traffic, FoxNetwork *network)
{
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive)
        {
            continue;
        }
        vehicle->changeWait = vehicle->changeWait > 0u ? vehicle->changeWait - 1u : 0u;
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        if (lane->kind != PATH_LANE || vehicle->changeFrom != NONE || vehicle->changeWait != 0u ||
            (traffic->tick + slot) % CHANGE_EVERY_TICKS != 0u || vehicle->permitted ||
            vehicle->claim != NONE || vehicle->through != NONE || vehicle->clearing != NONE)
        {
            continue;
        }
        const int64_t length = DrivingOf(vehicle)->length;
        const int64_t span =
            FoxMax(CHANGE_MIN_SPAN, (int64_t)vehicle->speed * CHANGE_TICKS / TICKS);
        const bool fits = Fits(network, vehicle, vehicle->path);
        if (vehicle->s + span > lane->length - SolidAt(network, lane, true))
        {
            if (!fits)
            {
                /* Too late to get over: on from this lane. */
                PlanRoute(traffic, network, vehicle, vehicle->path, true);
                ChooseConnection(traffic, network, vehicle, false);
            }
            continue;
        }
        if (vehicle->s < length + SolidAt(network, lane, false) ||
            network->roads[lane->road].laneCount[lane->direction] < 2u)
        {
            continue;
        }
        if (vehicle->ahead != NONE)
        {
            /* Its front must get over before it comes up to the one ahead,
             * which drives on meanwhile. */
            const Vehicle *leader = &traffic->vehicles[vehicle->ahead];
            if (leader->s - DrivingOf(leader)->length - vehicle->s +
                    (int64_t)leader->speed * CHANGE_TICKS / TICKS <
                span * 3 / 4)
            {
                continue;
            }
        }
        const int64_t here = LaneAcceleration(traffic, network, vehicle, vehicle->path, vehicle->s);
        const bool urgent = !fits || LaneEnds(network, vehicle->connection);
        if (here < -(urgent ? CHANGE_URGENT_BRAKE : CHANGE_SAFE_BRAKE))
        {
            /* Still in the lane it leaves for a while, the car would have to
             * brake as hard for the one ahead there. */
            continue;
        }
        const bool highway = network->roads[lane->road].type == FOX_ROAD_HIGHWAY;
        uint32_t best = NONE;
        int64_t bestGain = 0;
        for (int32_t side = -1; side <= 1; side += 2)
        {
            const uint32_t target = Neighbour(network, lane, side);
            /* Over to a lane the car would keep to, or on the way to one. */
            bool leads = false;
            for (uint32_t over = target; over != NONE && !leads;
                 over = Neighbour(network, FoxNetworkPathRecord(network, over), side))
            {
                leads = Suits(network, vehicle, over) && (over == target || urgent);
            }
            if (target == NONE || !leads)
            {
                continue;
            }
            const int64_t at = Across(network, vehicle->path, target, vehicle->s);
            const int64_t keep = highway ? (side > 0 ? KEEP_CURB : -KEEP_CURB) : 0;
            const int64_t gain = urgent ? INT64_MAX / 2
                                        : LaneAcceleration(traffic, network, vehicle, target, at) -
                                              here - CHANGE_GAIN - keep;
            if (gain > bestGain && SafeChange(traffic, network, vehicle, target, at,
                                              urgent ? CHANGE_URGENT_BRAKE : CHANGE_SAFE_BRAKE))
            {
                best = target;
                bestGain = gain;
            }
        }
        if (best == NONE)
        {
            continue;
        }
        const uint32_t left = vehicle->path;
        const FoxPath *target = FoxNetworkPathRecord(network, best);
        const int64_t at = Across(network, left, best, vehicle->s);
        vehicle->prevS = Across(network, left, best, vehicle->prevS);
        Unlink(traffic, network, slot);
        Link(traffic, network, slot, best, at);
        vehicle->prevPath = best;
        vehicle->prevGeneration = target->generation;
        vehicle->changeFrom = left;
        vehicle->changeGeneration = FoxNetworkPathRecord(network, left)->generation;
        vehicle->changeAt = at;
        vehicle->changeSpan = span;
        vehicle->changeWait = CHANGE_WAIT_TICKS;
        ++traffic->laneChanges;
        ChooseConnection(traffic, network, vehicle, true);
        if (traffic->changerCount < traffic->changerCapacity)
        {
            traffic->changers[traffic->changerCount++] = slot;
        }
    }
}

/* The cars changing lanes, for the others to see in the lanes they leave;
 * a change is through once the rear is three quarters of the way over and
 * the body clear of the lane left. False without memory. */
static bool GatherChangers(FoxTraffic *traffic, const FoxNetwork *network)
{
    if (traffic->changerCapacity < traffic->capacity)
    {
        uint32_t *grown = realloc(traffic->changers, (size_t)traffic->capacity * sizeof(*grown));
        if (grown == NULL)
        {
            return false;
        }
        traffic->changers = grown;
        traffic->changerCapacity = traffic->capacity;
    }
    traffic->changerCount = 0u;
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive || vehicle->changeFrom == NONE)
        {
            continue;
        }
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        const int64_t rear = vehicle->s - DrivingOf(vehicle)->length;
        if (path->kind != PATH_LANE || !Changing(network, vehicle) ||
            rear >= vehicle->changeAt + vehicle->changeSpan * 3 / 4)
        {
            vehicle->changeFrom = NONE;
            continue;
        }
        traffic->changers[traffic->changerCount++] = slot;
    }
    return true;
}

/* ---- Overtaking ------------------------------------------------------------------ */

/* Whether a car is out in the oncoming lane overtaking, or still pulling in
 * from it. */
static bool Passing(const Vehicle *vehicle)
{
    return vehicle->passOut != PASS_NONE;
}

/* The lane the other way on a road with one lane each way, beside a lane;
 * NONE on other roads. */
static uint32_t Oncoming(const FoxNetwork *network, const FoxPath *lane)
{
    const FoxRoad *road = &network->roads[lane->road];
    return road->laneCount[0] == 1u && road->laneCount[1] == 1u
               ? road->lanes[1u - lane->direction][0]
               : NONE;
}

/* The car ahead that a car keeps its distance to on its lane: past the one
 * it overtakes, and not one overtaking it while that one is still beside
 * it. */
static uint32_t AheadOf(const FoxTraffic *traffic, const Vehicle *vehicle)
{
    uint32_t ahead = vehicle->ahead;
    while (ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[ahead];
        const bool overtaken =
            Passing(vehicle) && vehicle->passBack == PASS_NONE && leader->serial == vehicle->passed;
        const bool overtaking =
            Passing(leader) && leader->passBack == PASS_NONE && leader->passed == vehicle->serial;
        if (!overtaken && !overtaking)
        {
            break;
        }
        ahead = leader->ahead;
    }
    return ahead;
}

/* Cars coming head on in a car's way: on a road with a lane each way, one
 * overtaking the other way towards it, and for a car overtaking, the
 * traffic in the lane it is out in, the gap between their fronts. */
static void HeadOn(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *vehicle,
                   Leader *lead)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
    const uint32_t other = lane->kind == PATH_LANE ? Oncoming(network, lane) : NONE;
    if (other == NONE)
    {
        return;
    }
    for (uint32_t index = 0u; index < traffic->passerCount; ++index)
    {
        const Vehicle *passer = &traffic->vehicles[traffic->passers[index]];
        if (passer->path != other)
        {
            continue;
        }
        const int64_t at = Across(network, other, vehicle->path, passer->s);
        if (at > vehicle->s)
        {
            Mind(lead, at - vehicle->s, 0);
        }
    }
    if (Passing(vehicle) && vehicle->passBack == PASS_NONE)
    {
        const int64_t at = Across(network, vehicle->path, other, vehicle->s);
        uint32_t ahead = NONE;
        uint32_t behind = NONE;
        FindPlace(traffic, FoxNetworkPathRecord(network, other), at, &ahead, &behind);
        if (behind != NONE)
        {
            Mind(lead, at - traffic->vehicles[behind].s, 0);
        }
    }
}

/* A car being overtaken holds back behind the one overtaking it when that
 * one asks to be let in. */
static void LetIn(const FoxTraffic *traffic, const Vehicle *vehicle, Leader *lead)
{
    for (uint32_t index = 0u; index < traffic->passerCount; ++index)
    {
        const Vehicle *passer = &traffic->vehicles[traffic->passers[index]];
        if (passer->passed == vehicle->serial && passer->passBack == PASS_NONE && passer->lettingIn)
        {
            Mind(lead, FoxMax(MIN_GAP / 2, passer->s - DrivingOf(passer)->length - vehicle->s),
                 passer->speed);
        }
    }
}

/* How fast a car overtakes: at the limit, as far as it can, never over it. */
static int64_t PassPace(const Vehicle *vehicle, const FoxPath *lane)
{
    return FoxMin((int64_t)lane->speedLimit, DrivingOf(vehicle)->topSpeed);
}

/* The gap a car overtaking leaves before pulling in ahead of the one it
 * overtook. */
static int64_t ReturnGap(const Vehicle *overtaken)
{
    return MIN_GAP + (int64_t)overtaken->speed * PASS_RETURN_MS / 1000;
}

/* How far a car must see the oncoming lane clear to overtake `leader`, from
 * its pace while overtaking and the oncoming traffic's limit; zero when it
 * would not gain on it fast enough to be worth it. `outReach` is how far
 * it drives meanwhile. */
static int64_t PassRoom(const Vehicle *vehicle, const Vehicle *leader, int64_t pace,
                        int64_t oncomingLimit, int64_t *outReach)
{
    const int64_t gain = pace - leader->speed;
    if (gain < PASS_MIN_GAIN)
    {
        return 0;
    }
    const int64_t gap = leader->s - DrivingOf(leader)->length - vehicle->s;
    const int64_t distance =
        FoxMax(0, gap) + DrivingOf(leader)->length + DrivingOf(vehicle)->length + ReturnGap(leader);
    const int64_t ticks = distance * (int64_t)TICKS / gain;
    *outReach = pace * ticks / (int64_t)TICKS + PASS_SPAN;
    return *outReach + oncomingLimit * ticks / (int64_t)TICKS + PASS_MARGIN;
}

/* How far ahead the oncoming lane is clear of traffic coming towards a car:
 * up to the nearest car on it, or one overtaking the other way, and with
 * nobody on it as far as the drivers through the intersection beyond and
 * at the lines there about to come onto it. */
static int64_t ClearAhead(const FoxTraffic *traffic, const FoxNetwork *network,
                          const Vehicle *vehicle, uint32_t other)
{
    const FoxPath *oncoming = FoxNetworkPathRecord(network, other);
    const int64_t at = Across(network, vehicle->path, other, vehicle->s);
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, oncoming, at, &ahead, &behind);
    int64_t clear = behind != NONE ? at - traffic->vehicles[behind].s : INT64_MAX;
    const FoxRoad *road = &network->roads[oncoming->road];
    const FoxNode *node = &network->nodes[road->nodes[oncoming->direction == 0u ? 0u : 1u]];
    for (uint32_t index = 0u; index < node->connectionCount && behind == NONE; ++index)
    {
        const uint32_t wayIndex = node->connections[index];
        const FoxPath *way = FoxNetworkPathRecord(network, wayIndex);
        const FoxPath *from = FoxNetworkPathRecord(network, way->from);
        if (way->to != other)
        {
            continue;
        }
        if (way->front != NONE)
        {
            clear = FoxMin(clear, at + way->length - traffic->vehicles[way->front].s);
        }
        else if (from->front != NONE && traffic->vehicles[from->front].connection == wayIndex)
        {
            clear =
                FoxMin(clear, at + way->length + from->length - traffic->vehicles[from->front].s);
        }
    }
    for (uint32_t index = 0u; index < traffic->passerCount; ++index)
    {
        const Vehicle *passer = &traffic->vehicles[traffic->passers[index]];
        if (passer->path == vehicle->path && passer != vehicle && passer->s > vehicle->s)
        {
            clear = FoxMin(clear, passer->s - vehicle->s);
        }
    }
    return clear;
}

/* The car a car overtakes, on its lane; NULL when it is gone. */
static const Vehicle *OvertakenBy(const FoxTraffic *traffic, const FoxPath *lane,
                                  const Vehicle *vehicle)
{
    for (uint32_t index = lane->front; index != NONE; index = traffic->vehicles[index].behind)
    {
        if (traffic->vehicles[index].serial == vehicle->passed)
        {
            return &traffic->vehicles[index];
        }
    }
    return NULL;
}

/* Whether a car overtaking could not get past the one it overtakes, gaining
 * on it as slowly as it does now, before the oncoming traffic comes. */
static bool Pressed(const FoxTraffic *traffic, const FoxNetwork *network, const Vehicle *vehicle,
                    const Vehicle *overtaken, const FoxPath *lane, uint32_t other)
{
    const int64_t pace = PassPace(vehicle, lane);
    const int64_t gain = FoxMax(pace - (int64_t)overtaken->speed, PASS_MIN_GAIN / 3);
    const int64_t distance =
        FoxMax(0, overtaken->s + ReturnGap(overtaken) + DrivingOf(vehicle)->length - vehicle->s);
    const int64_t ticks = distance * (int64_t)TICKS / gain;
    const int64_t oncoming = FoxNetworkPathRecord(network, other)->speedLimit;
    return ClearAhead(traffic, network, vehicle, other) <
           (pace + oncoming) * ticks / (int64_t)TICKS + PASS_SPAN + PASS_MARGIN;
}

/* Whether a car is being overtaken: it must not speed up meanwhile. */
static bool Overtaken(const FoxTraffic *traffic, const Vehicle *vehicle)
{
    for (uint32_t index = 0u; index < traffic->passerCount; ++index)
    {
        const Vehicle *passer = &traffic->vehicles[traffic->passers[index]];
        if (passer->passed == vehicle->serial && passer->passBack == PASS_NONE)
        {
            return true;
        }
    }
    return false;
}

/* A car stuck behind a much slower one, driving steadily, on a road with a
 * lane each way overtakes it through the oncoming lane, keeping the limit:
 * where the centre line is dashed, no intersection comes up before it is
 * through, there is room to pull in again ahead of the slower one, and the
 * oncoming lane, as far as the driver sees through the intersection ahead,
 * is clear far enough for the whole manoeuvre at the oncoming traffic's
 * limit. Until it is level with the one it overtakes it pulls back in when
 * someone comes; after that it carries on, and the oncoming car slows for
 * it, unless it could not get past before the oncoming car comes: then it
 * falls back behind the other one, or, more than half past it, that one
 * holds back to let it in. Trucks do not overtake. */
static void Overtake(FoxTraffic *traffic, FoxNetwork *network)
{
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        Vehicle *vehicle = &traffic->vehicles[slot];
        if (!vehicle->alive)
        {
            continue;
        }
        const FoxPath *lane = FoxNetworkPathRecord(network, vehicle->path);
        const uint32_t other = lane->kind == PATH_LANE ? Oncoming(network, lane) : NONE;
        const int64_t length = DrivingOf(vehicle)->length;
        if (Passing(vehicle))
        {
            const Vehicle *overtaken =
                lane->kind == PATH_LANE ? OvertakenBy(traffic, lane, vehicle) : NULL;
            if (lane->kind != PATH_LANE || other == NONE ||
                (vehicle->passBack != PASS_NONE &&
                 vehicle->s - length * 5 / 6 >= vehicle->passBack + PASS_SPAN))
            {
                /* Back in the lane. */
                vehicle->passOut = PASS_NONE;
                continue;
            }
            if (vehicle->passBack != PASS_NONE)
            {
                continue;
            }
            if (overtaken == NULL)
            {
                vehicle->passBack = vehicle->s;
                continue;
            }
            if (vehicle->s > overtaken->s &&
                vehicle->ahead == (uint32_t)(overtaken - traffic->vehicles))
            {
                /* Level and past: ahead of it in the lane. */
                const int64_t s = vehicle->s;
                Unlink(traffic, network, slot);
                Link(traffic, network, slot, vehicle->path, s);
            }
            int64_t reach = 0;
            const int64_t needed =
                PassRoom(vehicle, overtaken, PassPace(vehicle, lane),
                         FoxNetworkPathRecord(network, other)->speedLimit, &reach);
            const int64_t otherLength = DrivingOf(overtaken)->length;
            const int64_t behind = overtaken->s - otherLength - vehicle->s;
            const bool level = behind < 0;
            const bool pressed =
                level && Pressed(traffic, network, vehicle, overtaken, lane, other);
            vehicle->lettingIn = pressed && !vehicle->fallingBack &&
                                 2 * vehicle->s - length >= 2 * overtaken->s - otherLength;
            vehicle->fallingBack = vehicle->fallingBack || (pressed && !vehicle->lettingIn);
            if (vehicle->s - length - overtaken->s >= ReturnGap(overtaken) ||
                (vehicle->fallingBack && behind >= MIN_GAP) ||
                (!level && !vehicle->fallingBack &&
                 (needed == 0 || ClearAhead(traffic, network, vehicle, other) < needed)))
            {
                /* Through, behind it again, or someone comes before it is
                 * level: pull in. */
                vehicle->passBack = vehicle->s;
                vehicle->fallingBack = false;
                vehicle->lettingIn = false;
            }
            continue;
        }
        if (other == NONE || vehicle->kind == FOX_VEHICLE_TRUCK || vehicle->ahead == NONE ||
            (traffic->tick + slot) % CHANGE_EVERY_TICKS != 0u || vehicle->changeFrom != NONE)
        {
            continue;
        }
        const Vehicle *leader = &traffic->vehicles[vehicle->ahead];
        const int64_t pace = PassPace(vehicle, lane);
        const int64_t gap = leader->s - DrivingOf(leader)->length - vehicle->s;
        if (Passing(leader) || leader->speed < PASS_MIN_SPEED ||
            leader->acceleration > PASS_STEADY ||
            (int64_t)leader->speed * 100 > ScaledLimit(vehicle, lane->speedLimit) * PASS_SLOWER ||
            gap > PASS_REACH || vehicle->s < length + SolidAt(network, lane, false))
        {
            continue;
        }
        /* Room to pull in ahead of it. */
        if (leader->ahead != NONE)
        {
            const Vehicle *beyond = &traffic->vehicles[leader->ahead];
            if (beyond->s - DrivingOf(beyond)->length - leader->s <
                length + 2 * ReturnGap(leader) + PASS_ROOM)
            {
                continue;
            }
        }
        int64_t reach = 0;
        const int64_t needed = PassRoom(vehicle, leader, pace,
                                        FoxNetworkPathRecord(network, other)->speedLimit, &reach);
        if (needed == 0 || vehicle->s + reach > lane->length - SolidAt(network, lane, true) ||
            ClearAhead(traffic, network, vehicle, other) < needed)
        {
            continue;
        }
        if (vehicle->behind != NONE && Passing(&traffic->vehicles[vehicle->behind]))
        {
            continue;
        }
        vehicle->passOut = vehicle->s;
        vehicle->passBack = PASS_NONE;
        vehicle->passed = leader->serial;
        vehicle->fallingBack = false;
        vehicle->lettingIn = false;
        ++traffic->overtakes;
        if (traffic->passerCount < traffic->passerCapacity)
        {
            traffic->passers[traffic->passerCount++] = slot;
        }
    }
}

/* The cars overtaking, for the oncoming traffic to see. False without
 * memory. */
static bool GatherPassers(FoxTraffic *traffic)
{
    if (traffic->passerCapacity < traffic->capacity)
    {
        uint32_t *grown = realloc(traffic->passers, (size_t)traffic->capacity * sizeof(*grown));
        if (grown == NULL)
        {
            return false;
        }
        traffic->passers = grown;
        traffic->passerCapacity = traffic->capacity;
    }
    traffic->passerCount = 0u;
    for (uint32_t slot = 0u; slot < traffic->capacity; ++slot)
    {
        if (traffic->vehicles[slot].alive && Passing(&traffic->vehicles[slot]))
        {
            traffic->passers[traffic->passerCount++] = slot;
        }
    }
    return true;
}

/* ---- Spawning ------------------------------------------------------------------ */

/* Whether a new vehicle may join a lane `s` along it, and how fast: as fast
 * as it wants, but no faster than lets it stop at the line and follow the
 * car ahead without braking harder than comfortable, with nobody behind
 * braking harder than that for it, and clear of every car there, changing
 * lanes or overtaking. */
static bool JoinAt(const FoxTraffic *traffic, const FoxNetwork *network, Vehicle *joining,
                   const FoxPath *lane, uint32_t laneIndex, int64_t s)
{
    const Driving *driving = DrivingOf(joining);
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, lane, s, &ahead, &behind);
    if ((ahead != NONE && traffic->vehicles[ahead].s - s < SPAWN_CLEARANCE) ||
        (behind != NONE && s - traffic->vehicles[behind].s < SPAWN_CLEARANCE))
    {
        return false;
    }
    const int64_t desired = ScaledLimit(joining, lane->speedLimit);
    Leader lead = {.found = true, .gap = lane->length - s, .speed = 0};
    if (ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[ahead];
        Mind(&lead, leader->s - DrivingOf(leader)->length - s, leader->speed);
    }
    SideLeaders(traffic, network, joining, laneIndex, s, &lead);
    /* As if a light ahead held it: the fastest that needs no hard braking. */
    int64_t speed = desired;
    while (speed > 0 &&
           DriverModel(driving, speed, desired, lead.gap, lead.speed, true) < -CHANGE_SAFE_BRAKE)
    {
        speed = speed * 3 / 4 - FOX_METRES(1) / 4;
    }
    joining->speed = (int32_t)FoxMax(0, speed);
    if (behind == NONE && CrowdsBehind(traffic, network, joining, laneIndex, s, CHANGE_SAFE_BRAKE))
    {
        return false;
    }
    const uint32_t other = Oncoming(network, lane);
    for (uint32_t index = 0u; index < traffic->passerCount && other != NONE; ++index)
    {
        const Vehicle *passer = &traffic->vehicles[traffic->passers[index]];
        const int64_t at = Across(network, other, laneIndex, passer->s);
        if (passer->path == other && at - DrivingOf(passer)->length < s + SPAWN_CLEARANCE &&
            at > s - SPAWN_CLEARANCE)
        {
            return false;
        }
    }
    return SafeChange(traffic, network, joining, laneIndex, s, CHANGE_SAFE_BRAKE);
}

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

/* A road picked with odds by its length and lanes, so that cars spread
 * evenly over the network; NONE without roads. */
static uint32_t RandomRoad(FoxTraffic *traffic, const FoxNetwork *network)
{
    int64_t total = 0;
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        total += road->alive && !road->closing
                     ? road->length * (road->laneCount[0] + road->laneCount[1])
                     : 0;
    }
    if (total <= 0)
    {
        return NONE;
    }
    int64_t pick = FoxRandomRange(&traffic->random, 0, total - 1);
    for (uint32_t index = 0u; index < network->roadCapacity; ++index)
    {
        const FoxRoad *road = &network->roads[index];
        const int64_t share = road->alive && !road->closing
                                  ? road->length * (road->laneCount[0] + road->laneCount[1])
                                  : 0;
        if (pick < share)
        {
            return index;
        }
        pick -= share;
    }
    return NONE;
}

static void Spawn(FoxTraffic *traffic, FoxNetwork *network)
{
    for (uint32_t spawn = 0u;
         spawn < SPAWN_PER_TICK && traffic->count < traffic->target && network->roadCount != 0u;
         ++spawn)
    {
        const uint32_t roadIndex = RandomRoad(traffic, network);
        if (roadIndex == NONE)
        {
            return;
        }
        const FoxRoad *road = &network->roads[roadIndex];
        const uint32_t pick =
            FoxRandomBelow(&traffic->random, road->laneCount[0] + road->laneCount[1]);
        const uint32_t laneIndex = pick < road->laneCount[0]
                                       ? road->lanes[0][pick]
                                       : road->lanes[1][pick - road->laneCount[0]];
        const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
        if (lane == NULL || lane->retired || lane->length < 2 * SPAWN_CLEARANCE)
        {
            continue;
        }
        const int64_t s =
            FoxRandomRange(&traffic->random, SPAWN_CLEARANCE, lane->length - SPAWN_CLEARANCE);
        const uint64_t serial = traffic->nextSerial;
        Vehicle joining = {
            .alive = true,
            .serial = serial,
            .prevPath = NONE,
            .ahead = NONE,
            .behind = NONE,
            .connection = NONE,
            .claim = NONE,
            .clearing = NONE,
            .through = NONE,
            .changeFrom = NONE,
            .passOut = PASS_NONE,
            .passBack = PASS_NONE,
            .holdAt = HOLD_NONE,
            .destination = NONE,
            .random = FoxRandomStream(traffic->seed, serial),
        };
        joining.kind = FoxRandomBelow(&joining.random, 100u) < TRUCK_PERCENT &&
                               !LeadsToDeadEnd(network, EdgeOf(lane))
                           ? (uint8_t)FOX_VEHICLE_TRUCK
                           : (uint8_t)FOX_VEHICLE_CAR;
        joining.color = (uint8_t)FoxRandomBelow(&joining.random, COLORS);
        /* The vehicle's pace, and over it the driver's manner. */
        const int64_t pace = FoxRandomRange(&joining.random, DrivingOf(&joining)->slowest,
                                            DrivingOf(&joining)->fastest);
        const int64_t bold =
            traffic->manner == 0u
                ? 0
                : FoxRandomRange(&joining.random, 0, (int64_t)traffic->manner * FOX_UNIT / 100);
        joining.desire = (int32_t)(pace + bold);
        if (!JoinAt(traffic, network, &joining, lane, laneIndex, s))
        {
            continue;
        }
        const uint32_t slot = AcquireSlot(traffic);
        if (slot == NONE)
        {
            return;
        }
        ++traffic->nextSerial;
        Vehicle *vehicle = &traffic->vehicles[slot];
        *vehicle = joining;
        Link(traffic, network, slot, laneIndex, s);
        vehicle->prevPath = laneIndex;
        vehicle->prevGeneration = vehicle->pathGeneration;
        vehicle->prevS = s;
        ++traffic->count;
        ++traffic->spawned;
        PlanRoute(traffic, network, vehicle, laneIndex, false);
        ChooseConnection(traffic, network, vehicle, true);
    }
}

/* ---- Moving -------------------------------------------------------------------- */

/* How far ahead a car in an intersection stops for a way that a car still
 * covers where the two cross, as a driver does who counted on that car
 * being through and finds it is not, whoever had the right of way; HOLD_NONE
 * when nobody is there or the car is already too far in to stop short. The
 * car in the crossing itself always drives on, so nobody waits in a ring. */
static int64_t StillInWay(const FoxTraffic *traffic, const FoxNetwork *network,
                          const Vehicle *vehicle)
{
    const FoxPath *connection = FoxNetworkPathRecord(network, vehicle->path);
    int64_t first = HOLD_NONE;
    for (uint32_t index = 0u; index < connection->conflictCount; ++index)
    {
        const FoxConflict *conflict = &connection->conflicts[index];
        const FoxPath *other = FoxNetworkPathRecord(network, conflict->path);
        if (other == NULL)
        {
            continue;
        }
        const int64_t stop = MeetingOn(network, vehicle->path, other) - conflict->room;
        if (stop <= vehicle->s || stop - vehicle->s >= first)
        {
            continue;
        }
        for (uint32_t slot = other->front; slot != NONE; slot = traffic->vehicles[slot].behind)
        {
            const Vehicle *holder = &traffic->vehicles[slot];
            if (holder->s > conflict->otherAt - conflict->room &&
                holder->s - DrivingOf(holder)->length < conflict->otherAt + conflict->room)
            {
                first = stop - vehicle->s;
                break;
            }
        }
    }
    return first;
}

/* Slows a car down in time for a slower path `distance` ahead with the
 * limit `limit`: it aims for a speed from which comfortable braking makes
 * it, and brakes at least as hard as still reaching it needs. */
static void SlowFor(const Vehicle *vehicle, int64_t speed, int32_t limit, int64_t distance,
                    int64_t *desired, int64_t *needed)
{
    const int64_t pace = ScaledLimit(vehicle, limit);
    const int64_t ahead = FoxMax(FOX_METRES(1), distance);
    const int64_t brake = DrivingOf(vehicle)->brake;
    const int64_t reachable = (int64_t)FoxIntegerSqrt((uint64_t)(pace * pace + 2 * brake * ahead));
    *desired = FoxMin(*desired, reachable);
    /* Only faster than comfortable braking still slows it in time does it
     * have to brake now; until then it may still speed up. */
    if (speed > reachable)
    {
        *needed = FoxMax(*needed, (speed * speed - pace * pace) / (2 * ahead));
    }
}

/* No faster than a car could still stop a line `distance` ahead where it may
 * have to give way, braking comfortably, and braking as hard as stopping
 * there takes once faster than that. */
static void StoppableAt(const Driving *driving, int64_t speed, int64_t distance, int64_t *desired,
                        int64_t *needed)
{
    const int64_t room = FoxMax(0, distance - STOP_SHORT);
    const int64_t stoppable = (int64_t)FoxIntegerSqrt((uint64_t)(2 * driving->brake * room));
    *desired = FoxMin(*desired, stoppable);
    if (speed > stoppable)
    {
        *needed = FoxMax(*needed, speed * speed / (2 * FoxMax(FOX_METRES(1), room)));
    }
}

static void Accelerate(const FoxTraffic *traffic, const FoxNetwork *network, Vehicle *vehicle)
{
    const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
    const Driving *driving = DrivingOf(vehicle);
    const int64_t speed = vehicle->speed;
    int64_t desired = ScaledLimit(vehicle, path->speedLimit);
    const Vehicle *overtaken =
        Passing(vehicle) && vehicle->passBack == PASS_NONE && vehicle->fallingBack
            ? OvertakenBy(traffic, path, vehicle)
            : NULL;
    if (overtaken != NULL)
    {
        /* Falling back behind the one it overtakes. */
        desired = FoxMax(0, (int64_t)overtaken->speed - PASS_MIN_GAIN);
    }
    else if (Passing(vehicle) && vehicle->passBack == PASS_NONE)
    {
        desired = PassPace(vehicle, path);
    }
    else if (Overtaken(traffic, vehicle))
    {
        /* Nobody speeds up while being overtaken. */
        desired = FoxMin(desired, (int64_t)vehicle->speed);
    }
    if (path->axle && vehicle->leg % 2u == 1u)
    {
        desired = desired * 2 / 3;
    }
    /* Over the limit of the path itself: comfortable braking at least. */
    int64_t needed = speed > desired ? driving->brake : 0;
    if (path->axle && vehicle->leg < path->cuspCount)
    {
        /* To a stop where the leg ends: no faster than comfortable braking
         * stops it there, and braking as hard as stopping there takes once
         * faster than that. */
        const int64_t toCusp = FoxMax(1, path->cusps[vehicle->leg] - vehicle->s);
        const int64_t stopping = (int64_t)FoxIntegerSqrt((uint64_t)(2 * driving->brake * toCusp));
        desired = FoxMin(desired, stopping);
        if (speed > stopping)
        {
            needed = FoxMax(needed, speed * speed / (2 * toCusp));
        }
    }
    int64_t gap = 0;
    int64_t leaderSpeed = 0;
    bool hasLeader = false;
    const uint32_t front = AheadOf(traffic, vehicle);
    if (front != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[front];
        gap = leader->s - DrivingOf(leader)->length - vehicle->s;
        leaderSpeed = leader->speed;
        hasLeader = true;
    }
    if (path->kind == PATH_LANE)
    {
        Leader lead = {.found = hasLeader, .gap = gap, .speed = leaderSpeed};
        SideLeaders(traffic, network, vehicle, vehicle->path, vehicle->s, &lead);
        HeadOn(traffic, network, vehicle, &lead);
        LetIn(traffic, vehicle, &lead);
        gap = lead.gap;
        leaderSpeed = lead.speed;
        hasLeader = lead.found;
    }
    int64_t travelled = path->length - vehicle->s;
    uint32_t next = NONE;
    int64_t rear = 0;
    int64_t left = 0;
    const uint32_t departing =
        path->kind == PATH_LANE && !hasLeader
            ? Departing(traffic, network, path, vehicle->connection, &rear, &left)
            : NONE;
    if (departing != NONE && Lingers(&traffic->vehicles[departing], left, vehicle, travelled))
    {
        /* A car from this lane still where the ways part when this one
         * gets there: it comes up to the line no sooner than that one is
         * gone, and follows it when it stands. */
        const int64_t pace = traffic->vehicles[departing].speed;
        if (pace > STANDING_SPEED)
        {
            desired = FoxMin(desired, FoxMax(FOX_METRES(1), travelled * pace / left));
        }
        else
        {
            gap = travelled + rear;
            leaderSpeed = pace;
            hasLeader = true;
        }
    }
    /* Where the car has to stop, closer than any car ahead. */
    int64_t stop = HOLD_NONE;
    if (path->kind == PATH_LANE)
    {
        if (vehicle->connection == NONE || (!vehicle->permitted && !vehicle->clear))
        {
            /* The stop line, or the end of the road, and the car gone on
             * the way ahead that may still stand across the line. */
            stop = travelled;
            int64_t behind = 0;
            const uint32_t last =
                hasLeader ? NONE : LastOnWay(traffic, network, vehicle->connection, &behind);
            if (last != NONE)
            {
                gap = travelled + behind;
                leaderSpeed = traffic->vehicles[last].speed;
                hasLeader = true;
            }
        }
        else
        {
            next = vehicle->connection;
        }
        if (vehicle->connection != NONE && !vehicle->permitted &&
            GivesWay(network, path, vehicle->connection))
        {
            StoppableAt(driving, speed, travelled, &desired, &needed);
        }
    }
    else
    {
        next = path->to;
    }
    bool lookedAhead = false;
    bool held = false;
    while (next != NONE && travelled < LOOKAHEAD)
    {
        const FoxPath *upcoming = FoxNetworkPathRecord(network, next);
        if (upcoming == NULL)
        {
            break;
        }
        SlowFor(vehicle, speed, upcoming->speedLimit, travelled, &desired, &needed);
        if (!hasLeader)
        {
            /* The last car on the way ahead, and those changing out of a
             * lane ahead that are still partly in it. */
            Leader lead = {.found = false};
            if (upcoming->back != NONE)
            {
                const Vehicle *leader = &traffic->vehicles[upcoming->back];
                Mind(&lead, travelled + leader->s - DrivingOf(leader)->length, leader->speed);
            }
            if (upcoming->kind == PATH_LANE)
            {
                SideLeaders(traffic, network, vehicle, next, -travelled, &lead);
            }
            gap = lead.gap;
            leaderSpeed = lead.speed;
            hasLeader = lead.found;
        }
        uint32_t onward = NONE;
        if (upcoming->kind == PATH_LANE && !lookedAhead)
        {
            /* The intersection at the end of the next lane, seen from here:
             * where its light or a stop sign will stop the car, or cars hold
             * the way it takes, stop at its line, and else slow down in time
             * for that way. */
            lookedAhead = true;
            onward = PlanThrough(network, vehicle, next);
            const int64_t toLine = travelled + upcoming->length;
            const uint32_t turn = onward != NONE ? NONE : TurnAhead(network, vehicle, next);
            if (turn != NONE)
            {
                /* From the lane it changes over to on the way. */
                SlowFor(vehicle, speed, FoxNetworkPathRecord(network, turn)->speedLimit, toLine,
                        &desired, &needed);
            }
            for (uint32_t index = 0u; onward == NONE && turn == NONE && !vehicle->parking &&
                                      index < upcoming->outgoingCount;
                 ++index)
            {
                /* Not knowing the way it takes there yet, it can take the
                 * slowest. */
                SlowFor(vehicle, speed,
                        FoxNetworkPathRecord(network, upcoming->outgoing[index])->speedLimit,
                        toLine, &desired, &needed);
            }
            if (GivesWay(network, upcoming, onward))
            {
                StoppableAt(driving, speed, toLine, &desired, &needed);
            }
            held = !StopsAhead(network, vehicle, upcoming, onward, toLine);
            if (!held || HeldAhead(traffic, network, vehicle, upcoming, onward, toLine))
            {
                stop = toLine;
                break;
            }
            held = false;
        }
        travelled += upcoming->length;
        next = upcoming->kind == PATH_CONNECTION ? upcoming->to : onward;
    }
    /* Where it waits inside the intersection, and short of a crossing still
     * taken. */
    const int64_t ahead = stop;
    stop = FoxMin(stop, ToHold(network, vehicle));
    if (path->kind == PATH_CONNECTION)
    {
        stop = FoxMin(stop, StillInWay(traffic, network, vehicle));
    }
    int32_t acceleration =
        DriverModel(driving, vehicle->speed, desired, gap, leaderSpeed, hasLeader);
    if (stop != HOLD_NONE && (!hasLeader || stop < gap))
    {
        /* Cars holding a way at the intersection beyond the next lane are
         * mostly on their way out of it: for them a driver brakes no harder
         * than firmly, and at its own line as hard as it must. */
        const int32_t stopping = StopModel(driving, speed, desired, stop);
        acceleration = (int32_t)FoxMin(
            acceleration, held && stop == ahead ? FoxMax(stopping, -FIRM_BRAKE) : stopping);
    }
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
    const uint32_t ahead = AheadOf(traffic, vehicle);
    if (ahead != NONE)
    {
        const Vehicle *leader = &traffic->vehicles[ahead];
        room = leader->s - DrivingOf(leader)->length - FOX_METRES(1) / 2 - vehicle->s;
        roomSpeed = leader->speed;
    }
    else if (path->kind == PATH_LANE && (vehicle->connection == NONE || !vehicle->permitted))
    {
        room = path->length - FOX_METRES(1) / 4 - vehicle->s;
        int64_t rear = 0;
        const uint32_t last = LastOnWay(traffic, network, vehicle->connection, &rear);
        if (last != NONE && path->length - vehicle->s + rear - FOX_METRES(1) / 2 < room)
        {
            room = path->length - vehicle->s + rear - FOX_METRES(1) / 2;
            roomSpeed = traffic->vehicles[last].speed;
        }
    }
    else if (path->kind == PATH_LANE)
    {
        int64_t rear = 0;
        const uint32_t last = LastOnWay(traffic, network, vehicle->connection, &rear);
        if (last != NONE)
        {
            room = path->length - vehicle->s + rear - FOX_METRES(1) / 2;
            roomSpeed = traffic->vehicles[last].speed;
        }
    }
    else
    {
        const FoxPath *upcoming = FoxNetworkPathRecord(network, path->to);
        if (upcoming->back != NONE)
        {
            const Vehicle *leader = &traffic->vehicles[upcoming->back];
            room = path->length - vehicle->s + leader->s - DrivingOf(leader)->length -
                   FOX_METRES(1) / 2;
            roomSpeed = leader->speed;
        }
    }
    if (path->kind == PATH_LANE)
    {
        Leader nearest = {.nearest = true};
        SideLeaders(traffic, network, vehicle, vehicle->path, vehicle->s, &nearest);
        HeadOn(traffic, network, vehicle, &nearest);
        if (nearest.found && nearest.gap - FOX_METRES(1) / 2 < room)
        {
            room = nearest.gap - FOX_METRES(1) / 2;
            roomSpeed = nearest.speed;
        }
    }
    int64_t rear = 0;
    int64_t left = 0;
    const uint32_t departing =
        path->kind == PATH_LANE && ahead == NONE
            ? Departing(traffic, network, path, vehicle->connection, &rear, &left)
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
    const bool turning = path->axle && vehicle->leg < path->cuspCount;
    if (turning && path->cusps[vehicle->leg] - vehicle->s < room)
    {
        room = path->cusps[vehicle->leg] - vehicle->s;
        roomSpeed = 0;
    }
    if (advance > room)
    {
        advance = FoxMax(0, room);
        speed = FoxMin(speed, roomSpeed);
    }
    vehicle->s += advance;
    vehicle->speed = (int32_t)speed;
    if (turning && path->cusps[vehicle->leg] - vehicle->s <= CUSP_REACH && speed <= STANDING_SPEED)
    {
        /* Stood at the end of the leg, then into gear the other way. */
        vehicle->s = path->cusps[vehicle->leg];
        vehicle->speed = 0;
        if (++vehicle->pause >= CUSP_PAUSE_TICKS)
        {
            ++vehicle->leg;
            vehicle->pause = 0u;
        }
    }
    if (speed <= STANDING_SPEED)
    {
        ++vehicle->waitTicks;
    }
    /* Creeping on in a queue a car's length at a time is no standing. */
    vehicle->crept = speed > MOVING_SPEED ? PROGRESS : vehicle->crept + advance;
    if (vehicle->crept >= PROGRESS)
    {
        vehicle->waitTicks = 0u;
        vehicle->crept = 0;
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
                /* Nowhere to go from here yet: it waits at the end of the
                 * lane and looks again every second. */
                vehicle->s = path->length - 1;
                vehicle->speed = 0;
                if ((traffic->tick + slot) % TICKS == 0u)
                {
                    ChooseConnection(traffic, network, vehicle, false);
                }
                break;
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
            vehicle->leg = 0u;
            vehicle->pause = 0u;
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
        if (!road->alive || road->closing)
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
    if (GatherChangers(traffic, network))
    {
        ChangeLanes(traffic, network);
    }
    if (GatherPassers(traffic))
    {
        Overtake(traffic, network);
    }
    Spawn(traffic, network);
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

/* Whether a road still ahead on a car's route is gone or rebuilt since the
 * route was planned. */
static bool RouteBroken(const FoxNetwork *network, const Vehicle *vehicle)
{
    for (uint32_t index = vehicle->routeIndex; index < vehicle->routeLength; ++index)
    {
        const uint32_t roadIndex = vehicle->route[index] / 2u;
        if (roadIndex >= network->roadCapacity || !network->roads[roadIndex].alive ||
            network->roads[roadIndex].revision > vehicle->routeRevision)
        {
            return true;
        }
    }
    return false;
}

/* Whether a vehicle of `length` fits on a path at `s` among the cars there. */
static bool RoomAt(const FoxTraffic *traffic, const FoxPath *path, int64_t s, int64_t length)
{
    uint32_t ahead = NONE;
    uint32_t behind = NONE;
    FindPlace(traffic, path, s, &ahead, &behind);
    const Vehicle *leader = ahead != NONE ? &traffic->vehicles[ahead] : NULL;
    return (leader == NULL || leader->s - DrivingOf(leader)->length - s >= MIN_GAP / 2) &&
           (behind == NONE || s - length - traffic->vehicles[behind].s >= MIN_GAP / 2);
}

/* The nearest free spot for a car on a lane or the lanes beside it the same
 * way, within a few car lengths of `s` on `laneIndex`: the lane, NONE where
 * there is none, and the spot through outS. */
static uint32_t FreeSpotNear(const FoxTraffic *traffic, const FoxNetwork *network,
                             const Vehicle *vehicle, uint32_t laneIndex, int64_t s, int64_t *outS)
{
    const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
    const int64_t length = DrivingOf(vehicle)->length;
    if (lane == NULL || lane->kind != PATH_LANE)
    {
        *outS = s;
        return lane != NULL && RoomAt(traffic, lane, s, length) ? laneIndex : NONE;
    }
    const FoxRoad *road = &network->roads[lane->road];
    const int64_t centre = FoxLaneToCentre(lane, s);
    for (uint32_t step = 0u; step < REJOIN_STEPS; ++step)
    {
        /* Here, then a car length further back, further on, and so on. */
        const int64_t reach = (int64_t)((step + 1u) / 2u) * (length + MIN_GAP);
        const int64_t offset = step % 2u == 1u ? -reach : reach;
        for (uint32_t index = 0u; index <= road->laneCount[lane->direction]; ++index)
        {
            /* Its own lane first, then the others. */
            const uint32_t candidate =
                index == 0u ? laneIndex : road->lanes[lane->direction][index - 1u];
            const FoxPath *other = FoxNetworkPathRecord(network, candidate);
            if ((index != 0u && candidate == laneIndex) || other == NULL || other->retired)
            {
                continue;
            }
            const int64_t at = FoxCentreToLane(other, centre) + offset;
            if (at >= length && at < other->length && RoomAt(traffic, other, at, length))
            {
                *outS = at;
                return candidate;
            }
        }
    }
    return NONE;
}

/* Where a car goes whose path a change took away without a successor: the
 * nearest live lane running its way, near where it was. */
static uint32_t NearestLane(const FoxNetwork *network, const Vehicle *vehicle, int64_t *outS)
{
    FoxVec point;
    FoxVec heading;
    if (!FoxNetworkPathPoint(network, vehicle->path, vehicle->s, &point, &heading))
    {
        return NONE;
    }
    int64_t along = 0;
    const uint32_t roadIndex = FoxNetworkNearestRoad(network, point, REJOIN_REACH, &along);
    if (roadIndex == NONE)
    {
        return NONE;
    }
    const FoxRoad *road = &network->roads[roadIndex];
    uint32_t best = NONE;
    int64_t bestDot = INT64_MIN;
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        if (road->laneCount[direction] == 0u)
        {
            continue;
        }
        const uint32_t laneIndex = road->lanes[direction][0];
        const FoxPath *lane = FoxNetworkPathRecord(network, laneIndex);
        FoxVec at;
        FoxVec forward;
        if (lane == NULL ||
            !FoxNetworkPathPoint(network, laneIndex, FoxCentreToLane(lane, along), &at, &forward))
        {
            continue;
        }
        const int64_t dot = FoxVecDot(forward, heading);
        if (dot > bestDot)
        {
            bestDot = dot;
            best = laneIndex;
            *outS = FoxCentreToLane(lane, along);
        }
    }
    return best;
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
        /* Onto the path that took its place, or failing that the nearest
         * lane its way; where cars already stand there, the nearest free
         * spot. Only with no room anywhere near does it leave the road. */
        int64_t s = 0;
        uint32_t moved = MapPath(network, vehicle->path, vehicle->s, &s);
        if (moved == NONE)
        {
            moved = NearestLane(network, vehicle, &s);
        }
        moved = moved != NONE ? FreeSpotNear(traffic, network, vehicle, moved, s, &s) : NONE;
        if (moved == NONE)
        {
            Kill(traffic, network, slot, false);
            continue;
        }
        const FoxPath *target = FoxNetworkPathRecord(network, moved);
        uint32_t ahead = NONE;
        uint32_t behind = NONE;
        FindPlace(traffic, target, s, &ahead, &behind);
        if (ahead != NONE)
        {
            /* Never faster than the car it now has ahead. */
            vehicle->speed =
                (int32_t)FoxMin((int64_t)vehicle->speed, (int64_t)traffic->vehicles[ahead].speed);
        }
        Link(traffic, network, slot, moved, s);
        vehicle->trailCount = 0u;
        vehicle->holdAt = HOLD_NONE;
        vehicle->changeFrom = NONE;
        vehicle->passOut = PASS_NONE;
        if (target->kind == PATH_CONNECTION)
        {
            /* In the intersection, it holds the way it is on. */
            vehicle->connection = moved;
            vehicle->claim = moved;
            vehicle->permitted = true;
        }
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
        /* A turn that went away is chosen again; a claim on it went with it.
         * A driver not holding its way yet whose route goes over a road gone
         * or rebuilt plans it afresh; one that could take a new shortcut
         * finds it when it plans again anyway. */
        const FoxPath *path = FoxNetworkPathRecord(network, vehicle->path);
        const bool routed = network->router != NULL && !vehicle->parking &&
                            vehicle->claim == NONE && RouteBroken(network, vehicle);
        if (path->kind == PATH_LANE && routed)
        {
            PlanRoute(traffic, network, vehicle, vehicle->path, true);
        }
        const FoxPath *turn =
            vehicle->connection != NONE && FoxNetworkPathLive(network, vehicle->connection)
                ? FoxNetworkPathRecord(network, vehicle->connection)
                : NULL;
        /* Its turn stays, and its place in the rounds, while it still leaves
         * its lane, the route still goes that way and, unless it has leave
         * to, not onto a closing road. */
        const FoxPath *beyond =
            turn != NULL && turn->kind == PATH_CONNECTION && turn->from == vehicle->path &&
                    (vehicle->claim != NONE || !Closed(network, vehicle->connection))
                ? FoxNetworkPathRecord(network, turn->to)
                : NULL;
        const bool keeps =
            beyond != NULL && (!routed || vehicle->routeIndex >= vehicle->routeLength ||
                               EdgeOf(beyond) == vehicle->route[vehicle->routeIndex]);
        if (path->kind == PATH_LANE && !keeps)
        {
            ChooseConnection(traffic, network, vehicle, true);
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

/* Whether a remembered path is still the one the vehicle drove, and traces
 * the front: behind a dead end's turn, which traces the rear axle, the
 * body runs straight back from the lane out. */
static bool TrailLive(const FoxNetwork *network, uint32_t path, uint32_t generation)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    return record != NULL && record->generation == generation && record->count >= 2u &&
           !record->axle;
}

/* A smooth step from 0 to FOX_UNIT as `t` goes from 0 to FOX_UNIT. */
static int64_t Ease(int64_t t)
{
    t = FoxClamp(t, 0, FOX_UNIT);
    return t * t / FOX_UNIT * (3 * FOX_UNIT - 2 * t) / FOX_UNIT;
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
    else if (path == vehicle->path && vehicle->changeFrom != NONE && Changing(network, vehicle))
    {
        /* Changing lanes, the body eases over from the lane it left. */
        const int64_t eased =
            Ease((distance - vehicle->changeAt) * FOX_UNIT / FoxMax(1, vehicle->changeSpan));
        FoxVec left;
        (void)FoxNetworkPathPoint(network, vehicle->changeFrom,
                                  Across(network, path, vehicle->changeFrom, distance), &left,
                                  NULL);
        point = FoxVecLerp(left, point, eased);
    }
    const FoxPath *lane = FoxNetworkPathRecord(network, path);
    const uint32_t other = lane->kind == PATH_LANE ? Oncoming(network, lane) : NONE;
    if (distance >= 0 && path == vehicle->path && Passing(vehicle) && other != NONE)
    {
        /* Overtaking, the body swings out into the oncoming lane and back. */
        const int64_t out = Ease((distance - vehicle->passOut) * FOX_UNIT / PASS_SPAN);
        const int64_t back = vehicle->passBack == PASS_NONE
                                 ? 0
                                 : Ease((distance - vehicle->passBack) * FOX_UNIT / PASS_SPAN);
        FoxVec beside;
        (void)FoxNetworkPathPoint(network, other, Across(network, path, other, distance), &beside,
                                  NULL);
        point = FoxVecLerp(point, beside, out * (FOX_UNIT - back) / FOX_UNIT);
    }
    return point;
}

/* The pose of a car on a dead end's turn, its rear axle `rear` along it: on
 * the path, facing along it forward and against it in reverse, or before
 * the path still straight in the lane it came by. The heading is taken
 * over a short stretch of the leg, so it turns smoothly. */
static void TurnPose(const FoxNetwork *network, const Vehicle *vehicle, const FoxPath *path,
                     int64_t rear, FoxVec *outPosition, FoxVec *outForward)
{
    const int64_t length = DrivingOf(vehicle)->length;
    FoxVec axle;
    FoxVec forward;
    if (rear < 0)
    {
        axle = PointAlong(network, vehicle, rear - length * 5 / 6);
        forward = FoxVecDirection(FoxVecSub(PointAlong(network, vehicle, rear - length / 6), axle));
    }
    else
    {
        uint32_t leg = 0u;
        int64_t from = 0;
        while (leg < path->cuspCount && path->cusps[leg] < rear)
        {
            from = path->cusps[leg++];
        }
        const int64_t to = leg < path->cuspCount ? path->cusps[leg] : path->length;
        const int64_t reach = FOX_METRES(1) / 3;
        FoxVec before;
        FoxVec after;
        (void)FoxNetworkPathPoint(network, vehicle->path, rear, &axle, NULL);
        (void)FoxNetworkPathPoint(network, vehicle->path, FoxMax(from, rear - reach), &before,
                                  NULL);
        (void)FoxNetworkPathPoint(network, vehicle->path, FoxMin(to, rear + reach), &after, NULL);
        forward = FoxVecDirection(FoxVecSub(after, before));
        if (leg % 2u == 1u)
        {
            forward = FoxVecNegate(forward);
        }
    }
    if (FoxVecIsZero(forward))
    {
        (void)FoxNetworkPathPoint(network, vehicle->path, FoxMax(0, rear), NULL, &forward);
    }
    if (outPosition != NULL)
    {
        /* The middle of the body lies a third of it ahead of the rear axle. */
        *outPosition = FoxVecAdd(axle, FoxVecScale(forward, length / 3, FOX_UNIT));
    }
    if (outForward != NULL)
    {
        *outForward = forward;
    }
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
    if (path->axle)
    {
        /* On a dead end's turn the distance is the rear axle's. */
        TurnPose(network, vehicle, path, front, outPosition, outForward);
        return true;
    }
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
    stats.laneChanges = traffic->laneChanges;
    stats.overtakes = traffic->overtakes;
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
