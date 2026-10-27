#include "expect.h"
#include "sim/network_internal.h"
#include "sim/world.h"
#include "test_router.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Watches the demo town tick by tick for broken rules of the road: going
 * over a red light or past a stop sign without stopping, two cars in the
 * same place where their ways cross, a car with right of way braking hard
 * for one that should have given way, and cars with right of way and room
 * ahead standing at their line. Everywhere, two cars' bodies must never
 * overlap: changing lanes, overtaking or turning round. Out on the roads
 * round the planet drivers change lanes, and on a country road they
 * overtake slow trucks. */

#define TICKS FOX_TRAFFIC_TICKS_PER_SECOND
#define SECONDS(seconds) ((uint64_t)(seconds) * TICKS)
#define STANDING (FOX_METRES(1) / 2)
#define STOP_REACH FOX_METRES(5)
/* Two cars closer than this to where their ways cross, both at once; a car
 * waiting this far short of it is out of the way. */
#define TOUCH FOX_METRES(1)
#define WAITING_SHORT FOX_METRES(3)
/* Braking this hard is not something a driver with right of way should
 * have to do: well past comfortable. */
#define HARD_BRAKE (FOX_METRES(9) / 2)
/* A car standing this close to its line counts as stopped there, and the
 * lane beyond is free when it has this much room besides the car. */
#define AT_LINE FOX_METRES(8)
#define CLEAR_AHEAD FOX_METRES(4)
/* More vehicle slots than the town ever uses. */
#define SLOTS 1024u
/* Bodies are a car's width and a truck's, and count as overlapping only
 * this far into each other; cars further apart than the reach are not
 * compared. */
#define CAR_HALF_WIDTH 0.9
#define TRUCK_HALF_WIDTH 1.25
#define BODY_MARGIN 0.15
#define BODY_REACH 15.0

typedef struct Track
{
    uint64_t serial;
    uint32_t path;
    int32_t speed;
    /* The slowest the car went within reach of its line. */
    int32_t lineSpeed;
    /* Stood at its line with nothing in its way, counted once per stop. */
    bool idle;
    /* Came up to its line with right of way, counted once per lane. */
    bool arrived;
    /* Braked hard for a car giving way, counted once per path. */
    bool forced;
} Track;

typedef struct Tally
{
    uint64_t redEntries;
    uint64_t stopsSkipped;
    uint64_t collisions;
    uint64_t forcedBrakes;
    uint64_t idleStops;
    /* Ticks two cars' bodies overlapped, and a truck's with another's. */
    uint64_t bodies;
    uint64_t truckBodies;
    uint64_t priorityArrivals;
    uint64_t entries;
} Tally;

static TestRouter router;

static const FoxPath *Path(const FoxNetwork *network, uint32_t path)
{
    return FoxNetworkPathRecord(network, path);
}

static uint32_t LaneEnd(const FoxPath *lane)
{
    return lane->direction == 0u ? 1u : 0u;
}

/* The conflict of connection `a` with `b`, or NULL. */
static const FoxConflict *ConflictOf(const FoxPath *a, uint32_t b)
{
    for (uint32_t index = 0u; index < a->conflictCount; ++index)
    {
        if (a->conflicts[index].path == b)
        {
            return &a->conflicts[index];
        }
    }
    return NULL;
}

/* Whether a vehicle's body covers the point `at` along the path it is on. */
static bool Covers(const FoxVehicleView *vehicle, int64_t at)
{
    return vehicle->distance + TOUCH > at && vehicle->distance - vehicle->length - TOUCH < at;
}

/* Whether a car that should give way to `priority` is on its way across
 * it, up to the point where the two meet and not yet past it; one waiting
 * short of it is out of the way. */
static bool YielderCrossing(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t priority)
{
    const FoxPath *way = Path(network, priority);
    for (uint32_t index = 0u; index < way->conflictCount; ++index)
    {
        const FoxConflict *conflict = &way->conflicts[index];
        const FoxConflict *back = ConflictOf(Path(network, conflict->path), priority);
        if (conflict->yields || back == NULL || !back->yields)
        {
            continue;
        }
        for (uint32_t slot = 0u; slot < FoxTrafficCapacity(traffic); ++slot)
        {
            FoxVehicleView vehicle;
            if (FoxTrafficVehicle(traffic, slot, &vehicle) && vehicle.path == conflict->path &&
                vehicle.distance + WAITING_SHORT > conflict->otherAt &&
                vehicle.distance - vehicle.length < conflict->otherAt + TOUCH)
            {
                return true;
            }
        }
    }
    return false;
}

/* The light for a way through an intersection: its arrow while that shows,
 * else the light of its lane. */
static FoxLight LightFor(const FoxNetwork *network, uint32_t way)
{
    const FoxPath *connection = Path(network, way);
    const FoxPath *lane = Path(network, connection->from);
    const FoxLight arrow = connection->movement == MOVE_FAR
                               ? FoxNetworkArrow(network, lane->road, LaneEnd(lane))
                               : FOX_LIGHT_NONE;
    return arrow != FOX_LIGHT_NONE ? arrow : FoxNetworkLight(network, lane->road, LaneEnd(lane));
}

/* Whether the light at the start of a way lets its cars on. */
static bool Moving(const FoxNetwork *network, uint32_t way)
{
    const FoxLight light = LightFor(network, way);
    return light == FOX_LIGHT_NONE || light == FOX_LIGHT_GREEN;
}

/* Whether nothing keeps a vehicle at the front of its lane from taking its
 * way: the way is clear and the lane beyond has room for it and more. */
static bool NothingAhead(const FoxTraffic *traffic, const FoxNetwork *network, const FoxPath *lane,
                         uint32_t connectionIndex, int64_t length)
{
    for (uint32_t index = 0u; index < lane->outgoingCount; ++index)
    {
        const FoxPath *connection = Path(network, lane->outgoing[index]);
        if (connection->vehicleCount != 0u)
        {
            return false;
        }
    }
    const FoxPath *beyond = Path(network, Path(network, connectionIndex)->to);
    if (beyond->incoming != 0u)
    {
        /* Cars from elsewhere are on their way onto it. */
        return false;
    }
    if (beyond->back == FOX_NETWORK_NONE)
    {
        return true;
    }
    FoxVehicleView last;
    EXPECT(FoxTrafficVehicle(traffic, beyond->back, &last), "the last car beyond is alive");
    return last.distance - last.length > length + CLEAR_AHEAD;
}

/* A vehicle's body in the plane: its middle, the direction it faces and
 * the one across it, its half length and half width, in metres. */
typedef struct Body
{
    double middle[3];
    double forward[3];
    double across[3];
    double half;
    double side;
    bool truck;
} Body;

static double Dot(const double *a, const double *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Body BodyOf(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t slot,
                   const FoxVehicleView *view)
{
    FoxVec position;
    FoxVec forward;
    EXPECT(FoxTrafficVehiclePose(traffic, network, slot, FOX_UNIT, &position, &forward),
           "every car has a pose");
    const FoxVec up = FoxPlanetUp(position);
    const FoxVec across = FoxVecScale(FoxVecCross(forward, up), 1, FOX_UNIT);
    const bool truck = view->kind == FOX_VEHICLE_TRUCK;
    return (Body){
        .middle = {(double)position.x / FOX_UNITS_PER_METRE,
                   (double)position.y / FOX_UNITS_PER_METRE,
                   (double)position.z / FOX_UNITS_PER_METRE},
        .forward = {(double)forward.x / FOX_UNIT, (double)forward.y / FOX_UNIT,
                    (double)forward.z / FOX_UNIT},
        .across = {(double)across.x / FOX_UNIT, (double)across.y / FOX_UNIT,
                   (double)across.z / FOX_UNIT},
        .half = (double)view->length / FOX_UNITS_PER_METRE / 2.0,
        .side = truck ? TRUCK_HALF_WIDTH : CAR_HALF_WIDTH,
        .truck = truck,
    };
}

/* Whether two bodies overlap, by their separating axes. */
static bool Overlap(const Body *a, const Body *b)
{
    const double between[3] = {b->middle[0] - a->middle[0], b->middle[1] - a->middle[1],
                               b->middle[2] - a->middle[2]};
    if (Dot(between, between) > BODY_REACH * BODY_REACH)
    {
        return false;
    }
    const double *axes[4] = {a->forward, a->across, b->forward, b->across};
    for (uint32_t index = 0u; index < 4u; ++index)
    {
        const double *axis = axes[index];
        const double reach = fabs(Dot(a->forward, axis)) * (a->half - BODY_MARGIN) +
                             fabs(Dot(a->across, axis)) * (a->side - BODY_MARGIN) +
                             fabs(Dot(b->forward, axis)) * (b->half - BODY_MARGIN) +
                             fabs(Dot(b->across, axis)) * (b->side - BODY_MARGIN);
        if (fabs(Dot(between, axis)) > reach)
        {
            return false;
        }
    }
    return true;
}

static int CompareBodies(const void *left, const void *right)
{
    const double a = ((const Body *)left)->middle[0];
    const double b = ((const Body *)right)->middle[0];
    return (a > b) - (a < b);
}

/* Counts the ticks two bodies overlap, comparing those near each other. */
static void WatchBodies(const FoxTraffic *traffic, const FoxNetwork *network, Body *bodies,
                        Tally *tally)
{
    uint32_t count = 0u;
    for (uint32_t slot = 0u; slot < FoxTrafficCapacity(traffic); ++slot)
    {
        FoxVehicleView view;
        if (FoxTrafficVehicle(traffic, slot, &view))
        {
            bodies[count++] = BodyOf(traffic, network, slot, &view);
        }
    }
    qsort(bodies, count, sizeof(*bodies), CompareBodies);
    for (uint32_t a = 0u; a < count; ++a)
    {
        for (uint32_t b = a + 1u;
             b < count && bodies[b].middle[0] - bodies[a].middle[0] < BODY_REACH; ++b)
        {
            if (Overlap(&bodies[a], &bodies[b]))
            {
                if (bodies[a].truck || bodies[b].truck)
                {
                    ++tally->truckBodies;
                }
                else
                {
                    ++tally->bodies;
                }
            }
        }
    }
}

static void Watch(const FoxWorld *world, Track *tracks, Tally *tally)
{
    const FoxTraffic *traffic = FoxWorldTraffic(world);
    const FoxNetwork *network = FoxWorldNetwork(world);
    const uint32_t count = FoxTrafficCapacity(traffic);
    EXPECT(count <= SLOTS, "the watch has a track for every slot");
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        FoxVehicleView vehicle;
        Track *track = &tracks[slot];
        if (!FoxTrafficVehicle(traffic, slot, &vehicle))
        {
            track->serial = 0u;
            continue;
        }
        const FoxPath *path = Path(network, vehicle.path);
        const bool known = track->serial == vehicle.serial;
        if (known && track->path != vehicle.path)
        {
            const FoxPath *left = Path(network, track->path);
            if (left->kind == PATH_LANE && path->kind == PATH_CONNECTION)
            {
                ++tally->entries;
                const FoxLight light = LightFor(network, vehicle.path);
                tally->redEntries += light == FOX_LIGHT_RED ? 1u : 0u;
                const FoxSign sign = (FoxSign)network->roads[left->road].sign[LaneEnd(left)];
                tally->stopsSkipped +=
                    light == FOX_LIGHT_NONE && sign == FOX_SIGN_STOP && track->lineSpeed > STANDING
                        ? 1u
                        : 0u;
            }
            track->lineSpeed = INT32_MAX;
            track->idle = false;
            track->arrived = false;
            track->forced = false;
        }
        if (!known)
        {
            *track = (Track){.serial = vehicle.serial, .lineSpeed = INT32_MAX};
        }
        else if ((int64_t)(track->speed - vehicle.speed) * TICKS > HARD_BRAKE)
        {
            /* Hard braking with right of way, for a car that should have
             * waited; not for a light turning yellow. */
            const uint32_t way = vehicle.next;
            if (!track->forced && way != FOX_NETWORK_NONE && Moving(network, way) &&
                YielderCrossing(traffic, network, way))
            {
                track->forced = true;
                ++tally->forcedBrakes;
            }
        }
        if (path->kind == PATH_LANE)
        {
            const int64_t toLine = path->length - vehicle.distance;
            if (toLine <= STOP_REACH && vehicle.speed < track->lineSpeed)
            {
                track->lineSpeed = vehicle.speed;
            }
            const FoxLight light = FoxNetworkLight(network, path->road, LaneEnd(path));
            const FoxSign sign = (FoxSign)network->roads[path->road].sign[LaneEnd(path)];
            const FoxPath *next =
                vehicle.next != FOX_NETWORK_NONE ? Path(network, vehicle.next) : NULL;
            const bool priority =
                next != NULL && (next->movement == MOVE_STRAIGHT || next->movement == MOVE_CURB) &&
                (light == FOX_LIGHT_GREEN ||
                 (light == FOX_LIGHT_NONE && sign == FOX_SIGN_PRIORITY));
            if (priority && path->front == slot && toLine <= AT_LINE)
            {
                tally->priorityArrivals += track->arrived ? 0u : 1u;
                track->arrived = true;
                if (!track->idle && known && track->speed > STANDING && vehicle.speed <= STANDING &&
                    NothingAhead(traffic, network, path, vehicle.next, vehicle.length))
                {
                    track->idle = true;
                    ++tally->idleStops;
                }
            }
        }
        track->path = vehicle.path;
        track->speed = vehicle.speed;
    }
    /* Cars in the same place where their ways cross. */
    FoxVehicleView inside[SLOTS];
    uint32_t insideCount = 0u;
    for (uint32_t slot = 0u; slot < count; ++slot)
    {
        if (FoxTrafficVehicle(traffic, slot, &inside[insideCount]) &&
            Path(network, inside[insideCount].path)->kind == PATH_CONNECTION)
        {
            ++insideCount;
        }
    }
    for (uint32_t a = 0u; a < insideCount; ++a)
    {
        for (uint32_t b = a + 1u; b < insideCount; ++b)
        {
            const FoxConflict *onB = ConflictOf(Path(network, inside[a].path), inside[b].path);
            const FoxConflict *onA = ConflictOf(Path(network, inside[b].path), inside[a].path);
            if (onB != NULL && onA != NULL && Covers(&inside[a], onA->otherAt) &&
                Covers(&inside[b], onB->otherAt))
            {
                ++tally->collisions;
            }
        }
    }
}

/* A street all the way round the planet's equator out in the country, in
 * six pieces each bent through where the circle's tangents at its ends
 * meet: long stretches without an intersection where cars catch up with
 * trucks and overtake them. */
static bool BuildCountryRoad(FoxWorld *world)
{
    const double radius = (double)FoxPlanetRadius(FoxWorldPlanet(world));
    const double step = 2.0 * acos(-1.0) / 6.0;
    bool ok = true;
    for (int piece = 0; piece < 6; ++piece)
    {
        const double from = step * piece;
        const double middle = from + step / 2.0;
        const double out = radius / cos(step / 2.0);
        const FoxRoadPlan plan = {
            .start = {(int64_t)llround(radius * cos(from)), (int64_t)llround(radius * sin(from)),
                      0},
            .end = {(int64_t)llround(radius * cos(from + step)),
                    (int64_t)llround(radius * sin(from + step)), 0},
            .control = {(int64_t)llround(out * cos(middle)), (int64_t)llround(out * sin(middle)),
                        0},
            .curved = true,
            .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET),
        };
        ok = FoxWorldBuildRoad(world, &plan) == FOX_ROAD_OK && ok;
    }
    return ok;
}

/* Where a run takes place. */
typedef enum Place
{
    PLACE_TOWN,
    PLACE_PLANET,
    PLACE_COUNTRY,
} Place;

static void Run(const char *label, FoxDriveSide side, uint32_t cars, bool routed, Place place,
                uint32_t seconds)
{
    TestRouterInit(&router);
    const FoxWorldDesc desc = {.seed = 11u,
                               .driveSide = side,
                               .targetVehicles = cars,
                               .router = routed ? &router.router : NULL};
    FoxWorld *world = FoxWorldCreate(&desc);
    EXPECT(world != NULL, "the world is created");
    EXPECT(place == PLACE_COUNTRY ? BuildCountryRoad(world) : FoxWorldBuildDemoTown(world),
           "the roads are built");
    EXPECT(place != PLACE_PLANET || FoxWorldBuildPlanetRoads(world),
           "the roads round the planet are built");
    Track *tracks = calloc(SLOTS, sizeof(*tracks));
    Body *bodies = calloc(SLOTS, sizeof(*bodies));
    EXPECT(tracks != NULL && bodies != NULL, "tracks for every car");
    Tally tally = {0};
    for (uint64_t tick = 0u; tick < SECONDS(seconds); ++tick)
    {
        FoxWorldAdvance(world, 1u);
        Watch(world, tracks, &tally);
        WatchBodies(FoxWorldTraffic(world), FoxWorldNetwork(world), bodies, &tally);
    }
    const FoxTrafficStats stats = FoxTrafficGetStats(FoxWorldTraffic(world));
    printf("%s: %llu entries, %llu on red, %llu past stop signs, %llu collisions, %llu forced "
           "brakes, %llu of %llu arrivals with right of way stood idle; %llu arrived, %u standing "
           "long, "
           "mean %.1f m/s; %llu lane changes, %llu overtakes; bodies overlapping %llu ticks, with "
           "a truck %llu\n",
           label, (unsigned long long)tally.entries, (unsigned long long)tally.redEntries,
           (unsigned long long)tally.stopsSkipped, (unsigned long long)tally.collisions,
           (unsigned long long)tally.forcedBrakes, (unsigned long long)tally.idleStops,
           (unsigned long long)tally.priorityArrivals, (unsigned long long)stats.arrived,
           stats.waitingLong, (double)stats.meanSpeed / FOX_UNITS_PER_METRE,
           (unsigned long long)stats.laneChanges, (unsigned long long)stats.overtakes,
           (unsigned long long)tally.bodies, (unsigned long long)tally.truckBodies);
    EXPECT(tally.redEntries == 0u, "nobody drives into a red light");
    EXPECT(tally.stopsSkipped == 0u, "everyone stops at a stop sign");
    EXPECT(tally.collisions == 0u, "no two cars meet where their ways cross");
    EXPECT(tally.forcedBrakes == 0u, "nobody with right of way brakes hard for one giving way");
    /* A driver out of patience may still squeeze in ahead of one that can
     * stop for it comfortably; that is rare. */
    EXPECT(tally.idleStops * 100u <= tally.priorityArrivals,
           "a car with right of way and room goes but for a rare driver squeezing in");
    EXPECT(tally.bodies == 0u, "no two cars' bodies ever overlap");
    EXPECT(place != PLACE_PLANET || stats.laneChanges != 0u,
           "drivers change lanes where roads have more of them");
    EXPECT(place != PLACE_COUNTRY || stats.overtakes != 0u,
           "drivers overtake slow trucks on the open road");
    free(bodies);
    free(tracks);
    FoxWorldDestroy(world);
}

int main(void)
{
    Run("keep right, routed", FOX_DRIVE_RIGHT, 220u, true, PLACE_TOWN, 300u);
    Run("keep left, random turns", FOX_DRIVE_LEFT, 160u, false, PLACE_TOWN, 180u);
    Run("round the planet", FOX_DRIVE_RIGHT, 400u, true, PLACE_PLANET, 300u);
    Run("country road", FOX_DRIVE_RIGHT, 30u, false, PLACE_COUNTRY, 300u);
    puts("rules: ok");
    return 0;
}
