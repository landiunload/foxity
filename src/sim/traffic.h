#ifndef FOX_SIM_TRAFFIC_H
#define FOX_SIM_TRAFFIC_H

#include "sim/network.h"
#include "sim/space.h"

#include <stdbool.h>
#include <stdint.h>

/* Cars on the road network.
 *
 * Every car drives a lane, keeps its distance with the intelligent driver
 * model and slows for turns. Each intersection hands out its way in once per
 * tick: in the order cars came, a bigger road and going straight counting as
 * having come earlier, a car may go when no turn crossing its own is claimed
 * or given to a car before it, and when the lane beyond has room. A car held up
 * keeps its turn, so nobody waits forever and no intersection locks itself.
 * Beyond a lane too short to stop on, the next intersection is claimed at the
 * same time. A car standing for a minute is taken off the road, as city
 * builders do, so an overloaded network cannot stay knotted. Cars take the
 * fastest route the router knows; measured speeds feed back into the routing
 * costs, so jams push traffic onto other roads.
 *
 * All of it is integer arithmetic at a fixed tick rate: the same seed,
 * network and commands give the same traffic on every machine. */

#define FOX_TRAFFIC_TICKS_PER_SECOND 30
#define FOX_VEHICLE_LENGTH (FOX_METRES(9) / 2)
#define FOX_TRAFFIC_MAX_VEHICLES 1000000u

typedef struct FoxTraffic FoxTraffic;

typedef struct FoxTrafficDesc
{
    uint64_t seed;
    /* How many cars the city tries to keep on its roads. */
    uint32_t targetVehicles;
} FoxTrafficDesc;

typedef struct FoxVehicleView
{
    uint64_t serial;
    uint32_t path;
    int64_t distance;
    /* Units per second. */
    int32_t speed;
    uint8_t color;
    /* Ticks spent standing still. */
    uint32_t waitTicks;
} FoxVehicleView;

typedef struct FoxTrafficStats
{
    uint32_t vehicles;
    uint32_t standing;
    uint64_t spawned;
    uint64_t arrived;
    uint64_t removed;
    /* Of the removed: cars taken off after standing a minute. */
    uint64_t stuck;
    /* Mean speed of all cars, units per second. */
    int32_t meanSpeed;
} FoxTrafficStats;

FoxTraffic *FoxTrafficCreate(const FoxTrafficDesc *desc);
void FoxTrafficDestroy(FoxTraffic *traffic);

void FoxTrafficSetTarget(FoxTraffic *traffic, uint32_t vehicles);
uint32_t FoxTrafficTarget(const FoxTraffic *traffic);

/* Advances every car by one tick. */
void FoxTrafficStep(FoxTraffic *traffic, FoxNetwork *network);

/* Moves cars off the paths a network change retired onto their successors,
 * or takes them off the road where there is none. Call after every change
 * and before FoxNetworkReleaseRetired. */
void FoxTrafficRemap(FoxTraffic *traffic, FoxNetwork *network);

/* Cars live in slots; iterate up to the capacity. */
uint32_t FoxTrafficCapacity(const FoxTraffic *traffic);
bool FoxTrafficVehicle(const FoxTraffic *traffic, uint32_t slot, FoxVehicleView *outView);

/* Where a car is drawn: between its last two ticks by `blend` (0..FOX_UNIT),
 * with the direction it faces (FOX_UNIT long). */
bool FoxTrafficVehiclePose(const FoxTraffic *traffic, const FoxNetwork *network, uint32_t slot,
                           int64_t blend, FoxVec *outPosition, FoxVec *outForward);

FoxTrafficStats FoxTrafficGetStats(const FoxTraffic *traffic);
uint64_t FoxTrafficHash(const FoxTraffic *traffic);

#endif
