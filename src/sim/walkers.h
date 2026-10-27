#ifndef FOX_SIM_WALKERS_H
#define FOX_SIM_WALKERS_H

#include "sim/network.h"
#include "sim/space.h"
#include "sim/traffic.h"

#include <stdbool.h>
#include <stdint.h>

/* The residents walking the footways.
 *
 * Each walks a footway either way at a pace of its own, round the corners of
 * the intersections onto the next footway, on at random where ways part, and
 * back where a footway ends. At a pedestrian crossing it may go over: it
 * waits at the kerb until every car coming could still stop short of it
 * comfortably, steps out, and the cars wait for it. Where the city wants
 * fewer, walkers go home at the end of a footway.
 *
 * Integer arithmetic at the traffic's tick rate, like the rest of the
 * simulation. */

#define FOX_WALKERS_MAX 1000000u

typedef struct FoxWalkers FoxWalkers;

typedef struct FoxWalkersDesc
{
    uint64_t seed;
    /* How many walkers the city tries to keep out walking. */
    uint32_t target;
} FoxWalkersDesc;

typedef enum FoxWalkerState
{
    FOX_WALKER_WALKING = 0,
    /* At the kerb, waiting for the cars to let it over. */
    FOX_WALKER_WAITING,
    FOX_WALKER_CROSSING,
} FoxWalkerState;

typedef struct FoxWalkerView
{
    uint64_t serial;
    uint32_t path;
    int64_t distance;
    FoxWalkerState state;
    uint8_t color;
} FoxWalkerView;

typedef struct FoxWalkersStats
{
    uint32_t walkers;
    uint32_t waiting;
    uint32_t crossing;
    /* Crossings walked over so far. */
    uint64_t crossed;
} FoxWalkersStats;

FoxWalkers *FoxWalkersCreate(const FoxWalkersDesc *desc);
void FoxWalkersDestroy(FoxWalkers *walkers);

void FoxWalkersSetTarget(FoxWalkers *walkers, uint32_t target);
uint32_t FoxWalkersTarget(const FoxWalkers *walkers);

/* Moves every walker by one tick; the cars are read to cross by. */
void FoxWalkersStep(FoxWalkers *walkers, FoxNetwork *network, const FoxTraffic *traffic);
/* Moves walkers off the walks a network change retired onto the nearest
 * live footway, before the network releases them. */
void FoxWalkersRemap(FoxWalkers *walkers, FoxNetwork *network);

/* Walkers live in slots; iterate up to the capacity. */
uint32_t FoxWalkersCapacity(const FoxWalkers *walkers);
bool FoxWalkersWalker(const FoxWalkers *walkers, uint32_t slot, FoxWalkerView *outView);
/* Where a walker is drawn, between its last two ticks by `blend` (0 to
 * FOX_UNIT), keeping a little to the right of its way, and which way it
 * faces (FOX_UNIT long). */
bool FoxWalkersPose(const FoxWalkers *walkers, const FoxNetwork *network, uint32_t slot,
                    int64_t blend, FoxVec *outPosition, FoxVec *outFacing);

FoxWalkersStats FoxWalkersGetStats(const FoxWalkers *walkers);
uint64_t FoxWalkersHash(const FoxWalkers *walkers);

#endif
