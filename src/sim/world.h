#ifndef FOX_SIM_WORLD_H
#define FOX_SIM_WORLD_H

#include "sim/network.h"
#include "sim/planet.h"
#include "sim/router.h"
#include "sim/traffic.h"

#include <stdbool.h>
#include <stdint.h>

/* The city: the planet, its roads and its cars, advanced in fixed ticks.
 * Every change goes through the commands below, so a recorded list of
 * commands with their ticks replays the same city anywhere. */

typedef struct FoxWorld FoxWorld;

typedef struct FoxWorldDesc
{
    uint64_t seed;
    /* The city's own drive side, chosen when it is founded; zones can
     * override it locally later. */
    FoxDriveSide driveSide;
    /* Zero picks FOX_PLANET_DEFAULT_RADIUS. */
    int64_t planetRadius;
    uint32_t targetVehicles;
    /* Optional, see router.h; must outlive the world. */
    const FoxRouter *router;
} FoxWorldDesc;

FoxWorld *FoxWorldCreate(const FoxWorldDesc *desc);
void FoxWorldDestroy(FoxWorld *world);

uint64_t FoxWorldTick(const FoxWorld *world);
void FoxWorldAdvance(FoxWorld *world, uint64_t ticks);

const FoxPlanet *FoxWorldPlanet(const FoxWorld *world);
const FoxNetwork *FoxWorldNetwork(const FoxWorld *world);
const FoxTraffic *FoxWorldTraffic(const FoxWorld *world);

uint32_t FoxWorldDig(FoxWorld *world, FoxVec center, int64_t radius, FoxPlanetBox *outChanged);
uint32_t FoxWorldFill(FoxWorld *world, FoxVec center, int64_t radius, FoxPlanetBox *outChanged);
FoxRoadResult FoxWorldBuildRoad(FoxWorld *world, const FoxRoadPlan *plan);
bool FoxWorldRemoveRoad(FoxWorld *world, uint32_t road);
bool FoxWorldAddDriveZone(FoxWorld *world, FoxVec center, int64_t radius, FoxDriveSide side);
bool FoxWorldSetSpeedLimit(FoxWorld *world, uint32_t road, int32_t speed);
bool FoxWorldSetSign(FoxWorld *world, uint32_t road, uint32_t end, FoxSign sign);
bool FoxWorldSetSignals(FoxWorld *world, uint32_t node, bool on);
void FoxWorldSetTargetVehicles(FoxWorld *world, uint32_t vehicles);

/* A small grid town around the north pole: what the headless run, the
 * tests and a client started with --demo drive on. The avenues cross at
 * traffic lights and a quiet corner has stop signs. False if a road could
 * not be built. */
bool FoxWorldBuildDemoTown(FoxWorld *world);

/* Roads round the whole planet beyond the demo town: rings of a street,
 * an avenue, a boulevard and a one-way highway at different latitudes,
 * crossed by three meridians that run from the town's avenues down to the
 * south pole. Lights stand where big two-way roads cross. False if a road
 * could not be built. */
bool FoxWorldBuildPlanetRoads(FoxWorld *world);

/* A square grid around the north pole for loading the city: `lines` roads
 * each way `spacing` apart. Every fourth line is an avenue, the middle ones
 * boulevards, and lights stand where those cross. False if a road could not
 * be built. */
bool FoxWorldBuildGrid(FoxWorld *world, uint32_t lines, int64_t spacing);

/* One value for the whole state: equal on every machine that ran the same
 * commands. */
uint64_t FoxWorldHash(const FoxWorld *world);

#endif
