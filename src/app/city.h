#ifndef FOX_APP_CITY_H
#define FOX_APP_CITY_H

#include "app/modules.h"
#include "app/navigation_router.h"
#include "sim/world.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* A city with the engine technology it runs on: the world and, where the
 * engine has laiue.navigation, the router its cars take routes from. Shared
 * by the headless run and the client. */
typedef struct FoxCity
{
    FoxWorld *world;
    FoxNavigationRouter navigation;
    bool routed;
} FoxCity;

typedef struct FoxCityDesc
{
    uint64_t seed;
    FoxDriveSide driveSide;
    uint32_t cars;
    /* People out walking the footways. */
    uint32_t walkers;
    /* Builds the demo town right away, or with gridLines a square grid of
     * that many roads each way gridSpacing apart. */
    bool demo;
    uint32_t gridLines;
    int64_t gridSpacing;
    /* Loaded engine modules to take routing from; NULL turns cars at random. */
    const FoxModules *modules;
} FoxCityDesc;

/* False when the world could not be created; nothing is left to destroy. */
bool FoxCityCreate(FoxCity *city, const FoxCityDesc *desc);
void FoxCityDestroy(FoxCity *city);

/* Logs the tick, the roads, the cars and the state hash. */
void FoxCityReport(const FoxCity *city, const char *event);

#define FOX_TICK_TIMER_BUCKETS 32u

/* How long ticks took: a histogram by powers of two of microseconds. */
typedef struct FoxTickTimer
{
    uint64_t ticks;
    double seconds;
    double slowest;
    uint64_t buckets[FOX_TICK_TIMER_BUCKETS];
} FoxTickTimer;

void FoxTickTimerRecord(FoxTickTimer *timer, double seconds);

/* Writes everything known about the city to `stream`: the network, the
 * traffic and, given a timer, how fast it ran. */
void FoxCityPrintStatistics(const FoxCity *city, const FoxTickTimer *timer, FILE *stream);

#endif
