#ifndef FOX_APP_CITY_H
#define FOX_APP_CITY_H

#include "app/modules.h"
#include "app/navigation_router.h"
#include "sim/world.h"

#include <stdbool.h>
#include <stdint.h>

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
    /* Builds the demo town right away. */
    bool demo;
    /* Loaded engine modules to take routing from; NULL turns cars at random. */
    const FoxModules *modules;
} FoxCityDesc;

/* False when the world could not be created; nothing is left to destroy. */
bool FoxCityCreate(FoxCity *city, const FoxCityDesc *desc);
void FoxCityDestroy(FoxCity *city);

/* Logs the tick, the roads, the cars and the state hash. */
void FoxCityReport(const FoxCity *city, const char *event);

#endif
