#include "app/city.h"

#include "app/log.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

bool FoxCityCreate(FoxCity *city, const FoxCityDesc *desc)
{
    if (city == NULL || desc == NULL)
    {
        return false;
    }
    memset(city, 0, sizeof(*city));
    if (desc->modules != NULL)
    {
        uint32_t size = 0u;
        const LaiueNavigationServiceV1 *service = (const LaiueNavigationServiceV1 *)FoxModulesQuery(
            desc->modules, LAIUE_NAVIGATION_SERVICE_NAME, LAIUE_NAVIGATION_SERVICE_ABI_VERSION_1,
            (uint32_t)sizeof(*service), &size);
        city->routed = FoxNavigationRouterInit(&city->navigation, service, size);
    }
    if (!city->routed)
    {
        FoxLog(FOX_LOG_WARNING, "navigation is unavailable; cars turn at random");
    }
    const FoxWorldDesc worldDesc = {
        .seed = desc->seed,
        .driveSide = desc->driveSide,
        .targetVehicles = desc->cars,
        .router = city->routed ? &city->navigation.router : NULL,
    };
    city->world = FoxWorldCreate(&worldDesc);
    if (city->world == NULL)
    {
        FoxNavigationRouterDestroy(&city->navigation);
        return false;
    }
    if (desc->demo && !FoxWorldBuildDemoTown(city->world))
    {
        FoxLog(FOX_LOG_WARNING, "part of the demo town could not be built");
    }
    return true;
}

void FoxCityDestroy(FoxCity *city)
{
    if (city == NULL)
    {
        return;
    }
    FoxWorldDestroy(city->world);
    if (city->routed)
    {
        FoxNavigationRouterDestroy(&city->navigation);
    }
    memset(city, 0, sizeof(*city));
}

void FoxCityReport(const FoxCity *city, const char *event)
{
    if (city == NULL || city->world == NULL || event == NULL)
    {
        return;
    }
    const FoxNetwork *network = FoxWorldNetwork(city->world);
    const FoxTrafficStats stats = FoxTrafficGetStats(FoxWorldTraffic(city->world));
    FoxLog(FOX_LOG_INFO,
           "%s at tick %" PRIu64 ": %u roads, %u nodes, %u cars (%u standing, mean %.1f m/s), "
           "%" PRIu64 " arrived, %" PRIu64 " taken off stuck, state %016" PRIx64,
           event, FoxWorldTick(city->world), FoxNetworkRoadCount(network),
           FoxNetworkNodeCount(network), stats.vehicles, stats.standing,
           (double)stats.meanSpeed / FOX_UNITS_PER_METRE, stats.arrived, stats.stuck,
           FoxWorldHash(city->world));
}
