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
    if (desc->gridLines != 0u)
    {
        if (!FoxWorldBuildGrid(city->world, desc->gridLines, desc->gridSpacing))
        {
            FoxLog(FOX_LOG_WARNING, "part of the grid could not be built");
        }
    }
    else if (desc->demo &&
             (!FoxWorldBuildDemoTown(city->world) || !FoxWorldBuildPlanetRoads(city->world)))
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

void FoxTickTimerRecord(FoxTickTimer *timer, double seconds)
{
    if (timer == NULL || !(seconds >= 0.0))
    {
        return;
    }
    ++timer->ticks;
    timer->seconds += seconds;
    timer->slowest = seconds > timer->slowest ? seconds : timer->slowest;
    uint32_t bucket = 0u;
    for (uint64_t micro = (uint64_t)(seconds * 1e6);
         micro >= 2u && bucket + 1u < FOX_TICK_TIMER_BUCKETS; micro /= 2u)
    {
        ++bucket;
    }
    ++timer->buckets[bucket];
}

/* The upper bound of the bucket holding a share of the ticks, ms. */
static double Percentile(const FoxTickTimer *timer, double share)
{
    const uint64_t wanted = (uint64_t)((double)timer->ticks * share);
    uint64_t seen = 0u;
    for (uint32_t bucket = 0u; bucket < FOX_TICK_TIMER_BUCKETS; ++bucket)
    {
        seen += timer->buckets[bucket];
        if (seen > wanted)
        {
            return (double)(UINT64_C(2) << bucket) / 1000.0;
        }
    }
    return timer->slowest * 1000.0;
}

static double Kmh(int64_t units)
{
    return (double)units * 3.6 / FOX_UNITS_PER_METRE;
}

void FoxCityPrintStatistics(const FoxCity *city, const FoxTickTimer *timer, FILE *stream)
{
    if (city == NULL || city->world == NULL || stream == NULL)
    {
        return;
    }
    const FoxNetwork *network = FoxWorldNetwork(city->world);
    const FoxTraffic *traffic = FoxWorldTraffic(city->world);
    const FoxNetworkStats roads = FoxNetworkGetStats(network);
    const FoxTrafficStats cars = FoxTrafficGetStats(traffic);
    const uint64_t tick = FoxWorldTick(city->world);
    const double cityMinutes = (double)tick / FOX_TRAFFIC_TICKS_PER_SECOND / 60.0;
    fprintf(stream, "== city at tick %" PRIu64 " (%.1f min), state %016" PRIx64 "\n", tick,
            cityMinutes, FoxWorldHash(city->world));
    fprintf(stream, "routing: %s\n", city->routed ? "laiue.navigation" : "random turns");

    fprintf(stream, "-- roads\n");
    fprintf(stream, "roads %u, %.2f km of road, %.2f km of lanes in %u lanes\n", roads.roads,
            (double)roads.roadLength / FOX_METRES(1000),
            (double)roads.laneLength / FOX_METRES(1000), roads.lanes);
    for (uint32_t type = 0u; type < FOX_ROAD_TYPE_COUNT; ++type)
    {
        const FoxRoadTypeInfo *info = FoxRoadTypeGet((FoxRoadType)type);
        fprintf(stream, "  %-15s %5u  (%.0f km/h)\n", info->name, roads.roadsByType[type],
                Kmh(info->speedLimit));
    }
    fprintf(stream, "nodes %u: %u junctions (%u with lights), %u bends, %u dead ends\n",
            roads.nodes, roads.junctions, roads.signalled, roads.bends, roads.deadEnds);
    fprintf(stream, "signs at junctions: %u priority, %u give way, %u stop, %u none\n",
            roads.signs[FOX_SIGN_PRIORITY], roads.signs[FOX_SIGN_YIELD], roads.signs[FOX_SIGN_STOP],
            roads.signs[FOX_SIGN_NONE]);
    fprintf(stream, "speed limits set by hand: %u\n", roads.speedLimitsSet);
    fprintf(stream, "turns through junctions %u, conflicting pairs %u\n", roads.connections,
            roads.conflicts);

    fprintf(stream, "-- traffic\n");
    fprintf(stream, "vehicles %u (aim %u): %u cars, %u trucks\n", cars.vehicles,
            FoxTrafficTarget(traffic), cars.vehicles - cars.trucks, cars.trucks);
    fprintf(stream, "moving %u, standing %u (%.0f %%), standing over 30 s %u\n",
            cars.vehicles - cars.standing, cars.standing,
            cars.vehicles != 0u ? 100.0 * cars.standing / cars.vehicles : 0.0, cars.waitingLong);
    fprintf(stream, "speed: mean %.1f km/h, fastest %.1f km/h\n", Kmh(cars.meanSpeed),
            Kmh(cars.maxSpeed));
    /* How fast they go, in steps of 10 km/h. */
    uint32_t bands[14] = {0u};
    const uint32_t capacity = FoxTrafficCapacity(traffic);
    for (uint32_t slot = 0u; slot < capacity; ++slot)
    {
        FoxVehicleView vehicle;
        if (FoxTrafficVehicle(traffic, slot, &vehicle))
        {
            const uint32_t band = (uint32_t)(Kmh(vehicle.speed) / 10.0);
            ++bands[band < 13u ? band : 13u];
        }
    }
    fprintf(stream, "speeds:");
    for (uint32_t band = 0u; band < 14u; ++band)
    {
        if (bands[band] != 0u)
        {
            fprintf(stream, " %u-%u:%u", band * 10u, band * 10u + 10u, bands[band]);
        }
    }
    fprintf(stream, " km/h\n");
    fprintf(stream,
            "spawned %" PRIu64 ", arrived %" PRIu64 ", removed %" PRIu64
            ", %u standing half a minute or more\n",
            cars.spawned, cars.arrived, cars.removed, cars.waitingLong);
    if (cityMinutes > 0.0)
    {
        fprintf(stream, "per city minute: %.1f arrived\n", (double)cars.arrived / cityMinutes);
    }

    if (timer != NULL && timer->ticks != 0u)
    {
        const double mean = timer->seconds / (double)timer->ticks;
        fprintf(stream, "-- performance\n");
        fprintf(stream, "ticks %" PRIu64 " in %.3f s: %.0f ticks/s, %.0fx real time\n",
                timer->ticks, timer->seconds, (double)timer->ticks / timer->seconds,
                (double)timer->ticks / timer->seconds / FOX_TRAFFIC_TICKS_PER_SECOND);
        fprintf(stream, "tick: mean %.3f ms, p50 <= %.3f ms, p99 <= %.3f ms, slowest %.3f ms\n",
                mean * 1000.0, Percentile(timer, 0.5), Percentile(timer, 0.99),
                timer->slowest * 1000.0);
        fprintf(stream, "budget used at 1x: %.2f %% of a %.1f ms tick\n",
                100.0 * mean * FOX_TRAFFIC_TICKS_PER_SECOND, 1000.0 / FOX_TRAFFIC_TICKS_PER_SECOND);
    }
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
           "%" PRIu64 " arrived, %u standing half a minute, state %016" PRIx64,
           event, FoxWorldTick(city->world), FoxNetworkRoadCount(network),
           FoxNetworkNodeCount(network), stats.vehicles, stats.standing,
           (double)stats.meanSpeed / FOX_UNITS_PER_METRE, stats.arrived, stats.waitingLong,
           FoxWorldHash(city->world));
}
