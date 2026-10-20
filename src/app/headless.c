#include "app/headless.h"

#include "app/city.h"
#include "app/log.h"
#include "app/modules.h"
#include "mod/application.h"
#include "sim/clock.h"

#include <math.h>
#include <stddef.h>

/* Never sleep longer than this, so a termination request is noticed quickly
 * even while the city is paused. */
#define FOX_HEADLESS_MAX_SLEEP_MILLISECONDS 50u
/* A report every minute of city time. */
#define FOX_HEADLESS_REPORT_TICKS (UINT64_C(60) * FOX_CLOCK_TICKS_PER_SECOND)

_Static_assert(FOX_CLOCK_TICKS_PER_SECOND == FOX_TRAFFIC_TICKS_PER_SECOND,
               "the clock ticks at the traffic's rate");

/* Zero when the next tick is due within a millisecond: a sleep that short
 * oversleeps, which would hold a fast-forwarded city far below its pace. */
static uint32_t MillisecondsToNextTick(const FoxClock *clock)
{
    const double seconds = FoxClockSecondsToNextTick(clock);
    if (seconds < 0.0)
    {
        return FOX_HEADLESS_MAX_SLEEP_MILLISECONDS;
    }
    const double milliseconds = seconds * 1000.0;
    if (!(milliseconds >= 1.0))
    {
        return 0u;
    }
    return milliseconds >= (double)FOX_HEADLESS_MAX_SLEEP_MILLISECONDS
               ? FOX_HEADLESS_MAX_SLEEP_MILLISECONDS
               : (uint32_t)ceil(milliseconds);
}

int FoxHeadlessRun(const FoxHeadlessOptions *options)
{
    FoxClock clock;
    if (options == NULL ||
        !FoxClockInit(&clock, FOX_CLOCK_TICKS_PER_SECOND, FOX_CLOCK_DEFAULT_MAX_ADVANCE_SECONDS) ||
        !FoxClockSetTimeScale(&clock, options->timeScale))
    {
        FoxLog(FOX_LOG_ERROR, "headless options are invalid");
        return 2;
    }
    FoxModules modules;
    const bool engine = FoxModulesOpen(&modules);
    if (engine && options->routing)
    {
        const char *const names[] = {"navigation"};
        (void)FoxModulesLoad(&modules, names, 1u, NULL, 0u);
    }
    FoxCity city;
    const FoxCityDesc desc = {
        .seed = options->seed,
        .driveSide = options->driveSide,
        .cars = options->cars,
        .demo = true,
        .modules = engine && options->routing ? &modules : NULL,
    };
    if (!FoxCityCreate(&city, &desc))
    {
        FoxLog(FOX_LOG_ERROR, "the city could not be created");
        if (engine)
        {
            FoxModulesClose(&modules);
        }
        return 3;
    }
    if (LaiueApplicationInstallTerminationHandler() == 0u)
    {
        FoxLog(FOX_LOG_WARNING, "termination handler unavailable; stop the process to exit");
    }
    FoxCityReport(&city, "city founded");

    const double started = LaiueApplicationMonotonicSeconds();
    double previous = started;
    uint64_t nextReport = FOX_HEADLESS_REPORT_TICKS;
    while (LaiueApplicationTerminationRequested() == 0u &&
           (options->ticks == 0u || FoxWorldTick(city.world) < options->ticks))
    {
        const double now = LaiueApplicationMonotonicSeconds();
        uint64_t ticks = FoxClockAdvance(&clock, now - previous);
        previous = now;
        const uint64_t done = FoxWorldTick(city.world);
        if (options->ticks != 0u && ticks > options->ticks - done)
        {
            ticks = options->ticks - done;
        }
        FoxWorldAdvance(city.world, ticks);
        if (FoxWorldTick(city.world) >= nextReport)
        {
            FoxCityReport(&city, "running");
            nextReport += FOX_HEADLESS_REPORT_TICKS;
        }
        const uint32_t sleep = MillisecondsToNextTick(&clock);
        if (sleep != 0u)
        {
            LaiueApplicationSleepMilliseconds(sleep);
        }
    }
    LaiueApplicationRemoveTerminationHandler();
    FoxLog(FOX_LOG_INFO, "ran %.3f s", LaiueApplicationMonotonicSeconds() - started);
    FoxCityReport(&city, "stopped");
    FoxCityDestroy(&city);
    if (engine)
    {
        FoxModulesClose(&modules);
    }
    return 0;
}
