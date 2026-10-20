#ifndef FOX_APP_HEADLESS_H
#define FOX_APP_HEADLESS_H

#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* The city without presentation: a dedicated server, or an accelerated run
 * that checks the simulation on a machine without graphics. */
typedef struct FoxHeadlessOptions
{
    uint64_t seed;
    /* Stops after this many ticks; zero runs until the process is asked to
     * terminate. */
    uint64_t ticks;
    /* 1 is real time; larger values fast-forward as far as the machine
     * allows. */
    double timeScale;
    uint32_t cars;
    FoxDriveSide driveSide;
    /* Off ignores laiue.navigation even where it is present. */
    bool routing;
} FoxHeadlessOptions;

/* Returns the process exit code. */
int FoxHeadlessRun(const FoxHeadlessOptions *options);

#endif
