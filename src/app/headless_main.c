#include "app/arguments.h"
#include "app/headless.h"
#include "sim/traffic.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

int main(int argumentCount, char **arguments)
{
    FoxHeadlessOptions options = {
        .seed = 1u,
        .ticks = 0u,
        .timeScale = 1.0,
        .cars = 150u,
        .driveSide = FOX_DRIVE_RIGHT,
        .routing = true,
    };
    for (int index = 1; index < argumentCount; ++index)
    {
        const char *argument = arguments[index];
        if (strcmp(argument, "--no-routing") == 0)
        {
            options.routing = false;
            continue;
        }
        /* Every other option takes one value. */
        const char *value = index + 1 < argumentCount ? arguments[++index] : NULL;
        bool parsed = false;
        uint64_t number = 0u;
        if (strcmp(argument, "--seed") == 0)
        {
            parsed = FoxParseUint64(value, &options.seed);
        }
        else if (strcmp(argument, "--ticks") == 0)
        {
            parsed = FoxParseUint64(value, &options.ticks);
        }
        else if (strcmp(argument, "--time-scale") == 0)
        {
            parsed = FoxParseDouble(value, &options.timeScale);
        }
        else if (strcmp(argument, "--cars") == 0)
        {
            parsed = FoxParseUint64(value, &number) && number <= FOX_TRAFFIC_MAX_VEHICLES;
            options.cars = (uint32_t)number;
        }
        else if (strcmp(argument, "--drive") == 0)
        {
            parsed = value != NULL && (strcmp(value, "right") == 0 || strcmp(value, "left") == 0);
            options.driveSide =
                parsed && strcmp(value, "left") == 0 ? FOX_DRIVE_LEFT : FOX_DRIVE_RIGHT;
        }
        if (!parsed)
        {
            fputs("usage: FoxityHeadless [--seed N] [--ticks N] [--time-scale X] [--cars N] "
                  "[--drive right|left] [--no-routing]\n",
                  stderr);
            return 2;
        }
    }
    return FoxHeadlessRun(&options);
}
