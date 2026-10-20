#ifndef FOX_CLIENT_CLIENT_H
#define FOX_CLIENT_CLIENT_H

#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum FoxGraphicsChoice
{
    /* The best provider this platform has, falling back to the next one. */
    FOX_GRAPHICS_ANY = 0,
    FOX_GRAPHICS_D3D12,
    FOX_GRAPHICS_VULKAN,
} FoxGraphicsChoice;

typedef struct FoxClientOptions
{
    uint64_t seed;
    FoxGraphicsChoice graphics;
    /* Non-zero presents this many frames and exits: a check that the whole
     * presentation path works on this machine. */
    uint32_t smokeFrames;
    uint32_t cars;
    /* The city's drive side, chosen when it is founded. */
    FoxDriveSide driveSide;
    /* Starts with the demo town instead of bare ground, or with gridLines
     * with a grid of that many roads each way gridSpacing apart. */
    bool demo;
    uint32_t gridLines;
    int64_t gridSpacing;
    /* Metres from the camera to its focus; zero keeps the default. */
    double cameraDistance;
    /* Where the camera looks first, metres east and north of the pole. */
    double focusEast;
    double focusNorth;
    /* Presents without waiting for the display's refresh. */
    bool noVerticalSync;
    /* Non-NULL saves frame captureFrame as a BMP there and exits. */
    const char *capturePath;
    uint32_t captureFrame;
} FoxClientOptions;

/* Exit codes beyond 0 tell why the client could not run. */
#define FOX_CLIENT_EXIT_INVALID_OPTIONS 2
#define FOX_CLIENT_EXIT_NO_ENGINE 3
#define FOX_CLIENT_EXIT_NO_PRESENTATION 4
#define FOX_CLIENT_EXIT_FRAME_FAILED 5
#define FOX_CLIENT_EXIT_NO_WORLD 6

int FoxClientRun(const FoxClientOptions *options);

#endif
