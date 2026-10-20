#include "app/arguments.h"
#include "client/client.h"
#include "sim/traffic.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static bool ParseGraphics(const char *value, FoxGraphicsChoice *outChoice)
{
    if (value != NULL && strcmp(value, "d3d12") == 0)
    {
        *outChoice = FOX_GRAPHICS_D3D12;
        return true;
    }
    if (value != NULL && strcmp(value, "vulkan") == 0)
    {
        *outChoice = FOX_GRAPHICS_VULKAN;
        return true;
    }
    return false;
}

static int Run(int argumentCount, char **arguments)
{
    FoxClientOptions options = {
        .seed = 1u,
        .graphics = FOX_GRAPHICS_ANY,
        .cars = 150u,
        .driveSide = FOX_DRIVE_RIGHT,
        .demo = true,
        .gridSpacing = FOX_METRES(80),
    };
    for (int index = 1; index < argumentCount; ++index)
    {
        const char *argument = arguments[index];
        if (strcmp(argument, "--smoke") == 0)
        {
            options.smokeFrames = 3u;
            continue;
        }
        if (strcmp(argument, "--empty") == 0)
        {
            options.demo = false;
            continue;
        }
        if (strcmp(argument, "--no-vsync") == 0)
        {
            options.noVerticalSync = true;
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
        else if (strcmp(argument, "--graphics") == 0)
        {
            parsed = ParseGraphics(value, &options.graphics);
        }
        else if (strcmp(argument, "--cars") == 0)
        {
            parsed = FoxParseUint64(value, &number) && number <= FOX_TRAFFIC_MAX_VEHICLES;
            options.cars = (uint32_t)number;
        }
        else if (strcmp(argument, "--camera-distance") == 0)
        {
            parsed = FoxParseDouble(value, &options.cameraDistance) && options.cameraDistance > 0.0;
        }
        else if (strcmp(argument, "--grid") == 0)
        {
            parsed = FoxParseUint64(value, &number) && number >= 2u && number <= 64u;
            options.gridLines = (uint32_t)number;
        }
        else if (strcmp(argument, "--spacing") == 0)
        {
            parsed = FoxParseUint64(value, &number) && number >= 20u && number <= 600u;
            options.gridSpacing = FOX_METRES(number);
        }
        else if (strcmp(argument, "--focus-east") == 0)
        {
            parsed = FoxParseDouble(value, &options.focusEast) && fabs(options.focusEast) < 1000.0;
        }
        else if (strcmp(argument, "--focus-north") == 0)
        {
            parsed = FoxParseDouble(value, &options.focusNorth) && fabs(options.focusNorth) < 1000.0;
        }
        else if (strcmp(argument, "--capture") == 0)
        {
            parsed = value != NULL && value[0] != '\0';
            options.capturePath = value;
            options.captureFrame = options.captureFrame == 0u ? 300u : options.captureFrame;
        }
        else if (strcmp(argument, "--capture-frame") == 0)
        {
            parsed = FoxParseUint64(value, &number) && number > 0u && number <= 100000u;
            options.captureFrame = (uint32_t)number;
        }
        else if (strcmp(argument, "--drive") == 0)
        {
            parsed = value != NULL && (strcmp(value, "right") == 0 || strcmp(value, "left") == 0);
            options.driveSide =
                parsed && strcmp(value, "left") == 0 ? FOX_DRIVE_LEFT : FOX_DRIVE_RIGHT;
        }
        if (!parsed)
        {
            fputs("usage: Foxity [--seed N] [--cars N] [--drive right|left] [--empty] [--camera-distance M] "
                  "[--grid N] [--spacing M] [--focus-east M] [--focus-north M] "
                  "[--graphics d3d12|vulkan] [--no-vsync] [--smoke] "
                  "[--capture FILE.bmp] [--capture-frame N]\n",
                  stderr);
            return FOX_CLIENT_EXIT_INVALID_OPTIONS;
        }
    }
    return FoxClientRun(&options);
}
#if defined(_WIN32)

#include <windows.h>

#include <shellapi.h>

#define FOX_MAX_ARGUMENTS 64
#define FOX_MAX_ARGUMENT_BYTES 1024

/* A windowed program has no console of its own. When it is started from one,
 * diagnostics go there; a redirected stream is left as the parent set it. */
static void AttachParentConsole(void)
{
    HANDLE error = GetStdHandle(STD_ERROR_HANDLE);
    if ((error == NULL || error == INVALID_HANDLE_VALUE) && AttachConsole(ATTACH_PARENT_PROCESS))
    {
        FILE *stream = NULL;
        (void)freopen_s(&stream, "CONOUT$", "w", stderr);
    }
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE previousInstance, PWSTR commandLine,
                    int showCommand)
{
    (void)instance;
    (void)previousInstance;
    (void)commandLine;
    (void)showCommand;
    AttachParentConsole();

    int count = 0;
    LPWSTR *wide = CommandLineToArgvW(GetCommandLineW(), &count);
    if (wide == NULL || count < 1 || count > FOX_MAX_ARGUMENTS)
    {
        LocalFree((HLOCAL)wide);
        return FOX_CLIENT_EXIT_INVALID_OPTIONS;
    }
    static char storage[FOX_MAX_ARGUMENTS][FOX_MAX_ARGUMENT_BYTES];
    char *arguments[FOX_MAX_ARGUMENTS];
    for (int index = 0; index < count; ++index)
    {
        if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide[index], -1, storage[index],
                                FOX_MAX_ARGUMENT_BYTES, NULL, NULL) == 0)
        {
            LocalFree((HLOCAL)wide);
            return FOX_CLIENT_EXIT_INVALID_OPTIONS;
        }
        arguments[index] = storage[index];
    }
    LocalFree((HLOCAL)wide);
    return Run(count, arguments);
}

#else

int main(int argumentCount, char **arguments)
{
    return Run(argumentCount, arguments);
}

#endif
