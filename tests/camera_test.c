#include "client/camera.h"
#include "expect.h"

#include <stdint.h>

/* The camera covers the same ground at any frame rate: a second of panning
 * and a drag close to the ground, in one frame and in thousands. At 20000
 * frames a second a step is a fifth of a millimetre, less than a unit. */

static FoxCamera Start(const FoxPlanet *planet)
{
    FoxVec focus = {0, 0, FOX_PLANET_DEFAULT_RADIUS};
    EXPECT(FoxPlanetGround(planet, focus, &focus), "there is ground at the pole");
    FoxCamera camera;
    FoxCameraInit(&camera, focus);
    camera.distance = FOX_CAMERA_MIN_DISTANCE;
    camera.yaw = 0.5;
    return camera;
}

static FoxVec Pan(const FoxPlanet *planet, uint32_t frames)
{
    FoxCamera camera = Start(planet);
    for (uint32_t frame = 0u; frame < frames; ++frame)
    {
        FoxCameraPan(&camera, planet, 1.0f, 0.5f, 1.0 / (double)frames);
    }
    return camera.focus;
}

/* A drag of 300 by 200 pixels on a 1000 pixel high view. */
static FoxVec Drag(const FoxPlanet *planet, uint32_t frames)
{
    FoxCamera camera = Start(planet);
    for (uint32_t frame = 0u; frame < frames; ++frame)
    {
        FoxCameraDrag(&camera, planet, 300.0f / (float)frames, 200.0f / (float)frames, 1000u);
    }
    return camera.focus;
}

int main(void)
{
    FoxPlanet *planet = FoxPlanetCreate(FOX_PLANET_DEFAULT_RADIUS);
    EXPECT(planet != NULL, "the planet is created");
    const FoxVec start = Start(planet).focus;

    /* Four metres ahead and two to the right, 4.47 m: a second covers the
     * camera's distance. */
    const FoxVec panned = Pan(planet, 1u);
    const int64_t travelled = FoxVecDistance(start, panned);
    EXPECT(travelled > FOX_METRES(44) / 10 && travelled < FOX_METRES(455) / 100,
           "a second of panning covers the camera's distance");
    const FoxVec dragged = Drag(planet, 1u);
    EXPECT(FoxVecDistance(start, dragged) > FOX_METRES(1), "the drag moves the focus");

    static const uint32_t rates[] = {60u, 1000u, 20000u};
    for (uint32_t index = 0u; index < sizeof(rates) / sizeof(rates[0]); ++index)
    {
        /* Within two millimetres: the planet's curve over the few metres. */
        EXPECT(FoxVecDistance(Pan(planet, rates[index]), panned) <= 2,
               "panning covers the same ground at any frame rate");
        EXPECT(FoxVecDistance(Drag(planet, rates[index]), dragged) <= 2,
               "dragging covers the same ground at any frame rate");
    }

    FoxPlanetDestroy(planet);
    return 0;
}
