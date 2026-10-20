#include "expect.h"
#include "sim/planet.h"

#include <stdint.h>
#include <stdlib.h>

#define RADIUS FOX_PLANET_DEFAULT_RADIUS

static void TestUntouchedBall(void)
{
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    EXPECT(planet != NULL, "the planet is created");
    EXPECT(FoxPlanetCreate(1) == NULL, "a pebble is no planet");
    const int64_t surfaceCell = RADIUS / FOX_PLANET_CELL_UNITS;
    EXPECT(FoxPlanetSampleCell(planet, 0, 0, 0).material == FOX_MATERIAL_CORE,
           "the centre is core");
    EXPECT(FoxPlanetSampleCell(planet, 0, 0, surfaceCell - 1).material == FOX_MATERIAL_GRASS,
           "the skin is grass");
    EXPECT(FoxPlanetSampleCell(planet, 0, 0, surfaceCell - 8).material == FOX_MATERIAL_DIRT,
           "below the grass lies dirt");
    EXPECT(FoxPlanetSampleCell(planet, surfaceCell + 2, 0, 0).density < 0, "outside is air");
    EXPECT(FoxPlanetSampleCell(planet, INT64_C(1) << 40, 0, 0).density < 0,
           "far space is air without overflow");

    FoxVec ground;
    EXPECT(FoxPlanetGround(planet, (FoxVec){0, 0, FOX_METRES(5)}, &ground), "ground is found");
    EXPECT(FoxAbs(ground.z - RADIUS) < FOX_METRES(1) / 20 && ground.x == 0 && ground.y == 0,
           "the ground of the north pole is a radius up");
    EXPECT(FoxPlanetGround(planet, (FoxVec){-FOX_METRES(3), FOX_METRES(4), 0}, &ground) &&
               FoxAbs(FoxVecLength(ground) - RADIUS) < FOX_METRES(1) / 20,
           "ground on the equator is a radius out");

    FoxVec hit;
    const FoxVec eye = {FOX_METRES(3000), FOX_METRES(10), 0};
    EXPECT(FoxPlanetRaycast(planet, eye, (FoxVec){-1, 0, 0}, FOX_METRES(10000), &hit) &&
               FoxAbs(FoxVecLength(hit) - RADIUS) < FOX_METRES(1) / 20 && hit.x > 0,
           "a ray from space hits the near side");
    EXPECT(!FoxPlanetRaycast(planet, eye, (FoxVec){1, 0, 0}, FOX_METRES(10000), &hit),
           "a ray into space misses");
    EXPECT(!FoxPlanetRaycast(planet, eye, (FoxVec){-1, 0, 0}, FOX_METRES(500), &hit),
           "a short ray stops before the ground");

    EXPECT(FoxPlanetBoxMayHaveSurface(planet, (FoxVec){-FOX_METRES(4), -FOX_METRES(4), RADIUS - 64},
                                      (FoxVec){FOX_METRES(4), FOX_METRES(4), RADIUS + 64}),
           "the skin has a surface");
    EXPECT(!FoxPlanetBoxMayHaveSurface(planet, (FoxVec){-FOX_METRES(4), -FOX_METRES(4), 0},
                                       (FoxVec){FOX_METRES(4), FOX_METRES(4), FOX_METRES(8)}),
           "the core has none");
    EXPECT(FoxPlanetEditNodeCount(planet) == 0u, "untouched ground stores nothing");
    FoxPlanetDestroy(planet);
}

static void TestDigAndFill(void)
{
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    const uint64_t untouched = FoxPlanetHash(planet);
    const FoxVec pole = {0, 0, RADIUS};
    FoxPlanetBox changed;
    EXPECT(FoxPlanetDig(planet, pole, FOX_METRES(6), &changed) > 0u, "digging changes ground");
    EXPECT(changed.min.z < RADIUS - FOX_METRES(5) && changed.max.z >= RADIUS,
           "the changed box spans the pit");
    FoxVec ground;
    EXPECT(FoxPlanetGround(planet, pole, &ground) &&
               FoxAbs(ground.z - (RADIUS - FOX_METRES(6))) < FOX_METRES(1) / 4,
           "the pit floor is six metres down");
    EXPECT(FoxPlanetHash(planet) != untouched, "the hash sees the pit");
    EXPECT(FoxPlanetBoxMayHaveSurface(
               planet, (FoxVec){-FOX_METRES(1), -FOX_METRES(1), RADIUS - FOX_METRES(7)},
               (FoxVec){FOX_METRES(1), FOX_METRES(1), RADIUS - FOX_METRES(5)}),
           "the pit floor deep in the rock has a surface");

    const FoxVec equator = {RADIUS, 0, 0};
    EXPECT(FoxPlanetFill(planet, equator, FOX_METRES(5), NULL) > 0u, "filling changes ground");
    EXPECT(FoxPlanetGround(planet, equator, &ground) &&
               FoxAbs(ground.x - (RADIUS + FOX_METRES(5))) < FOX_METRES(1) / 4,
           "the mound top is five metres up");
    EXPECT(FoxPlanetSampleAt(planet, (FoxVec){RADIUS + FOX_METRES(4), 0, 0}).material ==
               FOX_MATERIAL_GRASS,
           "the mound's skin is grass");
    EXPECT(FoxPlanetDig(planet, (FoxVec){0, 0, RADIUS + FOX_METRES(40)}, FOX_METRES(10), NULL) ==
               0u,
           "digging the sky changes nothing");
    FoxPlanetDestroy(planet);
}

static void TestOrderIndependence(void)
{
    FoxPlanet *first = FoxPlanetCreate(RADIUS);
    FoxPlanet *second = FoxPlanetCreate(RADIUS);
    const FoxVec a = {FOX_METRES(3), 0, RADIUS};
    const FoxVec b = {-FOX_METRES(2), FOX_METRES(1), RADIUS - FOX_METRES(1)};
    (void)FoxPlanetDig(first, a, FOX_METRES(4), NULL);
    (void)FoxPlanetDig(first, b, FOX_METRES(3), NULL);
    (void)FoxPlanetDig(second, b, FOX_METRES(3), NULL);
    (void)FoxPlanetDig(second, a, FOX_METRES(4), NULL);
    EXPECT(FoxPlanetHash(first) == FoxPlanetHash(second),
           "the same ground hashes the same whatever the order");
    FoxPlanetDestroy(first);
    FoxPlanetDestroy(second);
}

static void TestHollowCollapses(void)
{
    /* A big cavern inside the rock: its interior becomes uniform air and
     * collapses, so memory follows the cavern wall, not its volume. */
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    for (int64_t step = 0; step < 8; ++step)
    {
        const FoxVec center = {step * FOX_METRES(40), 0, FOX_METRES(300)};
        EXPECT(FoxPlanetDig(planet, center, FOX_PLANET_EDIT_MAX_RADIUS, NULL) > 0u,
               "the cavern is dug");
    }
    const uint32_t nodes = FoxPlanetEditNodeCount(planet);
    EXPECT(nodes > 0u && nodes < 40000u, "a cavern costs its wall");
    FoxVec hit;
    EXPECT(FoxPlanetSampleAt(planet, (FoxVec){FOX_METRES(100), 0, FOX_METRES(300)}).density < 0,
           "inside the cavern is air");
    /* Halfway between two spheres 40 m apart the ceiling is lower than a
     * radius: sqrt(64^2 - 20^2) metres above the axis. */
    const int64_t ceiling = (int64_t)FoxIntegerSqrt(
        (uint64_t)(FOX_PLANET_EDIT_MAX_RADIUS * FOX_PLANET_EDIT_MAX_RADIUS -
                   FOX_METRES(20) * FOX_METRES(20)));
    EXPECT(FoxPlanetRaycast(planet, (FoxVec){FOX_METRES(100), 0, FOX_METRES(300)},
                            (FoxVec){0, 0, 1}, FOX_METRES(1000), &hit) &&
               FoxAbs(hit.z - (FOX_METRES(300) + ceiling)) < FOX_METRES(1),
           "a ray inside the cavern hits its ceiling");
    FoxPlanetDestroy(planet);
}

static void TestGrid(void)
{
    FoxPlanet *planet = FoxPlanetCreate(RADIUS);
    (void)FoxPlanetDig(planet, (FoxVec){0, 0, RADIUS}, FOX_METRES(3), NULL);
    const FoxPlanetGrid grid = {
        .cellX = -4, .cellY = -4, .cellZ = RADIUS / FOX_PLANET_CELL_UNITS - 8,
        .stride = 1u, .countX = 9u, .countY = 9u, .countZ = 12u,
    };
    FoxPlanetSample *samples = malloc((size_t)9u * 9u * 12u * sizeof(*samples));
    EXPECT(samples != NULL && FoxPlanetSampleGrid(planet, &grid, samples), "the grid samples");
    size_t index = 0u;
    for (uint32_t z = 0u; z < grid.countZ; ++z)
    {
        for (uint32_t y = 0u; y < grid.countY; ++y)
        {
            for (uint32_t x = 0u; x < grid.countX; ++x)
            {
                const FoxPlanetSample single = FoxPlanetSampleCell(
                    planet, grid.cellX + x, grid.cellY + y, grid.cellZ + z);
                EXPECT(single.density == samples[index].density &&
                           single.material == samples[index].material,
                       "the grid equals single samples");
                ++index;
            }
        }
    }
    const FoxPlanetGrid bad = {.stride = 3u, .countX = 1u, .countY = 1u, .countZ = 1u};
    EXPECT(!FoxPlanetSampleGrid(planet, &bad, samples), "a stride must be a power of two");
    free(samples);
    FoxPlanetDestroy(planet);
}

int main(void)
{
    TestUntouchedBall();
    TestDigAndFill();
    TestOrderIndependence();
    TestHollowCollapses();
    TestGrid();
    return 0;
}
