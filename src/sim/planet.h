#ifndef FOX_SIM_PLANET_H
#define FOX_SIM_PLANET_H

#include "sim/space.h"

#include <stdbool.h>
#include <stdint.h>

/* The planet: a solid ball at the origin of space, smooth for now, and
 * volumetric through and through, so the city can dig anywhere down to the
 * core and pile ground up anywhere.
 *
 * The ground is a density field sampled on a 0.5 m lattice: positive inside
 * matter, negative in air, zero on the surface. Untouched ground is a pure
 * function of the radius and costs nothing to store. What the city digs away
 * or piles up is a sparse layer over that function, kept in a three-level
 * tree of 8 x 8 x 8 blocks (4 m, 32 m and 256 m): a block that turned
 * uniformly into air or rock collapses into one value, so even a hollowed-out
 * planet stays small. Untouched ground reports its exact distance to the
 * surface, so even samples a kilometre apart place it right; edits are clamped
 * to a metre around the surface, which is all the surface needs there and
 * what lets dug space collapse.
 *
 * Every query is integer arithmetic: a server, every client and a replay see
 * the same ground. */

#define FOX_PLANET_CELL_SHIFT 9u
#define FOX_PLANET_CELL_UNITS (INT64_C(1) << FOX_PLANET_CELL_SHIFT)
#define FOX_PLANET_DEFAULT_RADIUS FOX_METRES(1000)
#define FOX_PLANET_MIN_RADIUS FOX_METRES(16)
#define FOX_PLANET_MAX_RADIUS FOX_METRES(1000000)
/* Edited densities never leave [-LIMIT, LIMIT]. */
#define FOX_PLANET_DENSITY_LIMIT FOX_METRES(1)
/* One dig or fill changes a sphere of at most this radius. */
#define FOX_PLANET_EDIT_MIN_RADIUS (FOX_METRES(1) / 2)
#define FOX_PLANET_EDIT_MAX_RADIUS FOX_METRES(64)
#define FOX_PLANET_GRID_MAX_SAMPLES (1u << 20)
/* Rays step at least this far, so ground thinner than it may be missed. */
#define FOX_PLANET_RAY_MIN_STEP (FOX_METRES(1) / 8)

typedef enum FoxMaterial
{
    FOX_MATERIAL_AIR = 0,
    FOX_MATERIAL_GRASS,
    FOX_MATERIAL_DIRT,
    FOX_MATERIAL_STONE,
    FOX_MATERIAL_CORE,
    FOX_MATERIAL_COUNT,
} FoxMaterial;

typedef struct FoxPlanet FoxPlanet;

typedef struct FoxPlanetSample
{
    int32_t density;
    uint8_t material;
} FoxPlanetSample;

/* A box in world units that a change touched; empty when min.x > max.x. */
typedef struct FoxPlanetBox
{
    FoxVec min;
    FoxVec max;
} FoxPlanetBox;

/* A box of lattice samples, `stride` cells apart (a power of two up to 2^20):
 * far ground is sampled coarsely. */
typedef struct FoxPlanetGrid
{
    int64_t cellX;
    int64_t cellY;
    int64_t cellZ;
    uint32_t stride;
    uint32_t countX;
    uint32_t countY;
    uint32_t countZ;
} FoxPlanetGrid;

/* NULL for a radius outside [MIN_RADIUS, MAX_RADIUS] or without memory. */
FoxPlanet *FoxPlanetCreate(int64_t radius);
void FoxPlanetDestroy(FoxPlanet *planet);

int64_t FoxPlanetRadius(const FoxPlanet *planet);

/* The direction away from the centre through `point`, of length FOX_UNIT;
 * straight up the z axis at the centre itself. */
FoxVec FoxPlanetUp(FoxVec point);

/* The sample at a lattice cell, edits included. */
FoxPlanetSample FoxPlanetSampleCell(const FoxPlanet *planet, int64_t cellX, int64_t cellY,
                                    int64_t cellZ);

/* The ground at any point: densities blended between the eight surrounding
 * samples, and the material of the nearest one. */
FoxPlanetSample FoxPlanetSampleAt(const FoxPlanet *planet, FoxVec point);

/* Fills countX * countY * countZ samples, X fastest, then Y, then Z. False on
 * an invalid request, which leaves the output untouched. Thread-safe while
 * nobody edits the planet. */
bool FoxPlanetSampleGrid(const FoxPlanet *planet, const FoxPlanetGrid *grid,
                         FoxPlanetSample *samples);

/* The first point where a ray from `origin` along `direction` (any non-zero
 * length) enters matter within `maxDistance` units, found to about a
 * centimetre. A ray that starts inside matter hits at its origin. */
bool FoxPlanetRaycast(const FoxPlanet *planet, FoxVec origin, FoxVec direction, int64_t maxDistance,
                      FoxVec *outHit);

/* The outermost ground on the line from the centre through `point`: where a
 * road or a house standing above that point rests. False only when the line
 * has no ground at all. */
bool FoxPlanetGround(const FoxPlanet *planet, FoxVec point, FoxVec *outGround);

/* Digs a sphere out of the ground, or piles one up. The radius is clamped to
 * the edit range. Returns how many samples changed and the box they span
 * (outChanged may be NULL); zero when nothing changed or memory ran out, and
 * then the ground is as before except that what fitted stays changed. */
uint32_t FoxPlanetDig(FoxPlanet *planet, FoxVec center, int64_t radius, FoxPlanetBox *outChanged);
uint32_t FoxPlanetFill(FoxPlanet *planet, FoxVec center, int64_t radius, FoxPlanetBox *outChanged);

/* Whether the surface may pass through a box: false only when the box is
 * certainly all matter or all air. Views skip such boxes. */
bool FoxPlanetBoxMayHaveSurface(const FoxPlanet *planet, FoxVec min, FoxVec max);

/* Tree nodes holding edits: zero for untouched ground. */
uint32_t FoxPlanetEditNodeCount(const FoxPlanet *planet);

/* Folds the radius and every edit into one value that depends only on the
 * resulting ground, not on the order of the digs that made it. */
uint64_t FoxPlanetHash(const FoxPlanet *planet);

#endif
