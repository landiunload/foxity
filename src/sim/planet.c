#include "sim/planet.h"

#include "sim/random.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* The edit tree. A top node covers 512^3 cells (256 m) and is found by hash;
 * its 8^3 slots cover 64^3 cells (32 m), the slots of the middle level 8^3
 * cells (4 m), and a leaf holds single cells. A slot is empty (the function
 * stands there), one uniform value, or a child. */
#define BLOCK_VOLUME 512u
#define TOP_SHIFT 9u
/* Cells along a top node; coordinates are multiplied by it, never shifted, as
 * shifting negative values is undefined. */
#define TOP_CELLS (INT64_C(1) << TOP_SHIFT)
#define MIDDLE_SHIFT 6u
#define LEAF_SHIFT 3u
#define LEAF_CELLS 8

enum
{
    SLOT_NONE = 0,
    SLOT_VALUE = 1,
    SLOT_CHILD = 2,
};

typedef struct Leaf
{
    int16_t density[BLOCK_VOLUME];
    uint8_t material[BLOCK_VOLUME];
    uint64_t edited[BLOCK_VOLUME / 64u];
    uint32_t editedCount;
} Leaf;

typedef struct Inner
{
    void *child[BLOCK_VOLUME];
    int16_t density[BLOCK_VOLUME];
    uint8_t material[BLOCK_VOLUME];
    uint8_t kind[BLOCK_VOLUME];
    uint32_t used;
} Inner;

typedef struct Top
{
    int64_t x;
    int64_t y;
    int64_t z;
    /* NULL marks a free hash slot. */
    Inner *node;
} Top;

struct FoxPlanet
{
    int64_t radius;
    Top *tops;
    uint32_t topCapacity;
    uint32_t topCount;
    uint32_t nodeCount;
    /* Cells that ever held an edit; it only grows. */
    bool hasEdits;
    int64_t editMin[3];
    int64_t editMax[3];
    /* The farthest any edit reaches from the centre, in units. */
    int64_t editReach;
};

static int64_t FloorShift(int64_t value, unsigned shift)
{
    if (value >= 0)
    {
        return (int64_t)((uint64_t)value >> shift);
    }
    const uint64_t magnitude = (uint64_t)(-(value + 1));
    return -1 - (int64_t)(magnitude >> shift);
}

static uint32_t SlotIndex(int64_t x, int64_t y, int64_t z, unsigned shift)
{
    return (uint32_t)(((uint64_t)x >> shift) & 7u) | (uint32_t)((((uint64_t)y >> shift) & 7u) << 3u) |
           (uint32_t)((((uint64_t)z >> shift) & 7u) << 6u);
}

static int32_t ClampDensity(int64_t value)
{
    return (int32_t)FoxClamp(value, -FOX_PLANET_DENSITY_LIMIT, FOX_PLANET_DENSITY_LIMIT);
}

FoxVec FoxPlanetUp(FoxVec point)
{
    const FoxVec up = FoxVecDirection(point);
    return FoxVecIsZero(up) ? (FoxVec){0, 0, FOX_UNIT} : up;
}

FoxPlanet *FoxPlanetCreate(int64_t radius)
{
    if (radius < FOX_PLANET_MIN_RADIUS || radius > FOX_PLANET_MAX_RADIUS)
    {
        return NULL;
    }
    FoxPlanet *planet = calloc(1u, sizeof(*planet));
    if (planet == NULL)
    {
        return NULL;
    }
    planet->radius = radius;
    planet->topCapacity = 64u;
    planet->tops = calloc(planet->topCapacity, sizeof(*planet->tops));
    if (planet->tops == NULL)
    {
        free(planet);
        return NULL;
    }
    return planet;
}

/* Frees a middle node and its leaves. */
static void FreeMiddle(FoxPlanet *planet, Inner *node)
{
    for (uint32_t slot = 0u; slot < BLOCK_VOLUME; ++slot)
    {
        if (node->kind[slot] == SLOT_CHILD)
        {
            free(node->child[slot]);
            --planet->nodeCount;
        }
    }
    free(node);
    --planet->nodeCount;
}

/* Frees a top node and everything below it. */
static void FreeUpper(FoxPlanet *planet, Inner *node)
{
    for (uint32_t slot = 0u; slot < BLOCK_VOLUME; ++slot)
    {
        if (node->kind[slot] == SLOT_CHILD)
        {
            FreeMiddle(planet, (Inner *)node->child[slot]);
        }
    }
    free(node);
    --planet->nodeCount;
}
void FoxPlanetDestroy(FoxPlanet *planet)
{
    if (planet == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < planet->topCapacity; ++index)
    {
        if (planet->tops[index].node != NULL)
        {
            FreeUpper(planet, planet->tops[index].node);
        }
    }
    free(planet->tops);
    free(planet);
}

int64_t FoxPlanetRadius(const FoxPlanet *planet)
{
    return planet == NULL ? 0 : planet->radius;
}

uint32_t FoxPlanetEditNodeCount(const FoxPlanet *planet)
{
    return planet == NULL ? 0u : planet->nodeCount;
}

/* ---- The untouched ball --------------------------------------------------- */

/* Distance from the centre to a lattice cell, or -1 so far away that the
 * square would not fit: such cells are air. */
static int64_t CellDistance(int64_t cellX, int64_t cellY, int64_t cellZ)
{
    const int64_t limit = INT64_C(1) << 21;
    if (FoxAbs(cellX) > limit || FoxAbs(cellY) > limit || FoxAbs(cellZ) > limit)
    {
        return -1;
    }
    const int64_t x = cellX * FOX_PLANET_CELL_UNITS;
    const int64_t y = cellY * FOX_PLANET_CELL_UNITS;
    const int64_t z = cellZ * FOX_PLANET_CELL_UNITS;
    return (int64_t)FoxIntegerSqrt((uint64_t)(x * x) + (uint64_t)(y * y) + (uint64_t)(z * z));
}

static uint8_t MaterialAtDepth(const FoxPlanet *planet, int64_t depth, int64_t distance)
{
    if (depth <= 0)
    {
        return FOX_MATERIAL_AIR;
    }
    if (depth <= FOX_METRES(3) / 2)
    {
        return FOX_MATERIAL_GRASS;
    }
    if (depth <= FOX_METRES(8))
    {
        return FOX_MATERIAL_DIRT;
    }
    return distance <= planet->radius / 4 ? FOX_MATERIAL_CORE : FOX_MATERIAL_STONE;
}

static FoxPlanetSample BaseSample(const FoxPlanet *planet, int64_t cellX, int64_t cellY,
                                  int64_t cellZ)
{
    const int64_t distance = CellDistance(cellX, cellY, cellZ);
    if (distance < 0)
    {
        return (FoxPlanetSample){.density = -FOX_PLANET_DENSITY_LIMIT, .material = FOX_MATERIAL_AIR};
    }
    const int64_t depth = planet->radius - distance;
    /* The exact distance to the surface: coarse samples far apart still place
     * it right. Only edits are clamped. */
    return (FoxPlanetSample){
        .density = (int32_t)FoxClamp(depth, -(INT64_C(1) << 30), INT64_C(1) << 30),
        .material = MaterialAtDepth(planet, depth, distance),
    };
}

/* ---- The edit tree ---------------------------------------------------------- */

static uint32_t TopSlot(const FoxPlanet *planet, int64_t x, int64_t y, int64_t z)
{
    uint64_t hash = FoxHashCombine(FoxHashCombine(FoxHash64((uint64_t)x), (uint64_t)y), (uint64_t)z);
    return (uint32_t)hash & (planet->topCapacity - 1u);
}

static Top *FindTop(const FoxPlanet *planet, int64_t x, int64_t y, int64_t z)
{
    uint32_t slot = TopSlot(planet, x, y, z);
    while (planet->tops[slot].node != NULL)
    {
        Top *top = &planet->tops[slot];
        if (top->x == x && top->y == y && top->z == z)
        {
            return top;
        }
        slot = (slot + 1u) & (planet->topCapacity - 1u);
    }
    return NULL;
}

static bool GrowTops(FoxPlanet *planet)
{
    const uint32_t capacity = planet->topCapacity * 2u;
    Top *tops = calloc(capacity, sizeof(*tops));
    if (tops == NULL)
    {
        return false;
    }
    Top *old = planet->tops;
    const uint32_t oldCapacity = planet->topCapacity;
    planet->tops = tops;
    planet->topCapacity = capacity;
    for (uint32_t index = 0u; index < oldCapacity; ++index)
    {
        if (old[index].node != NULL)
        {
            uint32_t slot = TopSlot(planet, old[index].x, old[index].y, old[index].z);
            while (planet->tops[slot].node != NULL)
            {
                slot = (slot + 1u) & (capacity - 1u);
            }
            planet->tops[slot] = old[index];
        }
    }
    free(old);
    return true;
}

static Inner *NewInner(FoxPlanet *planet)
{
    Inner *node = calloc(1u, sizeof(*node));
    if (node != NULL)
    {
        ++planet->nodeCount;
    }
    return node;
}

static Top *AcquireTop(FoxPlanet *planet, int64_t x, int64_t y, int64_t z)
{
    Top *top = FindTop(planet, x, y, z);
    if (top != NULL)
    {
        return top;
    }
    if ((planet->topCount + 1u) * 2u > planet->topCapacity && !GrowTops(planet))
    {
        return NULL;
    }
    Inner *node = NewInner(planet);
    if (node == NULL)
    {
        return NULL;
    }
    uint32_t slot = TopSlot(planet, x, y, z);
    while (planet->tops[slot].node != NULL)
    {
        slot = (slot + 1u) & (planet->topCapacity - 1u);
    }
    planet->tops[slot] = (Top){.x = x, .y = y, .z = z, .node = node};
    ++planet->topCount;
    return &planet->tops[slot];
}

/* Linear probing with backward shift keeps lookups free of tombstones. */
static void RemoveTop(FoxPlanet *planet, Top *top)
{
    const uint32_t mask = planet->topCapacity - 1u;
    uint32_t hole = (uint32_t)(top - planet->tops);
    FreeUpper(planet, top->node);
    planet->tops[hole].node = NULL;
    --planet->topCount;
    uint32_t next = (hole + 1u) & mask;
    while (planet->tops[next].node != NULL)
    {
        const uint32_t home = TopSlot(planet, planet->tops[next].x, planet->tops[next].y,
                                      planet->tops[next].z);
        /* Move the entry into the hole when its home does not lie in
         * (hole, next]. */
        const bool between =
            hole <= next ? (home > hole && home <= next) : (home > hole || home <= next);
        if (!between)
        {
            planet->tops[hole] = planet->tops[next];
            planet->tops[next].node = NULL;
            hole = next;
        }
        next = (next + 1u) & mask;
    }
}

static bool InsideEdits(const FoxPlanet *planet, int64_t cellX, int64_t cellY, int64_t cellZ)
{
    return planet->hasEdits && cellX >= planet->editMin[0] && cellX <= planet->editMax[0] &&
           cellY >= planet->editMin[1] && cellY <= planet->editMax[1] &&
           cellZ >= planet->editMin[2] && cellZ <= planet->editMax[2];
}

static bool EditSample(const FoxPlanet *planet, int64_t cellX, int64_t cellY, int64_t cellZ,
                       FoxPlanetSample *outSample)
{
    if (!InsideEdits(planet, cellX, cellY, cellZ))
    {
        return false;
    }
    const Top *top = FindTop(planet, FloorShift(cellX, TOP_SHIFT), FloorShift(cellY, TOP_SHIFT),
                             FloorShift(cellZ, TOP_SHIFT));
    if (top == NULL)
    {
        return false;
    }
    const Inner *upper = top->node;
    const uint32_t upperSlot = SlotIndex(cellX, cellY, cellZ, MIDDLE_SHIFT);
    if (upper->kind[upperSlot] == SLOT_NONE)
    {
        return false;
    }
    if (upper->kind[upperSlot] == SLOT_VALUE)
    {
        *outSample = (FoxPlanetSample){upper->density[upperSlot], upper->material[upperSlot]};
        return true;
    }
    const Inner *middle = (const Inner *)upper->child[upperSlot];
    const uint32_t middleSlot = SlotIndex(cellX, cellY, cellZ, LEAF_SHIFT);
    if (middle->kind[middleSlot] == SLOT_NONE)
    {
        return false;
    }
    if (middle->kind[middleSlot] == SLOT_VALUE)
    {
        *outSample = (FoxPlanetSample){middle->density[middleSlot], middle->material[middleSlot]};
        return true;
    }
    const Leaf *leaf = (const Leaf *)middle->child[middleSlot];
    const uint32_t cell = SlotIndex(cellX, cellY, cellZ, 0u);
    if ((leaf->edited[cell / 64u] & (UINT64_C(1) << (cell % 64u))) == 0u)
    {
        return false;
    }
    *outSample = (FoxPlanetSample){leaf->density[cell], leaf->material[cell]};
    return true;
}

FoxPlanetSample FoxPlanetSampleCell(const FoxPlanet *planet, int64_t cellX, int64_t cellY,
                                    int64_t cellZ)
{
    FoxPlanetSample sample;
    if (planet == NULL)
    {
        return (FoxPlanetSample){.density = -FOX_PLANET_DENSITY_LIMIT};
    }
    if (EditSample(planet, cellX, cellY, cellZ, &sample))
    {
        return sample;
    }
    return BaseSample(planet, cellX, cellY, cellZ);
}

/* The middle node holding a cell's 4 m block, created and materialised as
 * needed; NULL without memory. */
static Inner *AcquireMiddle(FoxPlanet *planet, int64_t cellX, int64_t cellY, int64_t cellZ)
{
    Top *top = AcquireTop(planet, FloorShift(cellX, TOP_SHIFT), FloorShift(cellY, TOP_SHIFT),
                          FloorShift(cellZ, TOP_SHIFT));
    if (top == NULL)
    {
        return NULL;
    }
    Inner *upper = top->node;
    const uint32_t upperSlot = SlotIndex(cellX, cellY, cellZ, MIDDLE_SHIFT);
    if (upper->kind[upperSlot] == SLOT_CHILD)
    {
        return (Inner *)upper->child[upperSlot];
    }
    Inner *middle = NewInner(planet);
    if (middle == NULL)
    {
        return NULL;
    }
    if (upper->kind[upperSlot] == SLOT_VALUE)
    {
        for (uint32_t slot = 0u; slot < BLOCK_VOLUME; ++slot)
        {
            middle->kind[slot] = SLOT_VALUE;
            middle->density[slot] = upper->density[upperSlot];
            middle->material[slot] = upper->material[upperSlot];
        }
        middle->used = BLOCK_VOLUME;
    }
    else
    {
        ++upper->used;
    }
    upper->kind[upperSlot] = SLOT_CHILD;
    upper->child[upperSlot] = middle;
    return middle;
}

static Leaf *AcquireLeaf(FoxPlanet *planet, Inner *middle, uint32_t middleSlot)
{
    if (middle->kind[middleSlot] == SLOT_CHILD)
    {
        return (Leaf *)middle->child[middleSlot];
    }
    Leaf *leaf = calloc(1u, sizeof(*leaf));
    if (leaf == NULL)
    {
        return NULL;
    }
    ++planet->nodeCount;
    if (middle->kind[middleSlot] == SLOT_VALUE)
    {
        for (uint32_t cell = 0u; cell < BLOCK_VOLUME; ++cell)
        {
            leaf->density[cell] = middle->density[middleSlot];
            leaf->material[cell] = middle->material[middleSlot];
        }
        memset(leaf->edited, 0xFF, sizeof(leaf->edited));
        leaf->editedCount = BLOCK_VOLUME;
    }
    else
    {
        ++middle->used;
    }
    middle->kind[middleSlot] = SLOT_CHILD;
    middle->child[middleSlot] = leaf;
    return leaf;
}

static void SetMiddleSlot(FoxPlanet *planet, Inner *middle, uint32_t slot, uint8_t kind,
                          FoxPlanetSample value)
{
    if (middle->kind[slot] == SLOT_CHILD)
    {
        free(middle->child[slot]);
        middle->child[slot] = NULL;
        --planet->nodeCount;
    }
    if (middle->kind[slot] == SLOT_NONE && kind != SLOT_NONE)
    {
        ++middle->used;
    }
    else if (middle->kind[slot] != SLOT_NONE && kind == SLOT_NONE)
    {
        --middle->used;
    }
    middle->kind[slot] = kind;
    middle->density[slot] = (int16_t)value.density;
    middle->material[slot] = value.material;
}

/* A leaf whose cells all match collapses into one value; one without edits
 * goes away. */
static void CollapseLeaf(FoxPlanet *planet, Inner *middle, uint32_t slot)
{
    if (middle->kind[slot] != SLOT_CHILD)
    {
        return;
    }
    const Leaf *leaf = (const Leaf *)middle->child[slot];
    if (leaf->editedCount == 0u)
    {
        SetMiddleSlot(planet, middle, slot, SLOT_NONE, (FoxPlanetSample){0});
        return;
    }
    if (leaf->editedCount != BLOCK_VOLUME)
    {
        return;
    }
    for (uint32_t cell = 1u; cell < BLOCK_VOLUME; ++cell)
    {
        if (leaf->density[cell] != leaf->density[0] || leaf->material[cell] != leaf->material[0])
        {
            return;
        }
    }
    const FoxPlanetSample value = {leaf->density[0], leaf->material[0]};
    SetMiddleSlot(planet, middle, slot, SLOT_VALUE, value);
}

/* The same one level up, for the middle nodes under a top node. */
static void CollapseMiddles(FoxPlanet *planet, Top *top)
{
    Inner *upper = top->node;
    for (uint32_t slot = 0u; slot < BLOCK_VOLUME; ++slot)
    {
        if (upper->kind[slot] != SLOT_CHILD)
        {
            continue;
        }
        Inner *middle = (Inner *)upper->child[slot];
        bool uniform = middle->used == BLOCK_VOLUME;
        for (uint32_t index = 0u; uniform && index < BLOCK_VOLUME; ++index)
        {
            uniform = middle->kind[index] == SLOT_VALUE &&
                      middle->density[index] == middle->density[0] &&
                      middle->material[index] == middle->material[0];
        }
        if (middle->used == 0u || uniform)
        {
            const int16_t density = middle->density[0];
            const uint8_t material = middle->material[0];
            FreeMiddle(planet, middle);
            upper->child[slot] = NULL;
            if (uniform)
            {
                upper->kind[slot] = SLOT_VALUE;
                upper->density[slot] = density;
                upper->material[slot] = material;
            }
            else
            {
                upper->kind[slot] = SLOT_NONE;
                --upper->used;
            }
        }
    }
}

static void ExtendEdits(FoxPlanet *planet, const int64_t minimum[3], const int64_t maximum[3])
{
    if (!planet->hasEdits)
    {
        memcpy(planet->editMin, minimum, sizeof(planet->editMin));
        memcpy(planet->editMax, maximum, sizeof(planet->editMax));
        planet->hasEdits = true;
    }
    else
    {
        for (uint32_t axis = 0u; axis < 3u; ++axis)
        {
            planet->editMin[axis] = FoxMin(planet->editMin[axis], minimum[axis]);
            planet->editMax[axis] = FoxMax(planet->editMax[axis], maximum[axis]);
        }
    }
    int64_t reach = 0;
    for (uint32_t corner = 0u; corner < 8u; ++corner)
    {
        const FoxVec point = {
            ((corner & 1u) != 0u ? planet->editMax[0] : planet->editMin[0]) * FOX_PLANET_CELL_UNITS,
            ((corner & 2u) != 0u ? planet->editMax[1] : planet->editMin[1]) * FOX_PLANET_CELL_UNITS,
            ((corner & 4u) != 0u ? planet->editMax[2] : planet->editMin[2]) * FOX_PLANET_CELL_UNITS,
        };
        reach = FoxMax(reach, FoxVecLength(point));
    }
    planet->editReach = reach;
}

/* ---- Digging and filling ---------------------------------------------------- */

typedef enum EditKind
{
    EDIT_DIG,
    EDIT_FILL,
} EditKind;

typedef struct Edit
{
    EditKind kind;
    FoxVec center;
    int64_t radius;
} Edit;

static FoxPlanetSample EditedValue(const Edit *edit, FoxPlanetSample old, FoxVec point)
{
    const int64_t distance = FoxVecDistance(point, edit->center);
    if (edit->kind == EDIT_DIG)
    {
        const int64_t wall = distance - edit->radius;
        if (wall >= old.density)
        {
            return old;
        }
        const int32_t density = ClampDensity(wall);
        return (FoxPlanetSample){density, density > 0 ? old.material : (uint8_t)FOX_MATERIAL_AIR};
    }
    const int64_t inside = edit->radius - distance;
    if (inside <= old.density)
    {
        return old;
    }
    /* New ground shows grass on its skin and dirt below it. */
    const uint8_t material =
        old.density > 0 ? old.material
                        : (inside <= FOX_METRES(1) ? (uint8_t)FOX_MATERIAL_GRASS
                                                   : (uint8_t)FOX_MATERIAL_DIRT);
    return (FoxPlanetSample){ClampDensity(inside), material};
}

/* Nearest and farthest distance from `point` to the lattice points of a
 * block of cells. */
static void BlockDistances(FoxVec point, const int64_t first[3], int64_t cells, int64_t *outNear,
                           int64_t *outFar)
{
    const int64_t centre[3] = {point.x, point.y, point.z};
    FoxVec nearOffset;
    FoxVec farOffset;
    int64_t *nearAxis[3] = {&nearOffset.x, &nearOffset.y, &nearOffset.z};
    int64_t *farAxis[3] = {&farOffset.x, &farOffset.y, &farOffset.z};
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        const int64_t low = first[axis] * FOX_PLANET_CELL_UNITS;
        const int64_t high = (first[axis] + cells - 1) * FOX_PLANET_CELL_UNITS;
        *nearAxis[axis] = centre[axis] < low ? low - centre[axis]
                                             : (centre[axis] > high ? centre[axis] - high : 0);
        *farAxis[axis] = FoxMax(FoxAbs(centre[axis] - low), FoxAbs(centre[axis] - high));
    }
    *outNear = FoxVecLength(nearOffset);
    *outFar = FoxVecLength(farOffset);
}

static bool BlockHasEdits(const FoxPlanet *planet, const int64_t first[3])
{
    const int64_t last[3] = {first[0] + LEAF_CELLS - 1, first[1] + LEAF_CELLS - 1,
                             first[2] + LEAF_CELLS - 1};
    if (!planet->hasEdits || last[0] < planet->editMin[0] || first[0] > planet->editMax[0] ||
        last[1] < planet->editMin[1] || first[1] > planet->editMax[1] ||
        last[2] < planet->editMin[2] || first[2] > planet->editMax[2])
    {
        return false;
    }
    const Top *top = FindTop(planet, FloorShift(first[0], TOP_SHIFT),
                             FloorShift(first[1], TOP_SHIFT), FloorShift(first[2], TOP_SHIFT));
    return top != NULL &&
           top->node->kind[SlotIndex(first[0], first[1], first[2], MIDDLE_SHIFT)] != SLOT_NONE;
}

typedef struct EditResult
{
    uint32_t changed;
    bool failed;
    int64_t minimum[3];
    int64_t maximum[3];
} EditResult;

static void NoteChange(EditResult *result, const int64_t first[3], const int64_t last[3],
                       uint32_t count)
{
    if (result->changed == 0u)
    {
        memcpy(result->minimum, first, sizeof(result->minimum));
        memcpy(result->maximum, last, sizeof(result->maximum));
    }
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        result->minimum[axis] = FoxMin(result->minimum[axis], first[axis]);
        result->maximum[axis] = FoxMax(result->maximum[axis], last[axis]);
    }
    result->changed = result->changed > UINT32_MAX - count ? UINT32_MAX : result->changed + count;
}

/* One 4 m block of an edit. */
static void EditBlock(FoxPlanet *planet, const Edit *edit, const int64_t first[3],
                      EditResult *result)
{
    int64_t nearest = 0;
    int64_t farthest = 0;
    BlockDistances(edit->center, first, LEAF_CELLS, &nearest, &farthest);
    const int64_t last[3] = {first[0] + LEAF_CELLS - 1, first[1] + LEAF_CELLS - 1,
                             first[2] + LEAF_CELLS - 1};
    /* Beyond the sphere's reach nothing changes. */
    if ((edit->kind == EDIT_DIG && nearest - edit->radius >= FOX_PLANET_DENSITY_LIMIT) ||
        (edit->kind == EDIT_FILL && edit->radius - nearest <= -FOX_PLANET_DENSITY_LIMIT))
    {
        return;
    }
    /* Deep inside the sphere the whole block takes one value, unless the
     * untouched ball already has it everywhere there. */
    const bool deep = edit->kind == EDIT_DIG ? farthest - edit->radius <= -FOX_PLANET_DENSITY_LIMIT
                                             : edit->radius - farthest >= FOX_PLANET_DENSITY_LIMIT;
    if (deep)
    {
        const FoxPlanetSample value =
            edit->kind == EDIT_DIG
                ? (FoxPlanetSample){-FOX_PLANET_DENSITY_LIMIT, FOX_MATERIAL_AIR}
                : (FoxPlanetSample){FOX_PLANET_DENSITY_LIMIT, FOX_MATERIAL_DIRT};
        int64_t centreNear = 0;
        int64_t centreFar = 0;
        BlockDistances((FoxVec){0, 0, 0}, first, LEAF_CELLS, &centreNear, &centreFar);
        const bool baseHasIt =
            edit->kind == EDIT_DIG
                ? planet->radius - centreNear <= -FOX_PLANET_DENSITY_LIMIT
                : planet->radius - centreFar >= FOX_PLANET_DENSITY_LIMIT;
        const bool edited = BlockHasEdits(planet, first);
        if (baseHasIt && !edited)
        {
            return;
        }
        Inner *middle = AcquireMiddle(planet, first[0], first[1], first[2]);
        if (middle == NULL)
        {
            result->failed = true;
            return;
        }
        const uint32_t slot = SlotIndex(first[0], first[1], first[2], LEAF_SHIFT);
        if (middle->kind[slot] == SLOT_VALUE && middle->density[slot] == value.density &&
            middle->material[slot] == value.material)
        {
            return;
        }
        SetMiddleSlot(planet, middle, slot, SLOT_VALUE, value);
        ExtendEdits(planet, first, last);
        NoteChange(result, first, last, BLOCK_VOLUME);
        return;
    }

    /* The boundary of the sphere: cell by cell. */
    FoxPlanetSample values[BLOCK_VOLUME];
    uint64_t changedMask[BLOCK_VOLUME / 64u] = {0};
    uint32_t changed = 0u;
    for (uint32_t cell = 0u; cell < BLOCK_VOLUME; ++cell)
    {
        const int64_t x = first[0] + (int64_t)(cell & 7u);
        const int64_t y = first[1] + (int64_t)((cell >> 3u) & 7u);
        const int64_t z = first[2] + (int64_t)(cell >> 6u);
        const FoxPlanetSample old = FoxPlanetSampleCell(planet, x, y, z);
        const FoxVec point = {x * FOX_PLANET_CELL_UNITS, y * FOX_PLANET_CELL_UNITS,
                              z * FOX_PLANET_CELL_UNITS};
        values[cell] = EditedValue(edit, old, point);
        if (values[cell].density != old.density || values[cell].material != old.material)
        {
            changedMask[cell / 64u] |= UINT64_C(1) << (cell % 64u);
            ++changed;
        }
    }
    if (changed == 0u)
    {
        return;
    }
    Inner *middle = AcquireMiddle(planet, first[0], first[1], first[2]);
    const uint32_t slot = SlotIndex(first[0], first[1], first[2], LEAF_SHIFT);
    Leaf *leaf = middle == NULL ? NULL : AcquireLeaf(planet, middle, slot);
    if (leaf == NULL)
    {
        result->failed = true;
        return;
    }
    for (uint32_t cell = 0u; cell < BLOCK_VOLUME; ++cell)
    {
        if ((changedMask[cell / 64u] & (UINT64_C(1) << (cell % 64u))) == 0u)
        {
            continue;
        }
        const int64_t x = first[0] + (int64_t)(cell & 7u);
        const int64_t y = first[1] + (int64_t)((cell >> 3u) & 7u);
        const int64_t z = first[2] + (int64_t)(cell >> 6u);
        const FoxPlanetSample base = BaseSample(planet, x, y, z);
        const uint64_t bit = UINT64_C(1) << (cell % 64u);
        const bool wasEdited = (leaf->edited[cell / 64u] & bit) != 0u;
        /* Ground equal to the untouched ball is no edit at all, which keeps
         * the tree and its hash a function of the ground alone. */
        if (values[cell].density == base.density && values[cell].material == base.material)
        {
            if (wasEdited)
            {
                leaf->edited[cell / 64u] &= ~bit;
                --leaf->editedCount;
            }
            continue;
        }
        if (!wasEdited)
        {
            leaf->edited[cell / 64u] |= bit;
            ++leaf->editedCount;
        }
        leaf->density[cell] = (int16_t)values[cell].density;
        leaf->material[cell] = values[cell].material;
    }
    CollapseLeaf(planet, middle, slot);
    ExtendEdits(planet, first, last);
    NoteChange(result, first, last, changed);
}

static uint32_t ApplyEdit(FoxPlanet *planet, EditKind kind, FoxVec center, int64_t radius,
                          FoxPlanetBox *outChanged)
{
    if (outChanged != NULL)
    {
        *outChanged = (FoxPlanetBox){.min = {1, 0, 0}, .max = {0, 0, 0}};
    }
    if (planet == NULL)
    {
        return 0u;
    }
    const int64_t limit = INT64_C(1) << 40;
    if (FoxAbs(center.x) > limit || FoxAbs(center.y) > limit || FoxAbs(center.z) > limit)
    {
        return 0u;
    }
    const Edit edit = {
        .kind = kind,
        .center = center,
        .radius = FoxClamp(radius, FOX_PLANET_EDIT_MIN_RADIUS, FOX_PLANET_EDIT_MAX_RADIUS),
    };
    const int64_t reach = edit.radius + FOX_PLANET_DENSITY_LIMIT + FOX_PLANET_CELL_UNITS;
    const int64_t centre[3] = {center.x, center.y, center.z};
    int64_t firstBlock[3];
    int64_t lastBlock[3];
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        firstBlock[axis] =
            FloorShift(FloorShift(centre[axis] - reach, FOX_PLANET_CELL_SHIFT), LEAF_SHIFT);
        lastBlock[axis] =
            FloorShift(FloorShift(centre[axis] + reach, FOX_PLANET_CELL_SHIFT) + 1, LEAF_SHIFT);
    }
    EditResult result = {0};
    for (int64_t z = firstBlock[2]; z <= lastBlock[2] && !result.failed; ++z)
    {
        for (int64_t y = firstBlock[1]; y <= lastBlock[1] && !result.failed; ++y)
        {
            for (int64_t x = firstBlock[0]; x <= lastBlock[0] && !result.failed; ++x)
            {
                const int64_t first[3] = {x * LEAF_CELLS, y * LEAF_CELLS, z * LEAF_CELLS};
                EditBlock(planet, &edit, first, &result);
            }
        }
    }
    /* Middle nodes that became uniform or empty collapse; top nodes left
     * empty go away. */
    for (int64_t z = FloorShift(firstBlock[2], TOP_SHIFT - LEAF_SHIFT);
         z <= FloorShift(lastBlock[2], TOP_SHIFT - LEAF_SHIFT); ++z)
    {
        for (int64_t y = FloorShift(firstBlock[1], TOP_SHIFT - LEAF_SHIFT);
             y <= FloorShift(lastBlock[1], TOP_SHIFT - LEAF_SHIFT); ++y)
        {
            for (int64_t x = FloorShift(firstBlock[0], TOP_SHIFT - LEAF_SHIFT);
                 x <= FloorShift(lastBlock[0], TOP_SHIFT - LEAF_SHIFT); ++x)
            {
                Top *top = FindTop(planet, x, y, z);
                if (top == NULL)
                {
                    continue;
                }
                CollapseMiddles(planet, top);
                if (top->node->used == 0u)
                {
                    RemoveTop(planet, top);
                }
            }
        }
    }
    if (result.failed || result.changed == 0u)
    {
        return 0u;
    }
    if (outChanged != NULL)
    {
        *outChanged = (FoxPlanetBox){
            .min = {result.minimum[0] * FOX_PLANET_CELL_UNITS,
                    result.minimum[1] * FOX_PLANET_CELL_UNITS,
                    result.minimum[2] * FOX_PLANET_CELL_UNITS},
            .max = {result.maximum[0] * FOX_PLANET_CELL_UNITS,
                    result.maximum[1] * FOX_PLANET_CELL_UNITS,
                    result.maximum[2] * FOX_PLANET_CELL_UNITS},
        };
    }
    return result.changed;
}

uint32_t FoxPlanetDig(FoxPlanet *planet, FoxVec center, int64_t radius, FoxPlanetBox *outChanged)
{
    return ApplyEdit(planet, EDIT_DIG, center, radius, outChanged);
}

uint32_t FoxPlanetFill(FoxPlanet *planet, FoxVec center, int64_t radius, FoxPlanetBox *outChanged)
{
    return ApplyEdit(planet, EDIT_FILL, center, radius, outChanged);
}

/* ---- Queries ---------------------------------------------------------------- */

FoxPlanetSample FoxPlanetSampleAt(const FoxPlanet *planet, FoxVec point)
{
    if (planet == NULL)
    {
        return (FoxPlanetSample){.density = -FOX_PLANET_DENSITY_LIMIT};
    }
    const int64_t cell[3] = {
        FloorShift(point.x, FOX_PLANET_CELL_SHIFT),
        FloorShift(point.y, FOX_PLANET_CELL_SHIFT),
        FloorShift(point.z, FOX_PLANET_CELL_SHIFT),
    };
    const int64_t fraction[3] = {
        point.x - cell[0] * FOX_PLANET_CELL_UNITS,
        point.y - cell[1] * FOX_PLANET_CELL_UNITS,
        point.z - cell[2] * FOX_PLANET_CELL_UNITS,
    };
    const int64_t one = FOX_PLANET_CELL_UNITS;
    int64_t sum = 0;
    uint8_t material = FOX_MATERIAL_AIR;
    int64_t bestWeight = -1;
    for (uint32_t corner = 0u; corner < 8u; ++corner)
    {
        const int64_t dx = (corner & 1u) != 0u ? 1 : 0;
        const int64_t dy = (corner & 2u) != 0u ? 1 : 0;
        const int64_t dz = (corner & 4u) != 0u ? 1 : 0;
        const FoxPlanetSample sample =
            FoxPlanetSampleCell(planet, cell[0] + dx, cell[1] + dy, cell[2] + dz);
        const int64_t weight = (dx != 0 ? fraction[0] : one - fraction[0]) *
                               (dy != 0 ? fraction[1] : one - fraction[1]) *
                               (dz != 0 ? fraction[2] : one - fraction[2]);
        sum += sample.density * weight;
        if (weight > bestWeight)
        {
            bestWeight = weight;
            material = sample.material;
        }
    }
    const int32_t density = (int32_t)FoxFloorDivide(sum, one * one * one);
    return (FoxPlanetSample){density, density > 0 ? material : (uint8_t)FOX_MATERIAL_AIR};
}

bool FoxPlanetSampleGrid(const FoxPlanet *planet, const FoxPlanetGrid *grid,
                         FoxPlanetSample *samples)
{
    if (planet == NULL || grid == NULL || samples == NULL || grid->countX == 0u ||
        grid->countY == 0u || grid->countZ == 0u || grid->stride == 0u ||
        grid->stride > (1u << 20) || (grid->stride & (grid->stride - 1u)) != 0u ||
        (uint64_t)grid->countX * grid->countY * grid->countZ > FOX_PLANET_GRID_MAX_SAMPLES)
    {
        return false;
    }
    const int64_t stride = grid->stride;
    const int64_t last[3] = {
        grid->cellX + (int64_t)(grid->countX - 1u) * stride,
        grid->cellY + (int64_t)(grid->countY - 1u) * stride,
        grid->cellZ + (int64_t)(grid->countZ - 1u) * stride,
    };
    const bool edited = planet->hasEdits && last[0] >= planet->editMin[0] &&
                        grid->cellX <= planet->editMax[0] && last[1] >= planet->editMin[1] &&
                        grid->cellY <= planet->editMax[1] && last[2] >= planet->editMin[2] &&
                        grid->cellZ <= planet->editMax[2];
    size_t index = 0u;
    for (uint32_t z = 0u; z < grid->countZ; ++z)
    {
        const int64_t cellZ = grid->cellZ + (int64_t)z * stride;
        for (uint32_t y = 0u; y < grid->countY; ++y)
        {
            const int64_t cellY = grid->cellY + (int64_t)y * stride;
            for (uint32_t x = 0u; x < grid->countX; ++x)
            {
                const int64_t cellX = grid->cellX + (int64_t)x * stride;
                if (!edited || !EditSample(planet, cellX, cellY, cellZ, &samples[index]))
                {
                    samples[index] = BaseSample(planet, cellX, cellY, cellZ);
                }
                ++index;
            }
        }
    }
    return true;
}

/* Distance from a point to the box of edited cells, zero inside it. */
static int64_t DistanceToEdits(const FoxPlanet *planet, FoxVec point)
{
    const int64_t coordinates[3] = {point.x, point.y, point.z};
    FoxVec offset = {0, 0, 0};
    int64_t *axes[3] = {&offset.x, &offset.y, &offset.z};
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        const int64_t low = (planet->editMin[axis] - 1) * FOX_PLANET_CELL_UNITS;
        const int64_t high = (planet->editMax[axis] + 1) * FOX_PLANET_CELL_UNITS;
        *axes[axis] = coordinates[axis] < low
                          ? low - coordinates[axis]
                          : (coordinates[axis] > high ? coordinates[axis] - high : 0);
    }
    return FoxVecLength(offset);
}

/* The density used for marching and a safe step from it. */
static int64_t MarchDensity(const FoxPlanet *planet, FoxVec point, int64_t *outStep)
{
    const int64_t distance = FoxVecLength(point);
    const int64_t analytic = planet->radius - distance;
    const int64_t toEdits = planet->hasEdits ? DistanceToEdits(planet, point) : INT64_MAX;
    if (toEdits > 0)
    {
        /* Away from edits the ball is exact: its distance is a safe step,
         * but never into the edited region. */
        *outStep = FoxMax(FoxMin(FoxAbs(analytic), toEdits), FOX_PLANET_RAY_MIN_STEP);
        return analytic;
    }
    const FoxPlanetSample sample = FoxPlanetSampleAt(planet, point);
    /* Clamped densities are a lower bound of the distance only near the
     * surface; half of it is safe for slopes. */
    *outStep = FoxMax(FoxAbs((int64_t)sample.density) / 2, FOX_PLANET_RAY_MIN_STEP);
    return sample.density;
}

bool FoxPlanetRaycast(const FoxPlanet *planet, FoxVec origin, FoxVec direction, int64_t maxDistance,
                      FoxVec *outHit)
{
    const FoxVec heading = FoxVecDirection(direction);
    const int64_t limit = INT64_C(1) << 40;
    if (planet == NULL || outHit == NULL || FoxVecIsZero(heading) || maxDistance <= 0 ||
        FoxAbs(origin.x) > limit || FoxAbs(origin.y) > limit || FoxAbs(origin.z) > limit)
    {
        return false;
    }
    /* Skip straight to the sphere that holds all ground. */
    const int64_t bound =
        FoxMax(planet->radius, planet->editReach) + FOX_PLANET_DENSITY_LIMIT + FOX_METRES(1);
    int64_t travelled = 0;
    const int64_t originDistance = FoxVecLength(origin);
    if (originDistance > bound)
    {
        /* Ray-sphere entry: t = -b - sqrt(b^2 - c) with b = o.d and c = |o|^2 - r^2,
         * in metres of float-free integer arithmetic scaled by the unit
         * direction. */
        const int64_t along = -FoxDivideRounded(FoxVecDot(origin, heading), FOX_UNIT);
        if (along <= 0)
        {
            return false;
        }
        const FoxVec closest = FoxVecAdd(origin, FoxVecScale(heading, along, FOX_UNIT));
        const int64_t miss = FoxVecLength(closest);
        if (miss > bound)
        {
            return false;
        }
        const int64_t half = (int64_t)FoxIntegerSqrt((uint64_t)(bound * bound - miss * miss));
        travelled = FoxMax(0, along - half);
        if (travelled >= maxDistance)
        {
            return false;
        }
    }
    FoxVec previous = FoxVecAdd(origin, FoxVecScale(heading, travelled, FOX_UNIT));
    int64_t step = 0;
    if (MarchDensity(planet, previous, &step) > 0)
    {
        *outHit = previous;
        return true;
    }
    int64_t previousTravelled = travelled;
    while (travelled < maxDistance)
    {
        travelled = FoxMin(travelled + step, maxDistance);
        const FoxVec point = FoxVecAdd(origin, FoxVecScale(heading, travelled, FOX_UNIT));
        if (FoxVecLength(point) > bound + FOX_METRES(2) && travelled > previousTravelled &&
            FoxVecDot(point, heading) > 0)
        {
            /* Left the ground sphere moving outwards. */
            return false;
        }
        if (MarchDensity(planet, point, &step) > 0)
        {
            /* Bisect the crossing to about a centimetre. */
            int64_t low = previousTravelled;
            int64_t high = travelled;
            while (high - low > FOX_METRES(1) / 100)
            {
                const int64_t middle = low + (high - low) / 2;
                int64_t unused = 0;
                const FoxVec probe = FoxVecAdd(origin, FoxVecScale(heading, middle, FOX_UNIT));
                if (MarchDensity(planet, probe, &unused) > 0)
                {
                    high = middle;
                }
                else
                {
                    low = middle;
                }
            }
            *outHit = FoxVecAdd(origin, FoxVecScale(heading, high, FOX_UNIT));
            return true;
        }
        previousTravelled = travelled;
        previous = point;
    }
    (void)previous;
    return false;
}

bool FoxPlanetGround(const FoxPlanet *planet, FoxVec point, FoxVec *outGround)
{
    if (planet == NULL || outGround == NULL)
    {
        return false;
    }
    const FoxVec up = FoxPlanetUp(point);
    const int64_t bound =
        FoxMax(planet->radius, planet->editReach) + FOX_PLANET_DENSITY_LIMIT + FOX_METRES(2);
    const FoxVec start = FoxVecScale(up, bound, FOX_UNIT);
    return FoxPlanetRaycast(planet, start, FoxVecNegate(up), bound * 2, outGround);
}

bool FoxPlanetBoxMayHaveSurface(const FoxPlanet *planet, FoxVec min, FoxVec max)
{
    if (planet == NULL || min.x > max.x || min.y > max.y || min.z > max.z)
    {
        return false;
    }
    const int64_t limit = INT64_C(1) << 40;
    if (FoxAbs(min.x) > limit || FoxAbs(min.y) > limit || FoxAbs(min.z) > limit ||
        FoxAbs(max.x) > limit || FoxAbs(max.y) > limit || FoxAbs(max.z) > limit)
    {
        return false;
    }
    const FoxVec nearest = {FoxClamp(0, min.x, max.x), FoxClamp(0, min.y, max.y),
                            FoxClamp(0, min.z, max.z)};
    const FoxVec farthest = {FoxMax(FoxAbs(min.x), FoxAbs(max.x)),
                             FoxMax(FoxAbs(min.y), FoxAbs(max.y)),
                             FoxMax(FoxAbs(min.z), FoxAbs(max.z))};
    const int64_t near = FoxVecLength(nearest);
    const int64_t far = FoxVecLength(farthest);
    const int64_t margin = FOX_PLANET_DENSITY_LIMIT + FOX_PLANET_CELL_UNITS;
    if (near <= planet->radius + margin && far >= planet->radius - margin)
    {
        return true;
    }
    if (!planet->hasEdits)
    {
        return false;
    }
    const int64_t first[3] = {FloorShift(min.x, FOX_PLANET_CELL_SHIFT) - 1,
                              FloorShift(min.y, FOX_PLANET_CELL_SHIFT) - 1,
                              FloorShift(min.z, FOX_PLANET_CELL_SHIFT) - 1};
    const int64_t last[3] = {FloorShift(max.x, FOX_PLANET_CELL_SHIFT) + 1,
                             FloorShift(max.y, FOX_PLANET_CELL_SHIFT) + 1,
                             FloorShift(max.z, FOX_PLANET_CELL_SHIFT) + 1};
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        if (last[axis] < planet->editMin[axis] || first[axis] > planet->editMax[axis])
        {
            return false;
        }
    }
    /* Any edited 32 m block overlapping the box may carry a surface. */
    for (uint32_t index = 0u; index < planet->topCapacity; ++index)
    {
        const Top *top = &planet->tops[index];
        if (top->node == NULL)
        {
            continue;
        }
        const int64_t topCoordinates[3] = {top->x, top->y, top->z};
        bool overlaps = true;
        for (uint32_t axis = 0u; overlaps && axis < 3u; ++axis)
        {
            const int64_t low = topCoordinates[axis] * TOP_CELLS;
            overlaps = last[axis] >= low && first[axis] < low + TOP_CELLS;
        }
        if (!overlaps)
        {
            continue;
        }
        for (uint32_t slot = 0u; slot < BLOCK_VOLUME; ++slot)
        {
            if (top->node->kind[slot] == SLOT_NONE)
            {
                continue;
            }
            const int64_t blockFirst[3] = {
                top->x * TOP_CELLS + (int64_t)(slot & 7u) * 64,
                top->y * TOP_CELLS + (int64_t)((slot >> 3u) & 7u) * 64,
                top->z * TOP_CELLS + (int64_t)(slot >> 6u) * 64,
            };
            if (last[0] >= blockFirst[0] && first[0] < blockFirst[0] + 64 &&
                last[1] >= blockFirst[1] && first[1] < blockFirst[1] + 64 &&
                last[2] >= blockFirst[2] && first[2] < blockFirst[2] + 64)
            {
                return true;
            }
        }
    }
    return false;
}

static uint64_t ValueHash(unsigned level, int64_t x, int64_t y, int64_t z, int32_t density,
                          uint8_t material)
{
    uint64_t hash = FoxHashCombine((uint64_t)level, (uint64_t)x);
    hash = FoxHashCombine(hash, (uint64_t)y);
    hash = FoxHashCombine(hash, (uint64_t)z);
    hash = FoxHashCombine(hash, (uint64_t)(uint32_t)density);
    return FoxHashCombine(hash, material);
}

uint64_t FoxPlanetHash(const FoxPlanet *planet)
{
    if (planet == NULL)
    {
        return 0u;
    }
    /* A sum is independent of the table order. */
    uint64_t sum = 0u;
    for (uint32_t index = 0u; index < planet->topCapacity; ++index)
    {
        const Top *top = &planet->tops[index];
        if (top->node == NULL)
        {
            continue;
        }
        for (uint32_t upperSlot = 0u; upperSlot < BLOCK_VOLUME; ++upperSlot)
        {
            const Inner *upper = top->node;
            const int64_t ux = top->x * TOP_CELLS + (int64_t)(upperSlot & 7u) * 64;
            const int64_t uy = top->y * TOP_CELLS + (int64_t)((upperSlot >> 3u) & 7u) * 64;
            const int64_t uz = top->z * TOP_CELLS + (int64_t)(upperSlot >> 6u) * 64;
            if (upper->kind[upperSlot] == SLOT_VALUE)
            {
                sum += ValueHash(2u, ux, uy, uz, upper->density[upperSlot],
                                 upper->material[upperSlot]);
                continue;
            }
            if (upper->kind[upperSlot] != SLOT_CHILD)
            {
                continue;
            }
            const Inner *middle = (const Inner *)upper->child[upperSlot];
            for (uint32_t middleSlot = 0u; middleSlot < BLOCK_VOLUME; ++middleSlot)
            {
                const int64_t mx = ux + (int64_t)(middleSlot & 7u) * 8;
                const int64_t my = uy + (int64_t)((middleSlot >> 3u) & 7u) * 8;
                const int64_t mz = uz + (int64_t)(middleSlot >> 6u) * 8;
                if (middle->kind[middleSlot] == SLOT_VALUE)
                {
                    sum += ValueHash(1u, mx, my, mz, middle->density[middleSlot],
                                     middle->material[middleSlot]);
                    continue;
                }
                if (middle->kind[middleSlot] != SLOT_CHILD)
                {
                    continue;
                }
                const Leaf *leaf = (const Leaf *)middle->child[middleSlot];
                for (uint32_t cell = 0u; cell < BLOCK_VOLUME; ++cell)
                {
                    if ((leaf->edited[cell / 64u] & (UINT64_C(1) << (cell % 64u))) != 0u)
                    {
                        sum += ValueHash(0u, mx + (int64_t)(cell & 7u),
                                         my + (int64_t)((cell >> 3u) & 7u),
                                         mz + (int64_t)(cell >> 6u), leaf->density[cell],
                                         leaf->material[cell]);
                    }
                }
            }
        }
    }
    return FoxHashCombine(sum, (uint64_t)planet->radius);
}
