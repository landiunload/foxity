#include "client/planet_view.h"

#include "app/log.h"
#include "mod/application.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Lattice samples spanned by one cube. Meshing reads two samples beyond it on
 * each side and emits one ring of cells past its border: neighbours of the
 * same level draw that ring identically, and against a neighbour of another
 * level it covers the crack the two resolutions would leave. */
#define CUBE_CELLS 32
#define BORDER_SAMPLES 2
#define SAMPLES (CUBE_CELLS + 2 * BORDER_SAMPLES)
#define MAX_SAMPLES ((uint32_t)(SAMPLES * SAMPLES * SAMPLES))
#define MAX_CELLS ((uint32_t)((SAMPLES - 1) * (SAMPLES - 1) * (SAMPLES - 1)))
#define MAX_SURFACE_VERTICES MAX_CELLS
#define MAX_SURFACE_INDICES (MAX_CELLS * 6u)
/* The coarsest cube spans 2 km; eight of them hold the planet. */
#define MAX_LEVEL 7
/* A cube is split while the eye is closer than this many cube widths. */
#define SPLIT_FACTOR 2.0
#define MAX_VISIBLE 2048u
/* Each cached cube holds up to two GPU handles; the device has 4096 and the
 * roads and cars need some. */
#define CACHE_CAPACITY 1000u
#define HASH_CAPACITY 4096u
/* A missing leaf is first covered by its ancestor this many levels up. */
#define COVER_LEVELS 2

typedef struct CubeKey
{
    int32_t level;
    int64_t x;
    int64_t y;
    int64_t z;
} CubeKey;

typedef struct Cube
{
    CubeKey key;
    bool used;
    bool empty;
    bool indexed;
    bool stale;
    LaiueGraphicsHandle vertexBuffer;
    LaiueGraphicsHandle indexBuffer;
    uint32_t elementCount;
    FoxVec origin;
    uint64_t lastUsedFrame;
    uint64_t drawnFrame;
} Cube;

typedef struct Leaf
{
    CubeKey key;
    double distance;
} Leaf;

struct FoxPlanetView
{
    LaiueGraphicsDeviceV2 *device;
    uint32_t (*buildSurfaceMesh)(LaiueSurfaceMeshRequestV1 *request);
    const LaiueSceneMathServiceV1 *math;
    bool nativeIndices;
    uint64_t frame;
    Cube cubes[CACHE_CAPACITY];
    int32_t hash[HASH_CAPACITY];
    Leaf leaves[MAX_VISIBLE];
    uint32_t leafCount;
    uint32_t drawList[MAX_VISIBLE];
    uint32_t drawCount;
    FoxPlanetViewStats stats;
    FoxPlanetSample *samples;
    float *values;
    uint8_t *materials;
    uint32_t *cellScratch;
    LaiueSurfaceVertexV1 *surfaceVertices;
    uint32_t *surfaceIndices;
    LaiueGraphicsVertexV2 *vertices;
};

static bool DeviceHas(const LaiueGraphicsDeviceV2 *device, size_t offset, size_t size)
{
    return device != NULL && (size_t)device->structSize >= offset + size;
}

FoxPlanetView *FoxPlanetViewCreate(LaiueGraphicsDeviceV2 *device, const LaiueMesherServiceV1 *mesher,
                                   uint32_t mesherServiceSize, const LaiueSceneMathServiceV1 *math)
{
    if (device == NULL || mesher == NULL || math == NULL ||
        mesherServiceSize < LAIUE_MESHER_SERVICE_V1_SURFACE_SIZE ||
        mesher->structSize < LAIUE_MESHER_SERVICE_V1_SURFACE_SIZE ||
        mesher->buildSurfaceMesh == NULL || math->frustumIntersectsBox == NULL ||
        !DeviceHas(device, offsetof(LaiueGraphicsDeviceV2, destroyHandle),
                   sizeof(device->destroyHandle)) ||
        device->createBuffer == NULL || device->uploadBuffer == NULL ||
        device->destroyHandle == NULL || device->submit == NULL)
    {
        return NULL;
    }
    FoxPlanetView *view = calloc(1u, sizeof(*view));
    if (view == NULL)
    {
        return NULL;
    }
    view->device = device;
    view->buildSurfaceMesh = mesher->buildSurfaceMesh;
    view->math = math;
    view->nativeIndices =
        DeviceHas(device, offsetof(LaiueGraphicsDeviceV2, getCapabilities),
                  sizeof(device->getCapabilities)) &&
        device->getCapabilities != NULL &&
        (device->getCapabilities(device) & LAIUE_GRAPHICS_CAP_NATIVE_INDICES) != 0u;
    view->samples = malloc((size_t)MAX_SAMPLES * sizeof(*view->samples));
    view->values = malloc((size_t)MAX_SAMPLES * sizeof(*view->values));
    view->materials = malloc((size_t)MAX_SAMPLES * sizeof(*view->materials));
    view->cellScratch = malloc((size_t)MAX_CELLS * sizeof(*view->cellScratch));
    view->surfaceVertices = malloc((size_t)MAX_SURFACE_VERTICES * sizeof(*view->surfaceVertices));
    view->surfaceIndices = malloc((size_t)MAX_SURFACE_INDICES * sizeof(*view->surfaceIndices));
    view->vertices = malloc((size_t)MAX_SURFACE_INDICES * sizeof(*view->vertices));
    if (view->samples == NULL || view->values == NULL || view->materials == NULL ||
        view->cellScratch == NULL || view->surfaceVertices == NULL ||
        view->surfaceIndices == NULL || view->vertices == NULL)
    {
        FoxPlanetViewDestroy(view);
        return NULL;
    }
    return view;
}

static void ReleaseCube(FoxPlanetView *view, Cube *cube)
{
    if (cube->vertexBuffer != 0u)
    {
        view->device->destroyHandle(view->device, cube->vertexBuffer);
    }
    if (cube->indexBuffer != 0u)
    {
        view->device->destroyHandle(view->device, cube->indexBuffer);
    }
    memset(cube, 0, sizeof(*cube));
}

void FoxPlanetViewDestroy(FoxPlanetView *view)
{
    if (view == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < CACHE_CAPACITY; ++index)
    {
        if (view->cubes[index].used)
        {
            ReleaseCube(view, &view->cubes[index]);
        }
    }
    free(view->samples);
    free(view->values);
    free(view->materials);
    free(view->cellScratch);
    free(view->surfaceVertices);
    free(view->surfaceIndices);
    free(view->vertices);
    free(view);
}

static uint32_t HashKey(CubeKey key)
{
    uint64_t value = (uint64_t)key.x * UINT64_C(0x9E3779B97F4A7C15);
    value ^= (uint64_t)key.y * UINT64_C(0xC2B2AE3D27D4EB4F);
    value ^= (uint64_t)key.z * UINT64_C(0x27D4EB2F165667C5);
    value ^= (uint64_t)(uint32_t)key.level * UINT64_C(0x165667B19E3779F9);
    value ^= value >> 29;
    return (uint32_t)value & (HASH_CAPACITY - 1u);
}

static bool SameKey(CubeKey a, CubeKey b)
{
    return a.level == b.level && a.x == b.x && a.y == b.y && a.z == b.z;
}

/* The table is rebuilt from the cache every frame: with at most a thousand
 * cubes that is cheaper than maintaining deletions. */
static void RebuildHash(FoxPlanetView *view)
{
    for (uint32_t slot = 0u; slot < HASH_CAPACITY; ++slot)
    {
        view->hash[slot] = -1;
    }
    for (uint32_t index = 0u; index < CACHE_CAPACITY; ++index)
    {
        if (!view->cubes[index].used)
        {
            continue;
        }
        uint32_t slot = HashKey(view->cubes[index].key);
        while (view->hash[slot] >= 0)
        {
            slot = (slot + 1u) & (HASH_CAPACITY - 1u);
        }
        view->hash[slot] = (int32_t)index;
    }
}

static Cube *FindCube(FoxPlanetView *view, CubeKey key)
{
    uint32_t slot = HashKey(key);
    while (view->hash[slot] >= 0)
    {
        Cube *cube = &view->cubes[view->hash[slot]];
        if (SameKey(cube->key, key))
        {
            return cube;
        }
        slot = (slot + 1u) & (HASH_CAPACITY - 1u);
    }
    return NULL;
}

/* Cells spanned by a cube of a level. */
static int64_t CubeSpan(int32_t level)
{
    return (int64_t)CUBE_CELLS << level;
}

/* The world box a cube's samples cover, borders included. */
static void CubeBox(CubeKey key, FoxVec *outMin, FoxVec *outMax)
{
    const int64_t span = CubeSpan(key.level);
    const int64_t border = ((int64_t)BORDER_SAMPLES + 1) << key.level;
    *outMin = (FoxVec){(key.x * span - border) * FOX_PLANET_CELL_UNITS,
                       (key.y * span - border) * FOX_PLANET_CELL_UNITS,
                       (key.z * span - border) * FOX_PLANET_CELL_UNITS};
    *outMax = (FoxVec){((key.x + 1) * span + border) * FOX_PLANET_CELL_UNITS,
                       ((key.y + 1) * span + border) * FOX_PLANET_CELL_UNITS,
                       ((key.z + 1) * span + border) * FOX_PLANET_CELL_UNITS};
}

static void AddLeaf(FoxPlanetView *view, CubeKey key, double distance)
{
    if (view->leafCount < MAX_VISIBLE)
    {
        view->leaves[view->leafCount++] = (Leaf){.key = key, .distance = distance};
    }
}

static double BoxDistance(const double eye[3], FoxVec min, FoxVec max)
{
    const double low[3] = {(double)min.x, (double)min.y, (double)min.z};
    const double high[3] = {(double)max.x, (double)max.y, (double)max.z};
    double squared = 0.0;
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        const double outside = fmax(0.0, fmax(low[axis] - eye[axis], eye[axis] - high[axis]));
        squared += outside * outside;
    }
    return sqrt(squared) / FOX_UNITS_PER_METRE;
}

/* Walks the octree below one root cube: a cube near the eye splits into its
 * eight children, a far one becomes a leaf. Cubes without surface and cubes
 * outside the view are left out. */
static void Visit(FoxPlanetView *view, const FoxPlanet *planet, const FoxCameraView *camera,
                  CubeKey root, const double eye[3])
{
    CubeKey stack[8 * (MAX_LEVEL + 1)];
    uint32_t depth = 0u;
    stack[depth++] = root;
    while (depth != 0u)
    {
        const CubeKey key = stack[--depth];
        FoxVec min;
        FoxVec max;
        CubeBox(key, &min, &max);
        if (!FoxPlanetBoxMayHaveSurface(planet, min, max))
        {
            continue;
        }
        float relativeMin[3];
        float relativeMax[3];
        FoxCameraRelative(camera, min, relativeMin);
        FoxCameraRelative(camera, max, relativeMax);
        if (!view->math->frustumIntersectsBox(camera->planes, relativeMin, relativeMax))
        {
            continue;
        }
        const double size = (double)CubeSpan(key.level) * 0.5;
        const double distance = BoxDistance(eye, min, max);
        if (key.level == 0 || distance >= SPLIT_FACTOR * size)
        {
            AddLeaf(view, key, distance);
            continue;
        }
        for (int64_t child = 0; child < 8; ++child)
        {
            stack[depth++] = (CubeKey){
                .level = key.level - 1,
                .x = key.x * 2 + (child & 1),
                .y = key.y * 2 + ((child >> 1) & 1),
                .z = key.z * 2 + (child >> 2),
            };
        }
    }
}

static int CompareLeaves(const void *left, const void *right)
{
    const double a = ((const Leaf *)left)->distance;
    const double b = ((const Leaf *)right)->distance;
    return (a > b) - (a < b);
}

static void SelectLeaves(FoxPlanetView *view, const FoxPlanet *planet, const FoxCameraView *camera)
{
    const double eye[3] = {
        (double)camera->renderOrigin.x + (double)camera->eye[0] * FOX_UNITS_PER_METRE,
        (double)camera->renderOrigin.y + (double)camera->eye[1] * FOX_UNITS_PER_METRE,
        (double)camera->renderOrigin.z + (double)camera->eye[2] * FOX_UNITS_PER_METRE,
    };
    view->leafCount = 0u;
    for (int64_t z = -1; z <= 0; ++z)
    {
        for (int64_t y = -1; y <= 0; ++y)
        {
            for (int64_t x = -1; x <= 0; ++x)
            {
                Visit(view, planet, camera, (CubeKey){.level = MAX_LEVEL, .x = x, .y = y, .z = z},
                      eye);
            }
        }
    }
    qsort(view->leaves, view->leafCount, sizeof(view->leaves[0]), CompareLeaves);
}

static uint32_t PackColor(float red, float green, float blue)
{
    const float channels[3] = {red, green, blue};
    uint32_t packed = UINT32_C(0xFF000000);
    for (uint32_t index = 0u; index < 3u; ++index)
    {
        const float value =
            channels[index] < 0.0f ? 0.0f : (channels[index] > 1.0f ? 1.0f : channels[index]);
        packed |= (uint32_t)lrintf(value * 255.0f) << (8u * index);
    }
    return packed;
}

/* Mock-up shading baked into vertex colours: always day, with a sun that
 * stands at the same place in every town's sky, so the whole planet reads
 * the same. Steep grass shows bare earth. */
static uint32_t ShadeVertex(const LaiueSurfaceVertexV1 *vertex, const double world[3],
                            double radiusMetres)
{
    static const float palette[FOX_MATERIAL_COUNT][3] = {
        [FOX_MATERIAL_AIR] = {1.0f, 0.0f, 1.0f},     [FOX_MATERIAL_GRASS] = {0.36f, 0.58f, 0.25f},
        [FOX_MATERIAL_DIRT] = {0.47f, 0.36f, 0.24f}, [FOX_MATERIAL_STONE] = {0.52f, 0.51f, 0.50f},
        [FOX_MATERIAL_CORE] = {0.86f, 0.36f, 0.12f},
    };
    uint32_t material = vertex->material < FOX_MATERIAL_COUNT ? vertex->material : 0u;
    /* Coarse cubes take the material of a corner metres below the surface;
     * the skin of the planet is grass whatever corner was picked. */
    const double depth = radiusMetres - sqrt(world[0] * world[0] + world[1] * world[1] +
                                             world[2] * world[2]);
    if ((material == FOX_MATERIAL_DIRT || material == FOX_MATERIAL_STONE) && depth < 2.0)
    {
        material = FOX_MATERIAL_GRASS;
    }
    float base[3] = {palette[material][0], palette[material][1], palette[material][2]};
    double up[3] = {world[0], world[1], world[2]};
    const double length = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    if (length > 0.0)
    {
        up[0] /= length;
        up[1] /= length;
        up[2] /= length;
    }
    else
    {
        up[2] = 1.0;
    }
    /* East and north of the place, for a sun in its south-east. */
    double east[3] = {-up[1], up[0], 0.0};
    const double eastLength = sqrt(east[0] * east[0] + east[1] * east[1]);
    if (eastLength < 1e-6)
    {
        east[0] = 1.0;
        east[1] = 0.0;
    }
    else
    {
        east[0] /= eastLength;
        east[1] /= eastLength;
    }
    const double north[3] = {up[1] * east[2] - up[2] * east[1], up[2] * east[0] - up[0] * east[2],
                             up[0] * east[1] - up[1] * east[0]};
    const float upward = (float)(vertex->normal[0] * up[0] + vertex->normal[1] * up[1] +
                                 vertex->normal[2] * up[2]);
    if (material == FOX_MATERIAL_GRASS && upward < 0.75f)
    {
        const float rock = upward < 0.55f ? 1.0f : (0.75f - upward) / 0.2f;
        for (uint32_t channel = 0u; channel < 3u; ++channel)
        {
            base[channel] += (palette[FOX_MATERIAL_DIRT][channel] - base[channel]) * rock;
        }
    }
    double sun[3];
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        sun[axis] = 0.78 * up[axis] + 0.42 * east[axis] - 0.46 * north[axis];
    }
    const double sunLength = sqrt(sun[0] * sun[0] + sun[1] * sun[1] + sun[2] * sun[2]);
    const float lambert = (float)((vertex->normal[0] * sun[0] + vertex->normal[1] * sun[1] +
                                   vertex->normal[2] * sun[2]) /
                                  sunLength);
    /* A faint light shared by the whole planet rounds the ball seen from afar
     * without ever leaving a side in the dark. */
    static const double globe[3] = {0.48, -0.36, 0.80};
    const float round = (float)(up[0] * globe[0] + up[1] * globe[1] + up[2] * globe[2]);
    const float light = (0.45f + 0.55f * (lambert > 0.0f ? lambert : 0.0f)) *
                        (0.80f + 0.20f * (round > 0.0f ? round : 0.0f));
    return PackColor(base[0] * light, base[1] * light, base[2] * light);
}

static bool Upload(FoxPlanetView *view, uint32_t usage, const void *data, uint64_t size,
                   LaiueGraphicsHandle *outHandle)
{
    const LaiueGraphicsBufferDescV1 description = {
        .structSize = sizeof(description),
        .usageFlags = usage,
        .sizeBytes = size,
    };
    *outHandle = 0u;
    if (view->device->createBuffer(view->device, &description, outHandle) == 0u)
    {
        return false;
    }
    const LaiueGraphicsBufferUploadV1 upload = {
        .structSize = sizeof(upload),
        .buffer = *outHandle,
        .data = data,
        .sizeBytes = size,
    };
    if (view->device->uploadBuffer(view->device, &upload) == 0u)
    {
        view->device->destroyHandle(view->device, *outHandle);
        *outHandle = 0u;
        return false;
    }
    return true;
}

/* Samples, meshes and uploads one cube. False only when it could not be
 * stored; a cube without surface is a success. */
static bool BuildCube(FoxPlanetView *view, const FoxPlanet *planet, Cube *cube)
{
    const int64_t stride = INT64_C(1) << cube->key.level;
    const int64_t border = (int64_t)BORDER_SAMPLES * stride;
    const FoxPlanetGrid grid = {
        .cellX = cube->key.x * CubeSpan(cube->key.level) - border,
        .cellY = cube->key.y * CubeSpan(cube->key.level) - border,
        .cellZ = cube->key.z * CubeSpan(cube->key.level) - border,
        .stride = (uint32_t)stride,
        .countX = SAMPLES,
        .countY = SAMPLES,
        .countZ = SAMPLES,
    };
    if (!FoxPlanetSampleGrid(planet, &grid, view->samples))
    {
        return false;
    }
    for (uint32_t index = 0u; index < MAX_SAMPLES; ++index)
    {
        view->values[index] = (float)view->samples[index].density;
        view->materials[index] = view->samples[index].material;
    }
    LaiueSurfaceMeshRequestV1 request = {
        .structSize = sizeof(request),
        .countX = SAMPLES,
        .countY = SAMPLES,
        .countZ = SAMPLES,
        .values = view->values,
        .materials = view->materials,
        .emitMin = {BORDER_SAMPLES - 1u, BORDER_SAMPLES - 1u, BORDER_SAMPLES - 1u},
        .emitMax = {CUBE_CELLS + BORDER_SAMPLES + 1u, CUBE_CELLS + BORDER_SAMPLES + 1u,
                    CUBE_CELLS + BORDER_SAMPLES + 1u},
        .cellScratch = view->cellScratch,
        .cellScratchCapacity = MAX_CELLS,
        .vertices = view->surfaceVertices,
        .vertexCapacity = MAX_SURFACE_VERTICES,
        .indices = view->surfaceIndices,
        .indexCapacity = MAX_SURFACE_INDICES,
    };
    if (view->buildSurfaceMesh(&request) != LAIUE_SURFACE_MESH_OK)
    {
        return false;
    }
    cube->origin = (FoxVec){grid.cellX * FOX_PLANET_CELL_UNITS, grid.cellY * FOX_PLANET_CELL_UNITS,
                            grid.cellZ * FOX_PLANET_CELL_UNITS};
    if (request.indexCount == 0u)
    {
        cube->empty = true;
        return true;
    }
    const float metresPerSample = (float)stride * 0.5f;
    const double origin[3] = {(double)cube->origin.x / FOX_UNITS_PER_METRE,
                              (double)cube->origin.y / FOX_UNITS_PER_METRE,
                              (double)cube->origin.z / FOX_UNITS_PER_METRE};
    const uint32_t count = view->nativeIndices ? request.vertexCount : request.indexCount;
    for (uint32_t index = 0u; index < count; ++index)
    {
        const LaiueSurfaceVertexV1 *source =
            &view->surfaceVertices[view->nativeIndices ? index : view->surfaceIndices[index]];
        const float local[3] = {source->position[0] * metresPerSample,
                                source->position[1] * metresPerSample,
                                source->position[2] * metresPerSample};
        const double world[3] = {origin[0] + local[0], origin[1] + local[1], origin[2] + local[2]};
        view->vertices[index] = (LaiueGraphicsVertexV2){
            .position = {local[0], local[1], local[2]},
            .colorRGBA = ShadeVertex(source, world,
                                     (double)FoxPlanetRadius(planet) / FOX_UNITS_PER_METRE),
        };
    }
    cube->elementCount = request.indexCount;
    if (view->nativeIndices)
    {
        cube->indexed = true;
        return Upload(view, LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX, view->vertices,
                      (uint64_t)request.vertexCount * sizeof(*view->vertices), &cube->vertexBuffer) &&
               Upload(view, LAIUE_GRAPHICS_BUFFER_USAGE_INDEX, view->surfaceIndices,
                      (uint64_t)request.indexCount * sizeof(uint32_t), &cube->indexBuffer);
    }
    return Upload(view, LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX, view->vertices,
                  (uint64_t)request.indexCount * sizeof(*view->vertices), &cube->vertexBuffer);
}

void FoxPlanetViewInvalidate(FoxPlanetView *view, const FoxPlanetBox *box)
{
    if (view == NULL || box == NULL || box->min.x > box->max.x)
    {
        return;
    }
    for (uint32_t index = 0u; index < CACHE_CAPACITY; ++index)
    {
        Cube *cube = &view->cubes[index];
        if (!cube->used)
        {
            continue;
        }
        FoxVec min;
        FoxVec max;
        CubeBox(cube->key, &min, &max);
        if (box->min.x <= max.x && box->max.x >= min.x && box->min.y <= max.y &&
            box->max.y >= min.y && box->min.z <= max.z && box->max.z >= min.z)
        {
            cube->stale = true;
        }
    }
}

/* Rebuilds a stale cube in place; the old mesh stays until the new one is
 * ready, and for good if the build fails. */
static bool Refresh(FoxPlanetView *view, const FoxPlanet *planet, Cube *cube)
{
    Cube fresh = *cube;
    fresh.vertexBuffer = 0u;
    fresh.indexBuffer = 0u;
    fresh.elementCount = 0u;
    fresh.empty = false;
    fresh.indexed = false;
    fresh.stale = false;
    cube->stale = false;
    if (!BuildCube(view, planet, &fresh))
    {
        FoxLog(FOX_LOG_WARNING, "planet cube level %d could not be rebuilt", (int)cube->key.level);
        ReleaseCube(view, &fresh);
        return false;
    }
    if (cube->vertexBuffer != 0u)
    {
        view->device->destroyHandle(view->device, cube->vertexBuffer);
    }
    if (cube->indexBuffer != 0u)
    {
        view->device->destroyHandle(view->device, cube->indexBuffer);
    }
    *cube = fresh;
    return true;
}

static Cube *AcquireCube(FoxPlanetView *view)
{
    Cube *oldest = NULL;
    for (uint32_t index = 0u; index < CACHE_CAPACITY; ++index)
    {
        Cube *cube = &view->cubes[index];
        if (!cube->used)
        {
            return cube;
        }
        if (cube->lastUsedFrame < view->frame &&
            (oldest == NULL || cube->lastUsedFrame < oldest->lastUsedFrame))
        {
            oldest = cube;
        }
    }
    if (oldest != NULL)
    {
        ReleaseCube(view, oldest);
    }
    return oldest;
}

static void MarkForDrawing(FoxPlanetView *view, Cube *cube)
{
    cube->lastUsedFrame = view->frame;
    if (cube->drawnFrame == view->frame)
    {
        return;
    }
    cube->drawnFrame = view->frame;
    if (!cube->empty && view->drawCount < MAX_VISIBLE)
    {
        view->drawList[view->drawCount++] = (uint32_t)(cube - view->cubes);
    }
}

static int64_t Half(int64_t value)
{
    return value >= 0 ? value / 2 : -((-value + 1) / 2);
}

static CubeKey ParentKey(CubeKey key)
{
    return (CubeKey){.level = key.level + 1, .x = Half(key.x), .y = Half(key.y), .z = Half(key.z)};
}

static Cube *CachedAncestor(FoxPlanetView *view, CubeKey key)
{
    for (CubeKey parent = key; parent.level < MAX_LEVEL;)
    {
        parent = ParentKey(parent);
        Cube *ancestor = FindCube(view, parent);
        if (ancestor != NULL)
        {
            return ancestor;
        }
    }
    return NULL;
}

static Cube *Build(FoxPlanetView *view, const FoxPlanet *planet, CubeKey key)
{
    if (FindCube(view, key) != NULL)
    {
        return NULL;
    }
    Cube *cube = AcquireCube(view);
    if (cube == NULL)
    {
        return NULL;
    }
    cube->key = key;
    cube->used = true;
    cube->lastUsedFrame = view->frame;
    if (!BuildCube(view, planet, cube))
    {
        FoxLog(FOX_LOG_WARNING, "planet cube level %d could not be built", (int)key.level);
        ReleaseCube(view, cube);
        cube = NULL;
    }
    RebuildHash(view);
    return cube;
}

void FoxPlanetViewUpdate(FoxPlanetView *view, const FoxPlanet *planet, const FoxCameraView *camera,
                         double budgetSeconds)
{
    if (view == NULL || planet == NULL || camera == NULL)
    {
        return;
    }
    const double started = LaiueApplicationMonotonicSeconds();
    ++view->frame;
    view->drawCount = 0u;
    view->stats = (FoxPlanetViewStats){0};
    RebuildHash(view);
    SelectLeaves(view, planet, camera);

    /* Within the budget, but always one build, so the view keeps coming even
     * on a slow machine. */
    uint32_t built = 0u;
#define WITHIN_BUDGET()                                                                            \
    (built == 0u || LaiueApplicationMonotonicSeconds() - started < budgetSeconds)

    /* Changed ground first: the player is looking at it. */
    for (uint32_t index = 0u; index < view->leafCount && WITHIN_BUDGET(); ++index)
    {
        Cube *cube = FindCube(view, view->leaves[index].key);
        if (cube != NULL && cube->stale)
        {
            built += Refresh(view, planet, cube) ? 1u : 0u;
        }
    }
    /* Coverage, then detail nearest first. */
    for (uint32_t index = 0u; index < view->leafCount && WITHIN_BUDGET(); ++index)
    {
        const CubeKey key = view->leaves[index].key;
        if (FindCube(view, key) == NULL && CachedAncestor(view, key) == NULL)
        {
            CubeKey cover = key;
            for (int32_t step = 0; step < COVER_LEVELS && cover.level < MAX_LEVEL; ++step)
            {
                cover = ParentKey(cover);
            }
            built += Build(view, planet, cover) != NULL ? 1u : 0u;
        }
    }
    for (uint32_t index = 0u; index < view->leafCount; ++index)
    {
        const CubeKey key = view->leaves[index].key;
        Cube *cube = FindCube(view, key);
        if (cube == NULL && WITHIN_BUDGET())
        {
            cube = Build(view, planet, key);
            ++built;
        }
        cube = cube != NULL ? cube : CachedAncestor(view, key);
        if (cube != NULL)
        {
            MarkForDrawing(view, cube);
        }
    }
#undef WITHIN_BUDGET
    view->stats.visibleCubes = view->leafCount;
    view->stats.builtThisFrame = built;
    for (uint32_t index = 0u; index < CACHE_CAPACITY; ++index)
    {
        view->stats.cachedCubes += view->cubes[index].used ? 1u : 0u;
    }
}

void FoxPlanetViewDraw(FoxPlanetView *view, const FoxCameraView *camera)
{
    if (view == NULL || camera == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < view->drawCount; ++index)
    {
        const Cube *cube = &view->cubes[view->drawList[index]];
        LaiueGraphicsDrawItemV2 draw = {
            .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
            .vertexBuffer = cube->vertexBuffer,
            .indexBuffer = cube->indexed ? cube->indexBuffer : 0u,
            .indexCount = cube->elementCount,
            .scale = 1.0f,
        };
        FoxCameraRelative(camera, cube->origin, draw.originRelative);
        view->stats.drawnCubes += view->device->submit(view->device, &draw, 1u) != 0u ? 1u : 0u;
    }
}

FoxPlanetViewStats FoxPlanetViewGetStats(const FoxPlanetView *view)
{
    return view == NULL ? (FoxPlanetViewStats){0} : view->stats;
}
