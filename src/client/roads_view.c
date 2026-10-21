#include "client/roads_view.h"

#include "app/log.h"
#include "client/road_mesh.h"
#include "mod/application.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PATCH_SHIFT 17u
#define NO_PATCH UINT32_MAX
/* Details of patches whose middle is further than this from the eye are left
 * out. */
#define DETAIL_DISTANCE 700.0
/* A lit lamp: a small bright box over the dark one. */
#define LAMP_VERTICES 36u
#define LAMP_HALF 0.11f

/* A road that can be built in the accent blue of the interface, one that
 * cannot in red. */
#define COLOR_PREVIEW UINT32_C(0xFFFF8A1A)
#define COLOR_PREVIEW_BAD UINT32_C(0xFF3C46F0)
/* The width of the tools' lines and rings, metres. */
#define OVERLAY_LINE_HALF 0.12
#define OVERLAY_RING_WIDTH 0.35

/* Red, yellow and green, lit. */
static const uint32_t lampColors[3] = {UINT32_C(0xFF2828FF), UINT32_C(0xFF14B4FF),
                                       UINT32_C(0xFF46E650)};

typedef struct Patch
{
    bool used;
    bool dirty;
    int64_t x;
    int64_t y;
    int64_t z;
    FoxVec origin;
    uint32_t *roads;
    uint32_t roadCount;
    uint32_t roadCapacity;
    uint32_t *nodes;
    uint32_t nodeCount;
    uint32_t nodeCapacity;
    LaiueGraphicsHandle baseBuffer;
    uint32_t baseCount;
    LaiueGraphicsHandle detailBuffer;
    uint32_t detailCount;
    FoxSignalHead *heads;
    uint32_t headCount;
} Patch;

typedef struct Tracked
{
    bool present;
    uint64_t revision;
    uint32_t patch;
} Tracked;

typedef struct LampBatch
{
    LaiueGraphicsInstanceV2 *instances;
    uint32_t count;
    uint32_t capacity;
} LampBatch;

struct FoxRoadsView
{
    LaiueGraphicsDeviceV2 *device;
    uint64_t seenRevision;
    Patch *patches;
    Tracked *roads;
    Tracked *nodes;
    /* The planned road and the tools' lines, each with its origin. */
    LaiueGraphicsHandle previewBuffer;
    LaiueGraphicsHandle overlayBuffer;
    FoxMesh base;
    FoxMesh detail;
    FoxSignalHeads heads;
    FoxVec previewOrigin;
    FoxVec overlayOrigin;
    LaiueGraphicsHandle lamps[3];
    LampBatch lampBatches[3];
    uint32_t patchCapacity;
    uint32_t roadCapacity;
    uint32_t nodeCapacity;
    uint32_t previewCount;
    uint32_t overlayCount;
    bool instancing;
    bool seenAny;
};

static bool UploadVertices(LaiueGraphicsDeviceV2 *device, const LaiueGraphicsVertexV2 *vertices,
                           uint32_t count, LaiueGraphicsHandle *outHandle)
{
    const uint64_t bytes = (uint64_t)count * sizeof(*vertices);
    const LaiueGraphicsBufferDescV1 description = {
        .structSize = sizeof(description),
        .usageFlags = LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX,
        .sizeBytes = bytes,
    };
    LaiueGraphicsHandle handle = 0u;
    if (device->createBuffer(device, &description, &handle) == 0u)
    {
        return false;
    }
    const LaiueGraphicsBufferUploadV1 upload = {
        .structSize = sizeof(upload),
        .buffer = handle,
        .data = vertices,
        .sizeBytes = bytes,
    };
    if (device->uploadBuffer(device, &upload) == 0u)
    {
        device->destroyHandle(device, handle);
        return false;
    }
    *outHandle = handle;
    return true;
}

/* A cube of lit lamp around the origin. */
static bool CreateLamp(LaiueGraphicsDeviceV2 *device, uint32_t color, LaiueGraphicsHandle *out)
{
    LaiueGraphicsVertexV2 vertices[LAMP_VERTICES];
    static const uint8_t faces[6][4] = {
        {4u, 5u, 7u, 6u}, {0u, 2u, 3u, 1u}, {0u, 1u, 5u, 4u},
        {2u, 6u, 7u, 3u}, {0u, 4u, 6u, 2u}, {1u, 3u, 7u, 5u},
    };
    static const uint8_t order[6] = {0u, 1u, 2u, 0u, 2u, 3u};
    uint32_t written = 0u;
    for (uint32_t face = 0u; face < 6u; ++face)
    {
        for (uint32_t index = 0u; index < 6u; ++index)
        {
            const uint32_t corner = faces[face][order[index]];
            vertices[written++] = (LaiueGraphicsVertexV2){
                .position = {(corner & 1u) != 0u ? LAMP_HALF : -LAMP_HALF,
                             (corner & 2u) != 0u ? LAMP_HALF : -LAMP_HALF,
                             (corner & 4u) != 0u ? LAMP_HALF : -LAMP_HALF},
                .colorRGBA = color,
            };
        }
    }
    return UploadVertices(device, vertices, written, out);
}

FoxRoadsView *FoxRoadsViewCreate(LaiueGraphicsDeviceV2 *device)
{
    if (device == NULL || device->createBuffer == NULL || device->uploadBuffer == NULL ||
        device->destroyHandle == NULL || device->submit == NULL)
    {
        return NULL;
    }
    FoxRoadsView *view = calloc(1u, sizeof(*view));
    if (view == NULL)
    {
        return NULL;
    }
    view->device = device;
    view->instancing =
        device->structSize >= LAIUE_GRAPHICS_DEVICE_V2_INSTANCES_SIZE &&
        device->getCapabilities != NULL && device->submitInstances != NULL &&
        (device->getCapabilities(device) & LAIUE_GRAPHICS_CAP_GENERIC_INSTANCES) != 0u;
    for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
    {
        if (!CreateLamp(device, lampColors[lamp], &view->lamps[lamp]))
        {
            view->lamps[lamp] = 0u;
            FoxRoadsViewDestroy(view);
            return NULL;
        }
    }
    return view;
}

static void ReleaseBuffer(FoxRoadsView *view, LaiueGraphicsHandle *handle)
{
    if (*handle != 0u)
    {
        view->device->destroyHandle(view->device, *handle);
        *handle = 0u;
    }
}

void FoxRoadsViewDestroy(FoxRoadsView *view)
{
    if (view == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < view->patchCapacity; ++index)
    {
        Patch *patch = &view->patches[index];
        ReleaseBuffer(view, &patch->baseBuffer);
        ReleaseBuffer(view, &patch->detailBuffer);
        free(patch->roads);
        free(patch->nodes);
        free(patch->heads);
    }
    ReleaseBuffer(view, &view->previewBuffer);
    ReleaseBuffer(view, &view->overlayBuffer);
    for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
    {
        ReleaseBuffer(view, &view->lamps[lamp]);
        free(view->lampBatches[lamp].instances);
    }
    free(view->patches);
    free(view->roads);
    free(view->nodes);
    FoxMeshFree(&view->base);
    FoxMeshFree(&view->detail);
    free(view->heads.items);
    free(view);
}

static bool GrowArray(void **array, uint32_t *capacity, uint32_t needed, size_t size)
{
    if (needed <= *capacity)
    {
        return true;
    }
    uint32_t grown = *capacity < 16u ? 16u : *capacity;
    while (grown < needed)
    {
        grown *= 2u;
    }
    void *bigger = realloc(*array, (size_t)grown * size);
    if (bigger == NULL)
    {
        return false;
    }
    memset((uint8_t *)bigger + (size_t)*capacity * size, 0, (size_t)(grown - *capacity) * size);
    *array = bigger;
    *capacity = grown;
    return true;
}

/* ---- Patches ----------------------------------------------------------------- */

static int64_t PatchCoordinate(int64_t value)
{
    return FoxFloorDivide(value, INT64_C(1) << PATCH_SHIFT);
}

static uint32_t FindPatch(FoxRoadsView *view, FoxVec point, bool create)
{
    const int64_t x = PatchCoordinate(point.x);
    const int64_t y = PatchCoordinate(point.y);
    const int64_t z = PatchCoordinate(point.z);
    uint32_t vacant = NO_PATCH;
    for (uint32_t index = 0u; index < view->patchCapacity; ++index)
    {
        const Patch *patch = &view->patches[index];
        if (patch->used && patch->x == x && patch->y == y && patch->z == z)
        {
            return index;
        }
        if (!patch->used && vacant == NO_PATCH)
        {
            vacant = index;
        }
    }
    if (!create)
    {
        return NO_PATCH;
    }
    if (vacant == NO_PATCH)
    {
        vacant = view->patchCapacity;
        if (!GrowArray((void **)&view->patches, &view->patchCapacity, vacant + 1u,
                       sizeof(*view->patches)))
        {
            return NO_PATCH;
        }
    }
    Patch *patch = &view->patches[vacant];
    patch->used = true;
    patch->x = x;
    patch->y = y;
    patch->z = z;
    const int64_t size = INT64_C(1) << PATCH_SHIFT;
    patch->origin = (FoxVec){x * size, y * size, z * size};
    return vacant;
}

static bool AddMember(uint32_t **members, uint32_t *count, uint32_t *capacity, uint32_t member)
{
    if (!GrowArray((void **)members, capacity, *count + 1u, sizeof(**members)))
    {
        return false;
    }
    (*members)[(*count)++] = member;
    return true;
}

static void RemoveMember(uint32_t *members, uint32_t *count, uint32_t member)
{
    for (uint32_t index = 0u; index < *count; ++index)
    {
        if (members[index] == member)
        {
            members[index] = members[--*count];
            return;
        }
    }
}

/* Moves roads and nodes between patches as the network changes. */
static void Track(FoxRoadsView *view, const FoxNetwork *network)
{
    const uint32_t roadCapacity = FoxNetworkRoadCapacity(network);
    const uint32_t nodeCapacity = FoxNetworkNodeCapacity(network);
    if (!GrowArray((void **)&view->roads, &view->roadCapacity, roadCapacity,
                   sizeof(*view->roads)) ||
        !GrowArray((void **)&view->nodes, &view->nodeCapacity, nodeCapacity, sizeof(*view->nodes)))
    {
        return;
    }
    for (uint32_t index = 0u; index < view->roadCapacity; ++index)
    {
        Tracked *tracked = &view->roads[index];
        FoxRoadView road = {0};
        const bool alive = index < roadCapacity && FoxNetworkRoad(network, index, &road);
        if (alive && tracked->present && tracked->revision == road.revision)
        {
            continue;
        }
        if (tracked->present)
        {
            Patch *old = &view->patches[tracked->patch];
            RemoveMember(old->roads, &old->roadCount, index);
            old->dirty = true;
            tracked->present = false;
        }
        if (alive)
        {
            const uint32_t patch = FindPatch(view, road.centre[road.centreCount / 2u], true);
            if (patch != NO_PATCH &&
                AddMember(&view->patches[patch].roads, &view->patches[patch].roadCount,
                          &view->patches[patch].roadCapacity, index))
            {
                view->patches[patch].dirty = true;
                *tracked = (Tracked){.present = true, .revision = road.revision, .patch = patch};
            }
        }
    }
    for (uint32_t index = 0u; index < view->nodeCapacity; ++index)
    {
        Tracked *tracked = &view->nodes[index];
        FoxNodeView node = {0};
        const bool alive = index < nodeCapacity && FoxNetworkNode(network, index, &node);
        if (alive && tracked->present && tracked->revision == node.revision)
        {
            continue;
        }
        if (tracked->present)
        {
            Patch *old = &view->patches[tracked->patch];
            RemoveMember(old->nodes, &old->nodeCount, index);
            old->dirty = true;
            tracked->present = false;
        }
        if (alive)
        {
            const uint32_t patch = FindPatch(view, node.position, true);
            if (patch != NO_PATCH &&
                AddMember(&view->patches[patch].nodes, &view->patches[patch].nodeCount,
                          &view->patches[patch].nodeCapacity, index))
            {
                view->patches[patch].dirty = true;
                *tracked = (Tracked){.present = true, .revision = node.revision, .patch = patch};
            }
        }
    }
}

static bool Upload(FoxRoadsView *view, const FoxMesh *mesh, LaiueGraphicsHandle *outHandle)
{
    *outHandle = 0u;
    return mesh->count == 0u ||
           UploadVertices(view->device, mesh->vertices, mesh->count, outHandle);
}

static void RebuildPatch(FoxRoadsView *view, const FoxNetwork *network, Patch *patch)
{
    patch->dirty = false;
    FoxMeshReset(&view->base);
    FoxMeshReset(&view->detail);
    view->heads.count = 0u;
    view->heads.failed = false;
    for (uint32_t index = 0u; index < patch->roadCount; ++index)
    {
        FoxRoadMeshRoad(network, patch->roads[index], patch->origin, &view->base, &view->detail,
                        &view->heads);
    }
    for (uint32_t index = 0u; index < patch->nodeCount; ++index)
    {
        FoxRoadMeshNode(network, patch->nodes[index], patch->origin, &view->base, &view->detail);
    }
    FoxSignalHead *heads = NULL;
    if (view->heads.count != 0u)
    {
        heads = malloc((size_t)view->heads.count * sizeof(*heads));
    }
    LaiueGraphicsHandle base = 0u;
    LaiueGraphicsHandle detail = 0u;
    if (view->base.failed || view->detail.failed || view->heads.failed ||
        (view->heads.count != 0u && heads == NULL) || !Upload(view, &view->base, &base) ||
        !Upload(view, &view->detail, &detail))
    {
        ReleaseBuffer(view, &base);
        ReleaseBuffer(view, &detail);
        free(heads);
        FoxLog(FOX_LOG_WARNING, "a road patch could not be built");
        return;
    }
    if (heads != NULL)
    {
        memcpy(heads, view->heads.items, (size_t)view->heads.count * sizeof(*heads));
    }
    ReleaseBuffer(view, &patch->baseBuffer);
    ReleaseBuffer(view, &patch->detailBuffer);
    free(patch->heads);
    patch->baseBuffer = base;
    patch->baseCount = view->base.count;
    patch->detailBuffer = detail;
    patch->detailCount = view->detail.count;
    patch->heads = heads;
    patch->headCount = view->heads.count;
    if (patch->roadCount == 0u && patch->nodeCount == 0u)
    {
        free(patch->roads);
        free(patch->nodes);
        free(patch->heads);
        memset(patch, 0, sizeof(*patch));
    }
}

void FoxRoadsViewUpdate(FoxRoadsView *view, const FoxNetwork *network, double budgetSeconds)
{
    if (view == NULL || network == NULL)
    {
        return;
    }
    const double started = LaiueApplicationMonotonicSeconds();
    const uint64_t revision = FoxNetworkRevision(network);
    if (!view->seenAny || revision != view->seenRevision)
    {
        Track(view, network);
        view->seenRevision = revision;
        view->seenAny = true;
    }
    uint32_t built = 0u;
    for (uint32_t index = 0u; index < view->patchCapacity; ++index)
    {
        if (built != 0u && LaiueApplicationMonotonicSeconds() - started >= budgetSeconds)
        {
            break;
        }
        Patch *patch = &view->patches[index];
        if (patch->used && patch->dirty)
        {
            RebuildPatch(view, network, patch);
            ++built;
        }
    }
}

void FoxRoadsViewSetPreview(FoxRoadsView *view, const FoxVec *centre, uint32_t count,
                            FoxRoadType type, bool valid)
{
    if (view == NULL)
    {
        return;
    }
    ReleaseBuffer(view, &view->previewBuffer);
    view->previewCount = 0u;
    if (centre == NULL || count < 2u || FoxRoadTypeGet(type) == NULL)
    {
        return;
    }
    view->previewOrigin = centre[0];
    FoxMeshReset(&view->base);
    FoxRoadMeshPreview(centre, count, type, valid ? COLOR_PREVIEW : COLOR_PREVIEW_BAD,
                       view->previewOrigin, &view->base);
    if (!view->base.failed && view->base.count != 0u &&
        UploadVertices(view->device, view->base.vertices, view->base.count, &view->previewBuffer))
    {
        view->previewCount = view->base.count;
    }
}

void FoxRoadsViewSetOverlay(FoxRoadsView *view, const FoxOverlayLine *lines, uint32_t lineCount,
                            const FoxOverlayRing *rings, uint32_t ringCount)
{
    if (view == NULL)
    {
        return;
    }
    ReleaseBuffer(view, &view->overlayBuffer);
    view->overlayCount = 0u;
    lineCount = lines != NULL ? lineCount : 0u;
    ringCount = rings != NULL ? ringCount : 0u;
    /* Any point of the overlay serves as its origin. */
    bool placed = false;
    for (uint32_t index = 0u; index < lineCount && !placed; ++index)
    {
        if (lines[index].points != NULL && lines[index].count != 0u)
        {
            view->overlayOrigin = lines[index].points[0];
            placed = true;
        }
    }
    if (!placed && ringCount != 0u)
    {
        view->overlayOrigin = rings[0].centre;
        placed = true;
    }
    if (!placed)
    {
        return;
    }
    FoxMeshReset(&view->base);
    for (uint32_t index = 0u; index < lineCount; ++index)
    {
        FoxRoadMeshLine(lines[index].points, lines[index].count, OVERLAY_LINE_HALF,
                        lines[index].dashed, lines[index].color, view->overlayOrigin, &view->base);
    }
    for (uint32_t index = 0u; index < ringCount; ++index)
    {
        FoxRoadMeshRing(rings[index].centre, rings[index].radius, OVERLAY_RING_WIDTH,
                        rings[index].color, view->overlayOrigin, &view->base);
    }
    if (!view->base.failed && view->base.count != 0u &&
        UploadVertices(view->device, view->base.vertices, view->base.count, &view->overlayBuffer))
    {
        view->overlayCount = view->base.count;
    }
}

static void Submit(FoxRoadsView *view, const FoxCameraView *camera, LaiueGraphicsHandle buffer,
                   uint32_t count, FoxVec origin)
{
    LaiueGraphicsDrawItemV2 draw = {
        .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
        .vertexBuffer = buffer,
        .indexCount = count,
        .scale = 1.0f,
    };
    FoxCameraRelative(camera, origin, draw.originRelative);
    (void)view->device->submit(view->device, &draw, 1u);
}

static void AddLamp(FoxRoadsView *view, const FoxCameraView *camera, uint32_t lamp, FoxVec point)
{
    LaiueGraphicsInstanceV2 instance = {.scale = 1.0f, .rotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    FoxCameraRelative(camera, point, instance.originRelative);
    if (!view->instancing)
    {
        Submit(view, camera, view->lamps[lamp], LAMP_VERTICES, point);
        return;
    }
    LampBatch *batch = &view->lampBatches[lamp];
    if (batch->count == batch->capacity)
    {
        const uint32_t capacity = batch->capacity < 64u ? 64u : batch->capacity * 2u;
        LaiueGraphicsInstanceV2 *grown =
            realloc(batch->instances, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            return;
        }
        batch->instances = grown;
        batch->capacity = capacity;
    }
    batch->instances[batch->count++] = instance;
}

void FoxRoadsViewDraw(FoxRoadsView *view, const FoxCameraView *camera, const FoxNetwork *network)
{
    if (view == NULL || camera == NULL)
    {
        return;
    }
    for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
    {
        view->lampBatches[lamp].count = 0u;
    }
    const double half = (double)(INT64_C(1) << PATCH_SHIFT) / 2.0;
    for (uint32_t index = 0u; index < view->patchCapacity; ++index)
    {
        const Patch *patch = &view->patches[index];
        if (!patch->used)
        {
            continue;
        }
        if (patch->baseBuffer != 0u)
        {
            Submit(view, camera, patch->baseBuffer, patch->baseCount, patch->origin);
        }
        const FoxVec middle = {patch->origin.x + (int64_t)half, patch->origin.y + (int64_t)half,
                               patch->origin.z + (int64_t)half};
        float relative[3];
        FoxCameraRelative(camera, middle, relative);
        const double dx = (double)relative[0] - (double)camera->eye[0];
        const double dy = (double)relative[1] - (double)camera->eye[1];
        const double dz = (double)relative[2] - (double)camera->eye[2];
        if (sqrt(dx * dx + dy * dy + dz * dz) > DETAIL_DISTANCE)
        {
            continue;
        }
        if (patch->detailBuffer != 0u)
        {
            Submit(view, camera, patch->detailBuffer, patch->detailCount, patch->origin);
        }
        for (uint32_t head = 0u; network != NULL && head < patch->headCount; ++head)
        {
            const FoxSignalHead *signal = &patch->heads[head];
            const FoxLight light = FoxNetworkLight(network, signal->road, signal->end);
            if (light != FOX_LIGHT_NONE)
            {
                const uint32_t lamp = light == FOX_LIGHT_RED      ? 0u
                                      : light == FOX_LIGHT_YELLOW ? 1u
                                                                  : 2u;
                AddLamp(view, camera, lamp, signal->lamps[lamp]);
            }
            const FoxLight arrow = FoxNetworkArrow(network, signal->road, signal->end);
            if (arrow != FOX_LIGHT_NONE)
            {
                AddLamp(view, camera, arrow == FOX_LIGHT_YELLOW ? 1u : 2u, signal->arrow);
            }
        }
    }
    if (view->instancing)
    {
        for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
        {
            const LampBatch *batch = &view->lampBatches[lamp];
            if (batch->count == 0u)
            {
                continue;
            }
            const LaiueGraphicsDrawItemV2 draw = {
                .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
                .vertexBuffer = view->lamps[lamp],
                .indexCount = LAMP_VERTICES,
                .scale = 1.0f,
            };
            (void)view->device->submitInstances(view->device, &draw, batch->instances,
                                                batch->count);
        }
    }
    if (view->previewBuffer != 0u)
    {
        Submit(view, camera, view->previewBuffer, view->previewCount, view->previewOrigin);
    }
    if (view->overlayBuffer != 0u)
    {
        Submit(view, camera, view->overlayBuffer, view->overlayCount, view->overlayOrigin);
    }
}
