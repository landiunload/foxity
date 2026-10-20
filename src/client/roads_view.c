#include "client/roads_view.h"

#include "app/log.h"
#include "mod/application.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PATCH_SHIFT 17u
#define NO_PATCH UINT32_MAX
/* Heights above the ground: asphalt, intersections and markings stack up so
 * they never fight for the same depth. */
#define ASPHALT_LIFT 0.12
#define JUNCTION_LIFT 0.13
#define MARKING_LIFT 0.15
#define SHOULDER 0.3
#define MARKING_WIDTH 0.18
#define DASH 3.0
#define DASH_GAP 6.0
#define JUNCTION_SEGMENTS 20u

#define COLOR_ASPHALT UINT32_C(0xFF3C3A38)
#define COLOR_JUNCTION UINT32_C(0xFF44423F)
#define COLOR_LINE UINT32_C(0xFFE6E6E6)
#define COLOR_CENTRE UINT32_C(0xFF2AB8E8)
#define COLOR_PREVIEW UINT32_C(0xFF55C46A)
#define COLOR_PREVIEW_BAD UINT32_C(0xFF3A3AD8)

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
    LaiueGraphicsHandle vertexBuffer;
    uint32_t vertexCount;
} Patch;

typedef struct Tracked
{
    bool present;
    uint64_t revision;
    uint32_t patch;
} Tracked;

typedef struct Mesh
{
    LaiueGraphicsVertexV2 *vertices;
    uint32_t count;
    uint32_t capacity;
    bool failed;
} Mesh;

struct FoxRoadsView
{
    LaiueGraphicsDeviceV2 *device;
    uint64_t seenRevision;
    bool seenAny;
    Patch *patches;
    uint32_t patchCapacity;
    Tracked *roads;
    uint32_t roadCapacity;
    Tracked *nodes;
    uint32_t nodeCapacity;
    Mesh mesh;
    LaiueGraphicsHandle previewBuffer;
    uint32_t previewCount;
    FoxVec previewOrigin;
};

FoxRoadsView *FoxRoadsViewCreate(LaiueGraphicsDeviceV2 *device)
{
    if (device == NULL || device->createBuffer == NULL || device->uploadBuffer == NULL ||
        device->destroyHandle == NULL || device->submit == NULL)
    {
        return NULL;
    }
    FoxRoadsView *view = calloc(1u, sizeof(*view));
    if (view != NULL)
    {
        view->device = device;
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
        ReleaseBuffer(view, &view->patches[index].vertexBuffer);
        free(view->patches[index].roads);
        free(view->patches[index].nodes);
    }
    ReleaseBuffer(view, &view->previewBuffer);
    free(view->patches);
    free(view->roads);
    free(view->nodes);
    free(view->mesh.vertices);
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
    return value >= 0 ? (int64_t)((uint64_t)value >> PATCH_SHIFT)
                      : -1 - (int64_t)((uint64_t)(-(value + 1)) >> PATCH_SHIFT);
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
    if (!GrowArray((void **)&view->roads, &view->roadCapacity, roadCapacity, sizeof(*view->roads)) ||
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

/* ---- Geometry ----------------------------------------------------------------- */

static void Push(Mesh *mesh, const double position[3], uint32_t color)
{
    if (!GrowArray((void **)&mesh->vertices, &mesh->capacity, mesh->count + 1u,
                   sizeof(*mesh->vertices)))
    {
        mesh->failed = true;
        return;
    }
    mesh->vertices[mesh->count++] = (LaiueGraphicsVertexV2){
        .position = {(float)position[0], (float)position[1], (float)position[2]},
        .colorRGBA = color,
    };
}

static void Quad(Mesh *mesh, const double a[3], const double b[3], const double c[3],
                 const double d[3], uint32_t color)
{
    Push(mesh, a, color);
    Push(mesh, b, color);
    Push(mesh, c, color);
    Push(mesh, a, color);
    Push(mesh, c, color);
    Push(mesh, d, color);
}

typedef struct Frame
{
    /* Metres relative to the mesh origin. */
    double point[3];
    double up[3];
    double right[3];
} Frame;

static void Unit(double vector[3])
{
    const double length = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (length > 0.0)
    {
        vector[0] /= length;
        vector[1] /= length;
        vector[2] /= length;
    }
}

/* The centre point, up and right of a road at each of its points. */
static Frame FrameAt(const FoxVec *centre, uint32_t count, uint32_t index, FoxVec origin)
{
    Frame frame;
    const FoxVec point = centre[index];
    frame.point[0] = (double)(point.x - origin.x) / FOX_UNITS_PER_METRE;
    frame.point[1] = (double)(point.y - origin.y) / FOX_UNITS_PER_METRE;
    frame.point[2] = (double)(point.z - origin.z) / FOX_UNITS_PER_METRE;
    frame.up[0] = (double)point.x;
    frame.up[1] = (double)point.y;
    frame.up[2] = (double)point.z;
    Unit(frame.up);
    const FoxVec before = centre[index == 0u ? 0u : index - 1u];
    const FoxVec after = centre[index + 1u < count ? index + 1u : index];
    double tangent[3] = {(double)(after.x - before.x), (double)(after.y - before.y),
                         (double)(after.z - before.z)};
    Unit(tangent);
    frame.right[0] = tangent[1] * frame.up[2] - tangent[2] * frame.up[1];
    frame.right[1] = tangent[2] * frame.up[0] - tangent[0] * frame.up[2];
    frame.right[2] = tangent[0] * frame.up[1] - tangent[1] * frame.up[0];
    Unit(frame.right);
    return frame;
}

static void Offset(const Frame *frame, double right, double lift, double out[3])
{
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        out[axis] = frame->point[axis] + frame->right[axis] * right + frame->up[axis] * lift;
    }
}

/* A strip between two offsets from the centre line along [from, to] metres
 * of it, dashed when `dash` is non-zero. */
static void Strip(Mesh *mesh, const FoxVec *centre, uint32_t count, FoxVec origin, double left,
                  double right, double lift, double from, double to, double dash, double gap,
                  uint32_t color)
{
    double travelled = 0.0;
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        const Frame a = FrameAt(centre, count, index, origin);
        const Frame b = FrameAt(centre, count, index + 1u, origin);
        const double dx = b.point[0] - a.point[0];
        const double dy = b.point[1] - a.point[1];
        const double dz = b.point[2] - a.point[2];
        const double length = sqrt(dx * dx + dy * dy + dz * dz);
        const double start = travelled;
        travelled += length;
        if (length <= 0.0 || travelled <= from || start >= to)
        {
            continue;
        }
        /* The pieces of this segment that are painted. */
        double piece = fmax(from, start);
        while (piece < fmin(to, travelled))
        {
            double end = fmin(to, travelled);
            if (dash > 0.0)
            {
                const double phase = fmod(piece - from, dash + gap);
                if (phase >= dash)
                {
                    piece += dash + gap - phase;
                    continue;
                }
                end = fmin(end, piece + dash - phase);
            }
            const double t0 = (piece - start) / length;
            const double t1 = (end - start) / length;
            Frame p0;
            Frame p1;
            for (uint32_t axis = 0u; axis < 3u; ++axis)
            {
                p0.point[axis] = a.point[axis] + (b.point[axis] - a.point[axis]) * t0;
                p1.point[axis] = a.point[axis] + (b.point[axis] - a.point[axis]) * t1;
                p0.up[axis] = a.up[axis] + (b.up[axis] - a.up[axis]) * t0;
                p1.up[axis] = a.up[axis] + (b.up[axis] - a.up[axis]) * t1;
                p0.right[axis] = a.right[axis] + (b.right[axis] - a.right[axis]) * t0;
                p1.right[axis] = a.right[axis] + (b.right[axis] - a.right[axis]) * t1;
            }
            double q[4][3];
            Offset(&p0, left, lift, q[0]);
            Offset(&p0, right, lift, q[1]);
            Offset(&p1, right, lift, q[2]);
            Offset(&p1, left, lift, q[3]);
            Quad(mesh, q[0], q[1], q[2], q[3], color);
            piece = end;
        }
    }
}

static double HalfWidth(FoxRoadType type)
{
    const FoxRoadTypeInfo *info = FoxRoadTypeGet(type);
    return (double)(info->forwardLanes + info->backwardLanes) * (double)FOX_LANE_WIDTH / 2.0 /
           FOX_UNITS_PER_METRE;
}

static void AddRoad(Mesh *mesh, const FoxRoadView *road, FoxVec origin)
{
    const double half = HalfWidth(road->type) + SHOULDER;
    const double length = (double)road->length / FOX_UNITS_PER_METRE;
    Strip(mesh, road->centre, road->centreCount, origin, -half, half, ASPHALT_LIFT, 0.0, length,
          0.0, 0.0, COLOR_ASPHALT);
    const FoxRoadTypeInfo *info = FoxRoadTypeGet(road->type);
    const double from = (double)road->trim[0] / FOX_UNITS_PER_METRE;
    const double to = length - (double)road->trim[1] / FOX_UNITS_PER_METRE;
    const double lane = (double)FOX_LANE_WIDTH / FOX_UNITS_PER_METRE;
    const double line = MARKING_WIDTH / 2.0;
    const double side = road->driveSide == FOX_DRIVE_RIGHT ? 1.0 : -1.0;
    if (info->backwardLanes != 0u)
    {
        Strip(mesh, road->centre, road->centreCount, origin, -line, line, MARKING_LIFT, from, to,
              0.0, 0.0, COLOR_CENTRE);
        for (uint32_t index = 1u; index < info->forwardLanes; ++index)
        {
            const double at = side * lane * (double)index;
            Strip(mesh, road->centre, road->centreCount, origin, at - line, at + line,
                  MARKING_LIFT, from, to, DASH, DASH_GAP, COLOR_LINE);
        }
        for (uint32_t index = 1u; index < info->backwardLanes; ++index)
        {
            const double at = -side * lane * (double)index;
            Strip(mesh, road->centre, road->centreCount, origin, at - line, at + line,
                  MARKING_LIFT, from, to, DASH, DASH_GAP, COLOR_LINE);
        }
        return;
    }
    for (uint32_t index = 1u; index < info->forwardLanes; ++index)
    {
        const double at = lane * ((double)info->forwardLanes / 2.0 - (double)index);
        Strip(mesh, road->centre, road->centreCount, origin, at - line, at + line, MARKING_LIFT,
              from, to, DASH, DASH_GAP, COLOR_LINE);
    }
}

static void AddJunction(Mesh *mesh, const FoxNodeView *node, FoxVec origin)
{
    if (node->roadCount == 2u)
    {
        return;
    }
    const FoxVec centre[1] = {node->position};
    Frame frame = FrameAt(centre, 1u, 0u, origin);
    /* Any tangent pair at the node. */
    const double axis[3] = {fabs(frame.up[2]) < 0.9 ? 0.0 : 1.0, 0.0,
                            fabs(frame.up[2]) < 0.9 ? 1.0 : 0.0};
    double first[3] = {axis[1] * frame.up[2] - axis[2] * frame.up[1],
                       axis[2] * frame.up[0] - axis[0] * frame.up[2],
                       axis[0] * frame.up[1] - axis[1] * frame.up[0]};
    Unit(first);
    const double second[3] = {frame.up[1] * first[2] - frame.up[2] * first[1],
                              frame.up[2] * first[0] - frame.up[0] * first[2],
                              frame.up[0] * first[1] - frame.up[1] * first[0]};
    const double radius = (double)node->radius / FOX_UNITS_PER_METRE + 0.5;
    double middle[3];
    for (uint32_t index = 0u; index < 3u; ++index)
    {
        middle[index] = frame.point[index] + frame.up[index] * JUNCTION_LIFT;
    }
    for (uint32_t segment = 0u; segment < JUNCTION_SEGMENTS; ++segment)
    {
        const double a = 6.283185307179586 * segment / JUNCTION_SEGMENTS;
        const double b = 6.283185307179586 * (segment + 1u) / JUNCTION_SEGMENTS;
        double p[3];
        double q[3];
        for (uint32_t index = 0u; index < 3u; ++index)
        {
            p[index] = middle[index] + (first[index] * cos(a) + second[index] * sin(a)) * radius;
            q[index] = middle[index] + (first[index] * cos(b) + second[index] * sin(b)) * radius;
        }
        Push(mesh, middle, COLOR_JUNCTION);
        Push(mesh, p, COLOR_JUNCTION);
        Push(mesh, q, COLOR_JUNCTION);
    }
}

static bool UploadMesh(FoxRoadsView *view, const Mesh *mesh, LaiueGraphicsHandle *outHandle)
{
    const uint64_t bytes = (uint64_t)mesh->count * sizeof(*mesh->vertices);
    const LaiueGraphicsBufferDescV1 description = {
        .structSize = sizeof(description),
        .usageFlags = LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX,
        .sizeBytes = bytes,
    };
    LaiueGraphicsHandle handle = 0u;
    if (view->device->createBuffer(view->device, &description, &handle) == 0u)
    {
        return false;
    }
    const LaiueGraphicsBufferUploadV1 upload = {
        .structSize = sizeof(upload),
        .buffer = handle,
        .data = mesh->vertices,
        .sizeBytes = bytes,
    };
    if (view->device->uploadBuffer(view->device, &upload) == 0u)
    {
        view->device->destroyHandle(view->device, handle);
        return false;
    }
    *outHandle = handle;
    return true;
}

static void RebuildPatch(FoxRoadsView *view, const FoxNetwork *network, Patch *patch)
{
    patch->dirty = false;
    view->mesh.count = 0u;
    view->mesh.failed = false;
    for (uint32_t index = 0u; index < patch->roadCount; ++index)
    {
        FoxRoadView road;
        if (FoxNetworkRoad(network, patch->roads[index], &road))
        {
            AddRoad(&view->mesh, &road, patch->origin);
        }
    }
    for (uint32_t index = 0u; index < patch->nodeCount; ++index)
    {
        FoxNodeView node;
        if (FoxNetworkNode(network, patch->nodes[index], &node))
        {
            AddJunction(&view->mesh, &node, patch->origin);
        }
    }
    LaiueGraphicsHandle fresh = 0u;
    if (view->mesh.failed || (view->mesh.count != 0u && !UploadMesh(view, &view->mesh, &fresh)))
    {
        FoxLog(FOX_LOG_WARNING, "a road patch could not be built");
        return;
    }
    ReleaseBuffer(view, &patch->vertexBuffer);
    patch->vertexBuffer = fresh;
    patch->vertexCount = view->mesh.count;
    if (patch->roadCount == 0u && patch->nodeCount == 0u)
    {
        free(patch->roads);
        free(patch->nodes);
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
    view->mesh.count = 0u;
    view->mesh.failed = false;
    const double half = HalfWidth(type) + SHOULDER;
    Strip(&view->mesh, centre, count, view->previewOrigin, -half, half, MARKING_LIFT + 0.05, 0.0,
          1e9, 0.0, 0.0, valid ? COLOR_PREVIEW : COLOR_PREVIEW_BAD);
    if (!view->mesh.failed && view->mesh.count != 0u &&
        UploadMesh(view, &view->mesh, &view->previewBuffer))
    {
        view->previewCount = view->mesh.count;
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

void FoxRoadsViewDraw(FoxRoadsView *view, const FoxCameraView *camera)
{
    if (view == NULL || camera == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < view->patchCapacity; ++index)
    {
        const Patch *patch = &view->patches[index];
        if (patch->used && patch->vertexBuffer != 0u)
        {
            Submit(view, camera, patch->vertexBuffer, patch->vertexCount, patch->origin);
        }
    }
    if (view->previewBuffer != 0u)
    {
        Submit(view, camera, view->previewBuffer, view->previewCount, view->previewOrigin);
    }
}
