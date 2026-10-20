#include "client/cars_view.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define COLORS 8u
#define TRUCK_COLORS 4u
#define MODELS (COLORS + TRUCK_COLORS)
/* Boxes of six faces of two triangles: a car has a body, a cabin and four
 * wheels, a truck a cab, a box and six wheels. */
#define BOX_VERTICES 36u
#define MAX_VERTICES (9u * BOX_VERTICES)
#define LIFT 0.15f
/* Cars further than this from the eye are too small to matter. */
#define DRAW_DISTANCE 2500.0f

static const uint32_t paints[COLORS] = {
    UINT32_C(0xFF2B3BD1), UINT32_C(0xFFD8D8D8), UINT32_C(0xFF2A2A2A), UINT32_C(0xFFC2702B),
    UINT32_C(0xFF3FA7E0), UINT32_C(0xFF4E9A3A), UINT32_C(0xFF7E7E86), UINT32_C(0xFF2E58A8),
};

typedef struct Batch
{
    LaiueGraphicsInstanceV2 *instances;
    uint32_t count;
    uint32_t capacity;
} Batch;

struct FoxCarsView
{
    LaiueGraphicsDeviceV2 *device;
    bool instancing;
    /* Cars in every paint, then trucks. */
    LaiueGraphicsHandle meshes[MODELS];
    uint32_t vertexCounts[MODELS];
    Batch batches[MODELS];
};

static uint32_t Darken(uint32_t color, float factor)
{
    uint32_t result = color & UINT32_C(0xFF000000);
    for (uint32_t channel = 0u; channel < 3u; ++channel)
    {
        const float value = (float)((color >> (8u * channel)) & 255u) * factor;
        result |= (uint32_t)lrintf(value > 255.0f ? 255.0f : value) << (8u * channel);
    }
    return result;
}

/* A box from `low` to `high` in car space: x forward, y left, z up. The top
 * is lit, the sides a little less, the bottom least. */
static uint32_t Box(LaiueGraphicsVertexV2 *out, const float low[3], const float high[3],
                    uint32_t color)
{
    static const uint8_t faces[6][4] = {
        {0u, 1u, 3u, 2u}, {4u, 6u, 7u, 5u}, {0u, 4u, 5u, 1u},
        {2u, 3u, 7u, 6u}, {0u, 2u, 6u, 4u}, {1u, 5u, 7u, 3u},
    };
    static const float shades[6] = {0.80f, 0.80f, 0.70f, 1.00f, 0.88f, 0.75f};
    float corners[8][3];
    for (uint32_t corner = 0u; corner < 8u; ++corner)
    {
        corners[corner][0] = (corner & 1u) != 0u ? high[0] : low[0];
        corners[corner][1] = (corner & 4u) != 0u ? high[1] : low[1];
        corners[corner][2] = (corner & 2u) != 0u ? high[2] : low[2];
    }
    uint32_t written = 0u;
    for (uint32_t face = 0u; face < 6u; ++face)
    {
        const uint32_t shade = Darken(color, shades[face]);
        static const uint8_t order[6] = {0u, 1u, 2u, 0u, 2u, 3u};
        for (uint32_t index = 0u; index < 6u; ++index)
        {
            const float *corner = corners[faces[face][order[index]]];
            out[written++] = (LaiueGraphicsVertexV2){
                .position = {corner[0], corner[1], corner[2]},
                .colorRGBA = shade,
            };
        }
    }
    return written;
}

static uint32_t Wheel(LaiueGraphicsVertexV2 *out, float x, float y, float radius)
{
    const float low[3] = {x - radius, y - 0.14f, 0.0f};
    const float high[3] = {x + radius, y + 0.14f, 2.0f * radius};
    return Box(out, low, high, UINT32_C(0xFF1A1A1A));
}

/* About 4.5 m long around its middle. */
static uint32_t CarModel(LaiueGraphicsVertexV2 *out, uint32_t paint)
{
    const float bodyLow[3] = {-2.25f, -0.9f, 0.3f};
    const float bodyHigh[3] = {2.25f, 0.9f, 1.0f};
    const float cabinLow[3] = {-1.25f, -0.8f, 1.0f};
    const float cabinHigh[3] = {0.95f, 0.8f, 1.5f};
    uint32_t count = Box(out, bodyLow, bodyHigh, paint);
    count += Box(out + count, cabinLow, cabinHigh, Darken(paint, 0.6f));
    for (uint32_t wheel = 0u; wheel < 4u; ++wheel)
    {
        count += Wheel(out + count, (wheel & 1u) != 0u ? 1.45f : -1.45f,
                       (wheel & 2u) != 0u ? 0.82f : -0.82f, 0.33f);
    }
    return count;
}

/* About 12 m long: a cab in front of a box. */
static uint32_t TruckModel(LaiueGraphicsVertexV2 *out, uint32_t paint)
{
    const float cabLow[3] = {3.7f, -1.2f, 0.5f};
    const float cabHigh[3] = {6.0f, 1.2f, 3.0f};
    const float boxLow[3] = {-6.0f, -1.25f, 0.9f};
    const float boxHigh[3] = {3.5f, 1.25f, 3.9f};
    uint32_t count = Box(out, cabLow, cabHigh, paint);
    count += Box(out + count, boxLow, boxHigh, UINT32_C(0xFFDCDCDC));
    static const float axles[3] = {4.8f, -3.2f, -4.6f};
    for (uint32_t wheel = 0u; wheel < 6u; ++wheel)
    {
        count += Wheel(out + count, axles[wheel / 2u], (wheel & 1u) != 0u ? 1.05f : -1.05f, 0.5f);
    }
    return count;
}

FoxCarsView *FoxCarsViewCreate(LaiueGraphicsDeviceV2 *device)
{
    if (device == NULL || device->createBuffer == NULL || device->uploadBuffer == NULL ||
        device->destroyHandle == NULL || device->submit == NULL)
    {
        return NULL;
    }
    FoxCarsView *view = calloc(1u, sizeof(*view));
    if (view == NULL)
    {
        return NULL;
    }
    view->device = device;
    view->instancing =
        device->structSize >= LAIUE_GRAPHICS_DEVICE_V2_INSTANCES_SIZE &&
        device->getCapabilities != NULL && device->submitInstances != NULL &&
        (device->getCapabilities(device) & LAIUE_GRAPHICS_CAP_GENERIC_INSTANCES) != 0u;
    for (uint32_t model = 0u; model < MODELS; ++model)
    {
        LaiueGraphicsVertexV2 vertices[MAX_VERTICES];
        const uint32_t count = model < COLORS ? CarModel(vertices, paints[model])
                                              : TruckModel(vertices, paints[model - COLORS]);
        view->vertexCounts[model] = count;
        const uint64_t bytes = (uint64_t)count * sizeof(vertices[0]);
        const LaiueGraphicsBufferDescV1 description = {
            .structSize = sizeof(description),
            .usageFlags = LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX,
            .sizeBytes = bytes,
        };
        if (device->createBuffer(device, &description, &view->meshes[model]) == 0u)
        {
            view->meshes[model] = 0u;
            FoxCarsViewDestroy(view);
            return NULL;
        }
        const LaiueGraphicsBufferUploadV1 upload = {
            .structSize = sizeof(upload),
            .buffer = view->meshes[model],
            .data = vertices,
            .sizeBytes = bytes,
        };
        if (device->uploadBuffer(device, &upload) == 0u)
        {
            FoxCarsViewDestroy(view);
            return NULL;
        }
    }
    return view;
}

void FoxCarsViewDestroy(FoxCarsView *view)
{
    if (view == NULL)
    {
        return;
    }
    for (uint32_t model = 0u; model < MODELS; ++model)
    {
        if (view->meshes[model] != 0u)
        {
            view->device->destroyHandle(view->device, view->meshes[model]);
        }
        free(view->batches[model].instances);
    }
    free(view);
}

/* The rotation taking car space (x forward, y left, z up) to the world. */
static void Orientation(const double forward[3], const double up[3], float out[4])
{
    double left[3] = {up[1] * forward[2] - up[2] * forward[1], up[2] * forward[0] - up[0] * forward[2],
                      up[0] * forward[1] - up[1] * forward[0]};
    const double leftLength = sqrt(left[0] * left[0] + left[1] * left[1] + left[2] * left[2]);
    if (leftLength < 1e-9)
    {
        out[0] = out[1] = out[2] = 0.0f;
        out[3] = 1.0f;
        return;
    }
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        left[axis] /= leftLength;
    }
    /* Re-square forward against up and left. */
    const double ahead[3] = {left[1] * up[2] - left[2] * up[1], left[2] * up[0] - left[0] * up[2],
                             left[0] * up[1] - left[1] * up[0]};
    /* Columns of the rotation matrix are ahead, left and up. */
    const double m00 = ahead[0], m01 = left[0], m02 = up[0];
    const double m10 = ahead[1], m11 = left[1], m12 = up[1];
    const double m20 = ahead[2], m21 = left[2], m22 = up[2];
    const double trace = m00 + m11 + m22;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    if (trace > 0.0)
    {
        const double s = sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        const double s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    }
    else
    {
        const double s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }
    const double length = sqrt(x * x + y * y + z * z + w * w);
    out[0] = (float)(x / length);
    out[1] = (float)(y / length);
    out[2] = (float)(z / length);
    out[3] = (float)(w / length);
}

static bool Append(Batch *batch, const LaiueGraphicsInstanceV2 *instance)
{
    if (batch->count == batch->capacity)
    {
        const uint32_t capacity = batch->capacity < 256u ? 256u : batch->capacity * 2u;
        LaiueGraphicsInstanceV2 *grown =
            realloc(batch->instances, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            return false;
        }
        batch->instances = grown;
        batch->capacity = capacity;
    }
    batch->instances[batch->count++] = *instance;
    return true;
}

uint32_t FoxCarsViewDraw(FoxCarsView *view, const FoxTraffic *traffic, const FoxNetwork *network,
                         const FoxCameraView *camera, float blend)
{
    if (view == NULL || traffic == NULL || network == NULL || camera == NULL)
    {
        return 0u;
    }
    for (uint32_t model = 0u; model < MODELS; ++model)
    {
        view->batches[model].count = 0u;
    }
    const int64_t mix = (int64_t)lrintf((blend < 0.0f ? 0.0f : (blend > 1.0f ? 1.0f : blend)) *
                                        (float)FOX_UNIT);
    uint32_t drawn = 0u;
    const uint32_t capacity = FoxTrafficCapacity(traffic);
    for (uint32_t slot = 0u; slot < capacity; ++slot)
    {
        FoxVehicleView car;
        FoxVec position;
        FoxVec forward;
        if (!FoxTrafficVehicle(traffic, slot, &car) ||
            !FoxTrafficVehiclePose(traffic, network, slot, mix, &position, &forward))
        {
            continue;
        }
        LaiueGraphicsInstanceV2 instance = {.scale = 1.0f};
        FoxCameraRelative(camera, position, instance.originRelative);
        const float dx = instance.originRelative[0] - camera->eye[0];
        const float dy = instance.originRelative[1] - camera->eye[1];
        const float dz = instance.originRelative[2] - camera->eye[2];
        if (dx * dx + dy * dy + dz * dz > DRAW_DISTANCE * DRAW_DISTANCE)
        {
            continue;
        }
        double up[3] = {(double)position.x, (double)position.y, (double)position.z};
        const double length = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
        if (length > 0.0)
        {
            up[0] /= length;
            up[1] /= length;
            up[2] /= length;
        }
        for (uint32_t axis = 0u; axis < 3u; ++axis)
        {
            instance.originRelative[axis] += (float)up[axis] * LIFT;
        }
        const double ahead[3] = {(double)forward.x, (double)forward.y, (double)forward.z};
        Orientation(ahead, up, instance.rotation);
        const uint32_t model = car.kind == FOX_VEHICLE_TRUCK ? COLORS + car.color % TRUCK_COLORS
                                                             : car.color % COLORS;
        Batch *batch = &view->batches[model];
        if (view->instancing)
        {
            drawn += Append(batch, &instance) ? 1u : 0u;
            continue;
        }
        LaiueGraphicsDrawItemV2 draw = {
            .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
            .vertexBuffer = view->meshes[model],
            .indexCount = view->vertexCounts[model],
            .originRelative = {instance.originRelative[0], instance.originRelative[1],
                               instance.originRelative[2]},
            .scale = 1.0f,
        };
        drawn += view->device->submit(view->device, &draw, 1u) != 0u ? 1u : 0u;
    }
    if (view->instancing)
    {
        for (uint32_t model = 0u; model < MODELS; ++model)
        {
            const Batch *batch = &view->batches[model];
            if (batch->count == 0u)
            {
                continue;
            }
            const LaiueGraphicsDrawItemV2 draw = {
                .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
                .vertexBuffer = view->meshes[model],
                .indexCount = view->vertexCounts[model],
                .scale = 1.0f,
            };
            (void)view->device->submitInstances(view->device, &draw, batch->instances,
                                                batch->count);
        }
    }
    return drawn;
}
