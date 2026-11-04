#include "client/scene.h"
#include "expect.h"

#include <stdint.h>

/* A device that counts the handles it has handed out and not got back. */
typedef struct Device
{
    LaiueGraphicsDeviceV2 table;
    LaiueGraphicsHandle next;
    uint32_t live;
} Device;

static uint32_t CreateBuffer(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsBufferDescV1 *desc,
                             LaiueGraphicsHandle *outBuffer)
{
    (void)desc;
    Device *device = table->context;
    *outBuffer = ++device->next;
    ++device->live;
    return 1u;
}

static uint32_t UploadBuffer(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsBufferUploadV1 *upload)
{
    (void)table;
    (void)upload;
    return 1u;
}

static void DestroyHandle(LaiueGraphicsDeviceV2 *table, LaiueGraphicsHandle handle)
{
    Device *device = table->context;
    device->live -= handle != 0u ? 1u : 0u;
}

static uint32_t Submit(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsDrawItemV2 *items,
                       uint32_t itemCount)
{
    (void)table;
    (void)items;
    (void)itemCount;
    return 1u;
}

static void InitializeDevice(Device *device)
{
    *device = (Device){
        .table =
            {
                .structSize = sizeof(LaiueGraphicsDeviceV2),
                .context = device,
                .createBuffer = CreateBuffer,
                .uploadBuffer = UploadBuffer,
                .destroyHandle = DestroyHandle,
                .submit = Submit,
            },
    };
}

int main(void)
{
    Device lost;
    Device replacement;
    InitializeDevice(&lost);
    InitializeDevice(&replacement);
    const FoxSceneDesc desc = {.seed = 1u, .driveSide = FOX_DRIVE_RIGHT, .device = &lost.table};
    FoxScene *scene = FoxSceneCreate(&desc);
    EXPECT(scene != NULL, "the scene");
    const uint32_t held = lost.live;
    EXPECT(held != 0u, "the views hold handles on the device");

    FoxSceneDetachDevice(scene);
    EXPECT(lost.live == 0u, "a lost device gets everything back");

    /* The city runs on with nothing to draw on. */
    const FoxControls controls = {.tool = -1, .density = 1.0f};
    float viewProjection[16];
    (void)FoxSceneUpdate(scene, &controls, 1.0, 1.0f, viewProjection);
    FoxSceneDraw(scene);

    FoxSceneAttachDevice(scene, &replacement.table);
    EXPECT(replacement.live == held, "the views return on the new device");
    EXPECT(lost.live == 0u, "the lost device is not touched again");
    (void)FoxSceneUpdate(scene, &controls, 1.0, 1.0f, viewProjection);
    FoxSceneDraw(scene);

    FoxSceneDestroy(scene);
    EXPECT(replacement.live == 0u, "the new device gets everything back");
    return 0;
}
