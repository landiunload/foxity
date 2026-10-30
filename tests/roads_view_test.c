#include "client/roads_view.h"
#include "expect.h"
#include "sim/network.h"
#include "sim/planet.h"

#include <stdint.h>

/* A device that keeps no geometry: it hands out handles and refuses uploads
 * while told to, as a real one does when a frame has no room left. */
typedef struct Device
{
    LaiueGraphicsDeviceV2 table;
    LaiueGraphicsHandle next;
    bool refusing;
    uint32_t asked;
    uint32_t accepted;
} Device;

static uint32_t CreateBuffer(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsBufferDescV1 *desc,
                             LaiueGraphicsHandle *outBuffer)
{
    (void)desc;
    Device *device = table->context;
    *outBuffer = ++device->next;
    return 1u;
}

static uint32_t UploadBuffer(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsBufferUploadV1 *upload)
{
    (void)upload;
    Device *device = table->context;
    ++device->asked;
    if (device->refusing)
    {
        return 0u;
    }
    ++device->accepted;
    return 1u;
}

static void DestroyHandle(LaiueGraphicsDeviceV2 *table, LaiueGraphicsHandle handle)
{
    (void)table;
    (void)handle;
}

static uint32_t Submit(LaiueGraphicsDeviceV2 *table, const LaiueGraphicsDrawItemV2 *items,
                       uint32_t itemCount)
{
    (void)table;
    (void)items;
    (void)itemCount;
    return 1u;
}

static void Update(FoxRoadsView *view, const FoxNetwork *network, uint32_t times)
{
    for (uint32_t update = 0u; update < times; ++update)
    {
        FoxRoadsViewUpdate(view, network, 1.0);
    }
}

int main(void)
{
    FoxPlanet *planet = FoxPlanetCreate(FOX_PLANET_DEFAULT_RADIUS);
    const FoxNetworkDesc networkDesc = {.driveSide = FOX_DRIVE_RIGHT};
    FoxNetwork *network = FoxNetworkCreate(&networkDesc);
    EXPECT(planet != NULL && network != NULL, "the planet and the network");
    const FoxRoadPlan plan = {
        .start = {FOX_METRES(-50), FOX_METRES(10), FOX_PLANET_DEFAULT_RADIUS},
        .end = {FOX_METRES(50), FOX_METRES(10), FOX_PLANET_DEFAULT_RADIUS},
        .layout = FoxRoadLayoutPreset(FOX_ROAD_STREET),
        .speedLimit = FoxRoadPresetSpeed(FOX_ROAD_STREET),
    };
    EXPECT(FoxNetworkBuildRoad(network, planet, &plan) == FOX_ROAD_OK, "a street");

    Device device = {
        .table =
            {
                .structSize = sizeof(LaiueGraphicsDeviceV2),
                .context = &device,
                .createBuffer = CreateBuffer,
                .uploadBuffer = UploadBuffer,
                .destroyHandle = DestroyHandle,
                .submit = Submit,
            },
    };
    FoxRoadsView *view = FoxRoadsViewCreate(&device.table);
    EXPECT(view != NULL, "the view");
    const uint32_t lamps = device.accepted;

    /* The device has no room: every patch of the street is refused. */
    device.refusing = true;
    Update(view, network, 1u);
    const uint32_t patches = device.asked - lamps;
    EXPECT(patches != 0u && device.accepted == lamps, "the street's patches are refused");

    /* Refused on and on, they are tried less and less often: two updates
     * later, then four, eight and so on up to 64, seven times in 200. */
    const uint32_t before = device.asked;
    Update(view, network, 200u);
    const uint32_t tries = (device.asked - before) / patches;
    EXPECT(tries >= 5u && tries <= 8u, "a refused patch backs off");

    /* With room again the patches are built at their next try, although the
     * street itself never changed. */
    device.refusing = false;
    Update(view, network, 65u);
    EXPECT(device.accepted > lamps, "the street is built once the device has room");
    const uint32_t built = device.accepted;
    Update(view, network, 200u);
    EXPECT(device.accepted == built, "a built patch is not built again");

    FoxRoadsViewDestroy(view);
    FoxNetworkDestroy(network);
    FoxPlanetDestroy(planet);
    return 0;
}
