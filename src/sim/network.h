#ifndef FOX_SIM_NETWORK_H
#define FOX_SIM_NETWORK_H

#include "sim/planet.h"
#include "sim/router.h"
#include "sim/space.h"

#include <stdbool.h>
#include <stdint.h>

/* The road network.
 *
 * Roads are curves draped over the ground between nodes. A node where three
 * or more roads meet is an intersection; one with a single road is a dead end
 * where cars turn around. Each road carries lanes in one or both directions,
 * and which side of the road a direction uses follows the drive side of the
 * place: the city's own, chosen when it is founded, or that of a zone laid
 * over part of the planet.
 *
 * Cars move along paths: the lanes of roads and the connections that lead
 * from a lane into an intersection and out of it into a lane of another road.
 * Connections that cross or merge are in conflict. For every conflicting pair
 * the network also records who has the right of way by the rules of the road:
 * the bigger road, then the one going straight over the one turning across,
 * then the one coming from the driver's curb side (the right where traffic
 * keeps right). The traffic hands out intersections by arrival with head
 * starts for bigger roads and going straight; the recorded rule is there for
 * signs, signals and anything that wants the formal answer.
 *
 * Building a road splits the roads it starts on, ends on or crosses, so
 * intersections form by themselves. Every change replaces the paths it
 * touches; until the caller releases them, retired paths stay readable, so
 * the traffic can move its cars onto their successors. */

/* 3.5 m. */
#define FOX_LANE_WIDTH INT64_C(3584)
#define FOX_ROAD_MAX_LANES 3u
#define FOX_NODE_MAX_ROADS 6u
#define FOX_ROAD_MIN_LENGTH FOX_METRES(8)
#define FOX_ROAD_MAX_LENGTH FOX_METRES(1200)
/* A road end this close to a node joins it; one this close to a road joins
 * the road in a new intersection. */
#define FOX_ROAD_SNAP_DISTANCE FOX_METRES(6)
#define FOX_ROAD_MAX_POINTS 512u
#define FOX_NETWORK_MAX_DRIVE_ZONES 64u
#define FOX_NETWORK_NONE UINT32_MAX

typedef enum FoxDriveSide
{
    FOX_DRIVE_RIGHT = 0,
    FOX_DRIVE_LEFT = 1,
} FoxDriveSide;

typedef enum FoxRoadType
{
    /* One lane each way, 40 km/h. */
    FOX_ROAD_STREET = 0,
    /* Two lanes each way, 50 km/h. */
    FOX_ROAD_AVENUE,
    /* Three lanes each way, 60 km/h. */
    FOX_ROAD_BOULEVARD,
    /* Two lanes one way, 50 km/h. */
    FOX_ROAD_ONE_WAY,
    /* Three lanes one way, 90 km/h. */
    FOX_ROAD_HIGHWAY,
    FOX_ROAD_TYPE_COUNT,
} FoxRoadType;

typedef struct FoxRoadTypeInfo
{
    const char *name;
    /* Units per second. */
    int32_t speedLimit;
    uint8_t forwardLanes;
    uint8_t backwardLanes;
    /* Larger roads have priority at intersections. */
    uint8_t rank;
} FoxRoadTypeInfo;

/* NULL for an unknown type. */
const FoxRoadTypeInfo *FoxRoadTypeGet(FoxRoadType type);

typedef enum FoxRoadResult
{
    FOX_ROAD_OK = 0,
    FOX_ROAD_INVALID,
    /* Shorter than FOX_ROAD_MIN_LENGTH, as a whole or between the
     * intersections it would make. */
    FOX_ROAD_TOO_SHORT,
    FOX_ROAD_TOO_LONG,
    /* No ground under some part of the road. */
    FOX_ROAD_NO_GROUND,
    /* A node would carry more than FOX_NODE_MAX_ROADS roads. */
    FOX_ROAD_CROWDED,
    /* Two nodes would be joined twice, or a road would loop onto itself. */
    FOX_ROAD_OVERLAP,
    FOX_ROAD_OUT_OF_MEMORY,
} FoxRoadResult;

typedef struct FoxRoadPlan
{
    FoxVec start;
    FoxVec end;
    /* With `curved`, the road bends towards this point like a quadratic
     * Bezier curve. */
    FoxVec control;
    bool curved;
    FoxRoadType type;
} FoxRoadPlan;

typedef struct FoxNetwork FoxNetwork;

typedef struct FoxNetworkDesc
{
    FoxDriveSide driveSide;
    /* Optional; NULL leaves the network without routing. Must outlive the
     * network. */
    const FoxRouter *router;
} FoxNetworkDesc;

FoxNetwork *FoxNetworkCreate(const FoxNetworkDesc *desc);
void FoxNetworkDestroy(FoxNetwork *network);

/* Counts every change; views compare it to know when to look again. */
uint64_t FoxNetworkRevision(const FoxNetwork *network);
FoxDriveSide FoxNetworkDefaultDriveSide(const FoxNetwork *network);
FoxDriveSide FoxNetworkDriveSideAt(const FoxNetwork *network, FoxVec point);

typedef struct FoxRoadView
{
    uint32_t nodes[2];
    FoxRoadType type;
    FoxDriveSide driveSide;
    const FoxVec *centre;
    uint32_t centreCount;
    int64_t length;
    /* How far the lanes stop short of each node, leaving room for the
     * intersection. */
    int64_t trim[2];
    /* Lane paths: [0] run from nodes[0] to nodes[1], [1] back; index 0 is
     * the curb lane. */
    uint32_t lanes[2][FOX_ROAD_MAX_LANES];
    uint8_t laneCount[2];
    /* The network revision of the road's last change. */
    uint64_t revision;
} FoxRoadView;

typedef struct FoxNodeView
{
    FoxVec position;
    uint32_t roadCount;
    uint32_t roads[FOX_NODE_MAX_ROADS];
    /* The radius of the intersection's paved area. */
    int64_t radius;
    uint64_t revision;
} FoxNodeView;

typedef struct FoxPathView
{
    bool connection;
    const FoxVec *points;
    const int64_t *lengths;
    uint32_t count;
    int64_t length;
    /* Units per second. */
    int32_t speedLimit;
} FoxPathView;

/* Roads, nodes and paths live in slots; iterate up to the capacity. False
 * for an empty slot. */
uint32_t FoxNetworkRoadCapacity(const FoxNetwork *network);
bool FoxNetworkRoad(const FoxNetwork *network, uint32_t road, FoxRoadView *outView);
uint32_t FoxNetworkNodeCapacity(const FoxNetwork *network);
bool FoxNetworkNode(const FoxNetwork *network, uint32_t node, FoxNodeView *outView);
uint32_t FoxNetworkRoadCount(const FoxNetwork *network);
uint32_t FoxNetworkNodeCount(const FoxNetwork *network);
/* Retired paths stay readable until released. */
bool FoxNetworkPath(const FoxNetwork *network, uint32_t path, FoxPathView *outView);

/* A point `distance` along a path and the direction of travel there
 * (FOX_UNIT long); the distance is clamped to the path. */
bool FoxNetworkPathPoint(const FoxNetwork *network, uint32_t path, int64_t distance,
                         FoxVec *outPoint, FoxVec *outDirection);

/* The nearest node within `maxDistance`, or FOX_NETWORK_NONE. */
uint32_t FoxNetworkNearestNode(const FoxNetwork *network, FoxVec point, int64_t maxDistance);
/* The nearest road whose centre line passes within `maxDistance`, with the
 * distance along it; FOX_NETWORK_NONE without one. */
uint32_t FoxNetworkNearestRoad(const FoxNetwork *network, FoxVec point, int64_t maxDistance,
                               int64_t *outAlong);

/* The centre line a plan would get, draped on the ground, without building
 * anything: what a tool previews. Returns the number of points written, zero
 * when the plan is invalid or has no ground. */
uint32_t FoxNetworkPlanCentre(const FoxPlanet *planet, const FoxRoadPlan *plan, FoxVec *points,
                              uint32_t capacity);

/* Builds a road. On anything but FOX_ROAD_OK the network is unchanged. */
FoxRoadResult FoxNetworkBuildRoad(FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan);
/* Removes a road; nodes left without roads go with it. */
bool FoxNetworkRemoveRoad(FoxNetwork *network, uint32_t road);

/* Lays a zone with its own drive side over every road whose middle lies
 * within `radius` of `center`; later zones win where zones overlap. False
 * when the zone table is full or the input invalid. */
bool FoxNetworkAddDriveZone(FoxNetwork *network, FoxVec center, int64_t radius,
                            FoxDriveSide side);
uint32_t FoxNetworkDriveZoneCount(const FoxNetwork *network);

/* Frees the paths retired by the changes so far. The traffic must have moved
 * its cars off them first. */
void FoxNetworkReleaseRetired(FoxNetwork *network);

uint64_t FoxNetworkHash(const FoxNetwork *network);

#endif
