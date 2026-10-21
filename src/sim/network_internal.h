#ifndef FOX_SIM_NETWORK_INTERNAL_H
#define FOX_SIM_NETWORK_INTERNAL_H

/* The network's own records, shared with the traffic that drives on them.
 * Nothing outside src/sim includes this header. */

#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum PathKind
{
    PATH_LANE = 1,
    PATH_CONNECTION = 2,
} PathKind;

typedef enum Movement
{
    MOVE_STRAIGHT = 0,
    /* Towards the curb side: right where traffic keeps right. */
    MOVE_CURB,
    /* Across the oncoming lanes. */
    MOVE_FAR,
    MOVE_U_TURN,
} Movement;

typedef struct FoxConflict
{
    uint32_t path;
    /* This connection yields to the other one. */
    bool yields;
    /* How far along the other connection the two meet: past it a car on
     * the other one no longer stands in this one's way. */
    int64_t otherAt;
} FoxConflict;

typedef struct FoxPath
{
    uint8_t kind;
    bool retired;
    uint32_t generation;
    FoxVec *points;
    /* Distance from the first point; lengths[count - 1] is the length. */
    int64_t *lengths;
    uint32_t count;
    int64_t length;
    /* The lower of the road's limit and what the path's tightest bend
     * allows. */
    int32_t speedLimit;
    int32_t curveSpeed;

    /* Lanes: the road, its generation, the direction (0 runs from nodes[0]
     * to nodes[1]), the lane (0 at the curb), the distance along the road's
     * centre line at each point and the connections leaving the lane's end. */
    uint32_t road;
    uint32_t roadGeneration;
    uint8_t direction;
    uint8_t lane;
    int64_t *centre;
    uint32_t *outgoing;
    uint32_t outgoingCount;

    /* Connections: the node, the lanes joined, how it turns and the paths it
     * conflicts with. */
    uint32_t node;
    uint32_t from;
    uint32_t to;
    uint8_t movement;
    FoxConflict *conflicts;
    uint32_t conflictCount;

    /* Owned by the traffic: the cars on the path from front to back; on
     * connections the cars holding or using it, and on lanes the length of
     * the cars that have claimed a way in but are not on the lane yet. */
    uint32_t front;
    uint32_t back;
    uint32_t vehicleCount;
    uint32_t claims;
    uint32_t incoming;
} FoxPath;

/* Traffic light stages within a phase. */
typedef enum SignalStage
{
    STAGE_GREEN = 0,
    STAGE_YELLOW,
    STAGE_ALL_RED,
} SignalStage;

typedef struct FoxRoad
{
    bool alive;
    uint32_t generation;
    uint32_t nodes[2];
    uint8_t type;
    uint8_t driveSide;
    FoxVec *centre;
    int64_t *centreLengths;
    uint32_t centreCount;
    int64_t length;
    int64_t trim[2];
    uint32_t lanes[2][FOX_ROAD_MAX_LANES];
    uint8_t laneCount[2];
    uint64_t revision;
    /* The limit in force and the mayor's, zero for the type's. */
    int32_t speedLimit;
    int32_t speedSetting;
    /* Per end, for the traffic arriving there: the mayor's sign and the one
     * in force. */
    uint8_t signSetting[2];
    uint8_t sign[2];
    FoxVec middle;
    int64_t boundRadius;
    /* Traffic statistics per direction, for routing costs. */
    int64_t speedSum[2];
    uint32_t speedSamples[2];
    int32_t averageSpeed[2];
    uint32_t routedCost[2];
} FoxRoad;

typedef struct FoxNode
{
    bool alive;
    uint32_t generation;
    FoxVec position;
    FoxVec up;
    uint8_t driveSide;
    uint32_t roadCount;
    uint32_t roads[FOX_NODE_MAX_ROADS];
    uint32_t *connections;
    uint32_t connectionCount;
    int64_t radius;
    uint64_t revision;
    /* Traffic lights: the phase each road slot belongs to, the phase showing
     * green, its stage and the ticks spent in it. */
    bool signals;
    uint8_t phaseCount;
    uint8_t phases[FOX_NODE_MAX_ROADS];
    uint8_t phase;
    uint8_t stage;
    uint32_t stageTicks;
} FoxNode;

/* Where a replaced road went: the part of its centre line in [start, end)
 * now belongs to `road`, starting at `offset` of that road's centre line. */
typedef struct FoxLineage
{
    uint32_t oldRoad;
    uint32_t oldGeneration;
    uint32_t road;
    uint32_t generation;
    int64_t start;
    int64_t end;
    int64_t offset;
} FoxLineage;

typedef struct FoxDriveZone
{
    FoxVec center;
    int64_t radius;
    uint8_t side;
} FoxDriveZone;

struct FoxNetwork
{
    uint8_t defaultSide;
    const FoxRouter *router;
    uint64_t revision;

    FoxRoad *roads;
    uint32_t roadCapacity;
    uint32_t roadCount;
    FoxNode *nodes;
    uint32_t nodeCapacity;
    uint32_t nodeCount;
    FoxPath *paths;
    uint32_t pathCapacity;

    /* Paths waiting for FoxNetworkReleaseRetired, and where replaced roads
     * went since the last release. */
    uint32_t *retired;
    uint32_t retiredCount;
    uint32_t retiredCapacity;
    FoxLineage *lineage;
    uint32_t lineageCount;
    uint32_t lineageCapacity;

    FoxDriveZone zones[FOX_NETWORK_MAX_DRIVE_ZONES];
    uint32_t zoneCount;
};

/* ---- network.c ---------------------------------------------------------- */

const FoxPath *FoxNetworkPathRecord(const FoxNetwork *network, uint32_t path);
FoxPath *FoxNetworkPathRecordMutable(FoxNetwork *network, uint32_t path);
bool FoxNetworkPathLive(const FoxNetwork *network, uint32_t path);

/* Interpolates a polyline at a distance; the index of the segment holding it
 * is written to outIndex (may be NULL). */
FoxVec FoxPolylinePoint(const FoxVec *points, const int64_t *lengths, uint32_t count,
                        int64_t distance, uint32_t *outIndex);
/* Fills lengths[0..count) with the distances from the first point. */
int64_t FoxPolylineMeasure(const FoxVec *points, int64_t *lengths, uint32_t count);

/* ---- lanes.c ------------------------------------------------------------ */

/* Replaces the lanes of a road from its centre line and trims, retiring the
 * old ones. False without memory; the road keeps no lanes then. */
bool FoxLanesBuildRoad(FoxNetwork *network, uint32_t road);
/* Replaces the connections at a node, retiring the old ones. */
bool FoxLanesBuildNode(FoxNetwork *network, uint32_t node);
/* The trims of every road at a node and the node's paved radius. */
void FoxLanesMeasureNode(FoxNetwork *network, uint32_t node);
/* Allocates a path slot; FOX_NETWORK_NONE without memory. */
uint32_t FoxNetworkAllocatePath(FoxNetwork *network);
/* Marks a path retired; it is freed by the next release. */
bool FoxNetworkRetirePath(FoxNetwork *network, uint32_t path);

/* Lane distance <-> centre line distance, interpolated over a lane's points.
 * Both directions are monotonic along a lane. */
int64_t FoxLaneToCentre(const FoxPath *lane, int64_t distance);
int64_t FoxCentreToLane(const FoxPath *lane, int64_t centre);

/* The signs in force at a node's road ends, from the mayor's and the roads. */
void FoxLanesResolveSigns(FoxNetwork *network, uint32_t node);
/* Rebuilds which of a node's connections conflict and who yields. */
bool FoxLanesBuildConflicts(FoxNetwork *network, uint32_t node);
/* Applies the roads' limits to the lanes of a road / the connections of a
 * node, in place. */
void FoxLanesApplyRoadSpeed(FoxNetwork *network, uint32_t road);
void FoxLanesApplyNodeSpeeds(FoxNetwork *network, uint32_t node);
/* Which way the end of a road at a node leads, the end index. */
uint32_t FoxRoadEndAt(const FoxRoad *road, uint32_t node);

/* ---- signals.c ---------------------------------------------------------- */

/* Groups a node's road ends into phases: opposite approaches share one. */
void FoxSignalsAssign(FoxNetwork *network, uint32_t node);
/* Advances every traffic light by a tick; phases with cars waiting get
 * green, empty ones are skipped. */
void FoxSignalsStep(FoxNetwork *network);
FoxLight FoxSignalsLight(const FoxNetwork *network, uint32_t road, uint32_t end);
/* Whether the green of a road end has just ended and the intersection is
 * being cleared before the next phase. */
bool FoxSignalsClearing(const FoxNetwork *network, uint32_t road, uint32_t end);

#endif
