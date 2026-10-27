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
    /* Where people walk, either way along it: a footway, a crossing over a
     * road, or a corner from one footway round to the next. */
    PATH_WALK = 3,
} PathKind;

typedef enum WalkKind
{
    WALK_FOOTWAY = 0,
    WALK_CROSSING,
    WALK_CORNER,
} WalkKind;

/* How many walks one end of a walk meets at most. */
#define FOX_WALK_MAX_JOINS 4u
/* A walk's end, as the walks name each other's: path * 2 + end. */
#define FOX_WALK_END(path, end) ((path) * 2u + (end))

typedef enum Movement
{
    MOVE_STRAIGHT = 0,
    /* Towards the curb side: right where traffic keeps right. */
    MOVE_CURB,
    /* Across the oncoming lanes. */
    MOVE_FAR,
    MOVE_U_TURN,
} Movement;

/* A dead end's turn changes direction at most this often. */
#define FOX_PATH_MAX_CUSPS 6u

typedef struct FoxConflict
{
    uint32_t path;
    /* This connection yields to the other one. */
    bool yields;
    /* How far along the other connection the two meet: past it a car on
     * the other one no longer stands in this one's way. */
    int64_t otherAt;
    /* How far short of where they meet a car keeps clear of the other way,
     * and how far past it its rear must be: the sharper the angle the two
     * cross at, the further. The same for both. */
    int64_t room;
} FoxConflict;

/* Another way from the same lane, and how far along this one a car's rear
 * must be before a car taking that way can no longer touch it. */
typedef struct FoxSibling
{
    uint32_t path;
    int64_t parting;
} FoxSibling;

typedef struct FoxPath
{
    uint8_t kind;
    bool retired;
    /* Lanes: the direction (0 runs from nodes[0] to nodes[1]) and the lane
     * (0 at the curb). */
    uint8_t direction;
    uint8_t lane;
    uint32_t generation;
    FoxVec *points;
    /* Distance from the first point; lengths[count - 1] is the length. */
    int64_t *lengths;
    int64_t length;
    uint32_t count;
    /* The lower of the road's limit and what the path's tightest bend
     * allows. */
    int32_t speedLimit;
    int32_t curveSpeed;

    /* Lanes: the road, its generation, the distance along the road's centre
     * line at each point and the connections leaving the lane's end. */
    uint32_t road;
    uint32_t roadGeneration;
    uint32_t outgoingCount;
    int64_t *centre;
    uint32_t *outgoing;

    /* Connections: the paths it conflicts with and the other ways from its
     * lane, the node, the lanes joined and how it turns. */
    FoxConflict *conflicts;
    FoxSibling *siblings;
    uint32_t conflictCount;
    uint32_t siblingCount;
    uint32_t node;
    uint32_t from;
    uint32_t to;
    uint8_t movement;
    /* A dead end's turn traces the rear axle rather than the front, and is
     * driven forward up to its first cusp, in reverse to the next, and so
     * on: how far along each cusp lies. */
    bool axle;
    uint8_t cuspCount;
    int64_t cusps[FOX_PATH_MAX_CUSPS];

    /* Lanes: the pedestrian crossings over them, how many, how far along
     * the lane each lies and the walk across it, FOX_NETWORK_NONE where
     * nobody can walk it. Walks: which kind, and what each end meets, as
     * FOX_WALK_END; a crossing also how far along the road it lies. The
     * walkers on a crossing and those stepping onto it count in `claims`. */
    int64_t crossingAt[FOX_ROAD_MAX_CROSSINGS];
    int64_t roadAlong;

    /* Owned by the traffic: the cars on the path from front to back; on
     * connections the cars holding or using it, and on lanes the length of
     * the cars that have claimed a way in but are not on the lane yet. */
    uint32_t front;
    uint32_t back;
    uint32_t vehicleCount;
    uint32_t claims;
    uint32_t incoming;
    /* On connections: a car stood waiting to take it in the last round. */
    bool waited;

    uint8_t crossingCount;
    uint8_t walkKind;
    uint8_t joinCount[2];
    uint32_t crossingWalk[FOX_ROAD_MAX_CROSSINGS];
    uint32_t joins[2][FOX_WALK_MAX_JOINS];
} FoxPath;

/* Traffic light stages within a phase. */
typedef enum SignalStage
{
    STAGE_GREEN = 0,
    STAGE_YELLOW,
    /* Only the turns across the oncoming lanes go, then their yellow. */
    STAGE_ARROW,
    STAGE_ARROW_YELLOW,
    STAGE_ALL_RED,
} SignalStage;

typedef struct FoxRoad
{
    FoxVec *centre;
    int64_t *centreLengths;
    int64_t length;
    uint64_t revision;
    int64_t boundRadius;
    int64_t trim[2];
    /* Traffic statistics per direction, for routing costs. */
    int64_t speedSum[2];
    FoxVec middle;
    /* Pedestrian crossings in order along the centre line. */
    int64_t crossings[FOX_ROAD_MAX_CROSSINGS];
    /* Every crossing to be seen, the mayor's and those of paths, in order. */
    int64_t zebras[FOX_ROAD_MAX_CROSSINGS];
    uint32_t generation;
    uint32_t centreCount;
    /* The limit in force, the one it was built with (zero for its layout's)
     * and the mayor's (zero for none). */
    int32_t speedLimit;
    int32_t builtSpeed;
    int32_t speedSetting;
    uint32_t nodes[2];
    uint32_t speedSamples[2];
    int32_t averageSpeed[2];
    uint32_t routedCost[2];
    uint32_t lanes[2][FOX_ROAD_MAX_LANES];
    /* The walks along its footways, [side][piece] from nodes[0] on, split at
     * its crossings and taps, and the walk over each crossing,
     * FOX_NETWORK_NONE where nobody can walk it. */
    uint32_t crossingWalks[FOX_ROAD_MAX_CROSSINGS];
    uint32_t walks[2][FOX_ROAD_MAX_CROSSINGS + 1u];
    /* The nodes where paths meet it from the side, in no order: a path
     * crossing it leaves a crossing there. Per tap, from lanes.c: the walk
     * over it (FOX_NETWORK_NONE without), which footway pieces meet there
     * (pieces [tapPiece] and [tapPiece + 1]) and which sides paths leave
     * from, bit 0 the left seen from nodes[0]. */
    uint32_t taps[FOX_ROAD_MAX_CROSSINGS];
    uint32_t tapWalks[FOX_ROAD_MAX_CROSSINGS];
    bool alive;
    /* Removed while cars were still on it or on their way onto it: gone for
     * everyone else, it carries them on as it was until they are off it. */
    bool closing;
    uint8_t driveSide;
    uint8_t crossingCount;
    uint8_t tapCount;
    uint8_t zebraCount;
    uint8_t laneCount[2];
    uint8_t walkCount[2];
    /* Per end, for the traffic arriving there: the mayor's sign and the one
     * in force. */
    uint8_t signSetting[2];
    uint8_t sign[2];
    uint8_t tapPiece[FOX_ROAD_MAX_CROSSINGS];
    uint8_t tapSides[FOX_ROAD_MAX_CROSSINGS];
    /* As built, and as driven: the same but where a road running one way
     * ends in a dead end, which lanes.c fits with a way back out. */
    FoxRoadLayout layout;
    FoxRoadLayout driven;
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
    /* The walks round its corners, from one footway to the next; at a tap,
     * from each path onto the road it meets. */
    uint32_t *corners;
    uint32_t cornerCount;
    /* Where only paths meet on a road's centre line rather than at its end:
     * the road, FOX_NETWORK_NONE elsewhere, and how far along it. Its paths
     * stop at the road's edge and walk on along its footways or over it. */
    uint32_t tap;
    int64_t tapAlong;
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
    uint32_t closingCount;
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

/* Roads with car lanes at a node, closing ones too: a node with one is a
 * dead end for cars, with two a bend, whatever paths meet there besides. */
uint32_t FoxNodeCarRoads(const FoxNetwork *network, uint32_t node);
/* The limit a road keeps without the mayor's: as built, or its layout's. */
int32_t FoxRoadOwnSpeed(const FoxRoad *road);
/* How far right of the centre line a car lane runs, by its direction and its
 * index from the curb; a shared lane's two ways keep a quarter lane to
 * either side. */
int64_t FoxRoadLaneOffset(const FoxRoadLayout *layout, FoxDriveSide side, uint32_t direction,
                          uint32_t lane);
/* The widest a road reaches either side of its centre line, shoulder
 * included. */
int64_t FoxRoadReach(const FoxRoad *road);
/* A point `offset` units right of a road's centre line `along` it (seen from
 * nodes[0]), and the unit vector to the right there. */
FoxVec FoxRoadPointAt(const FoxRoad *road, int64_t along, int64_t offset, FoxVec *outRight);

/* ---- lanes.c ------------------------------------------------------------ */

/* Replaces the lanes of a road from its centre line and trims, retiring the
 * old ones. False without memory; the road keeps no lanes then. */
bool FoxLanesBuildRoad(FoxNetwork *network, uint32_t road);
/* Retires the walks along a road's footways and over its crossings. */
void FoxLanesRetireWalks(FoxNetwork *network, FoxRoad *road);
/* Replaces the connections at a node, and the walks round its corners,
 * retiring the old ones. */
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
/* The arrow for the turns across the oncoming lanes from a road end:
 * FOX_LIGHT_NONE while it is dark and the main light rules them. */
FoxLight FoxSignalsArrow(const FoxNetwork *network, uint32_t road, uint32_t end);
/* Ticks before the yellow showing at a node turns red; zero without one. */
uint32_t FoxSignalsYellowLeft(const FoxNetwork *network, uint32_t node);

#endif
