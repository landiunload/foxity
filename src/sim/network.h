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
 * where cars turn around. A road is laid out across in strips, built one at
 * a time: lanes for cars, paved in asphalt or concrete, in the middle, and
 * along either edge a bike lane and outside it a footway half a lane wide.
 * Car lanes run one way or the other, and a road of a single lane may be
 * driven both ways; which side of the road a direction uses follows the
 * drive side of the place: the city's own, chosen when it is founded, or
 * that of a zone laid over part of the planet. Where a road that runs one
 * way only ends in a dead end, its innermost lane turns into a slow way back
 * for the cars caught there, a lone lane into one driven both ways.
 *
 * Cars move along paths: the lanes of roads and the connections that lead
 * from a lane into an intersection and out of it into a lane of another road.
 * Connections that cross or merge are in conflict. For every conflicting pair
 * the network also records who has the right of way by the rules of the road:
 * the one with the priority sign, then the one going straight over the one
 * turning across, then the one coming from the driver's curb side (the right
 * where traffic keeps right).
 *
 * Every road end at an intersection carries a sign for the traffic arriving
 * there. Left to itself the network signs intersections like a road
 * authority: where roads of different size meet, the biggest get the
 * priority sign and the others give way; equal roads get none. The mayor can
 * set any end to priority, give way or stop, put traffic lights on an
 * intersection and set the speed limit of a road.
 *
 * Building a road splits the roads it starts on, ends on or crosses, so
 * intersections form by themselves. Every change replaces the paths it
 * touches; until the caller releases them, retired paths stay readable, so
 * the traffic can move its cars onto their successors. */

/* 3.5 m. */
#define FOX_LANE_WIDTH INT64_C(3584)
/* Paved beyond the outer lanes, about 0.4 m. */
#define FOX_ROAD_SHOULDER INT64_C(410)
/* A footway is half a lane wide. */
#define FOX_FOOT_WIDTH (FOX_LANE_WIDTH / 2)
/* A speed in km/h as units per second. */
#define FOX_KMH(kmh) ((int32_t)(FOX_METRES(kmh) * 10 / 36))
/* Speed limits a sign can show. */
#define FOX_SPEED_LIMIT_MIN FOX_KMH(10)
#define FOX_SPEED_LIMIT_MAX ((int32_t)FOX_ROUTER_MAX_SPEED)
#define FOX_ROAD_MAX_LANES 3u
/* The car lanes and a bike lane and a footway at either edge. */
#define FOX_ROAD_MAX_STRIPS (2u * FOX_ROAD_MAX_LANES + 4u)
#define FOX_ROAD_MAX_CROSSINGS 8u
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

/* What a strip across a road is: a lane cars drive, paved in asphalt or in
 * concrete; a bike lane; or a footway. */
typedef enum FoxLaneKind
{
    FOX_LANE_ASPHALT = 0,
    FOX_LANE_CONCRETE,
    FOX_LANE_BIKE,
    FOX_LANE_FOOT,
    FOX_LANE_KIND_COUNT,
} FoxLaneKind;

/* Which way a strip is used, seen from nodes[0] of its road. */
typedef enum FoxWay
{
    FOX_WAY_NONE = 0,
    FOX_WAY_FORWARD,
    FOX_WAY_BACKWARD,
    FOX_WAY_BOTH,
} FoxWay;

/* How a road is laid out across. The car lanes lie in the middle, centred on
 * the centre line, each way's on the side its drive side keeps to; along
 * either edge may run a bike lane and outside it a footway. A road of only
 * paths centres its strips on the centre line. */
typedef struct FoxRoadLayout
{
    /* Car lanes: [0] from nodes[0] to nodes[1], [1] back. A shared lane is
     * one lane driven both ways and counts once each way. */
    uint8_t lanes[2];
    bool shared;
    /* Each car lane's paving, FOX_LANE_ASPHALT or FOX_LANE_CONCRETE, by
     * direction and lane from the curb. */
    uint8_t surface[2][FOX_ROAD_MAX_LANES];
    /* At either edge, [0] on the left seen from nodes[0] and [1] on the
     * right: a bike lane's way, FOX_WAY_NONE for none, and whether a
     * footway runs outside it. */
    uint8_t bike[2];
    bool foot[2];
} FoxRoadLayout;

/* One strip across a road: what it is, which way it is used and where its
 * left and right edges lie, units right of the centre line. */
typedef struct FoxRoadStrip
{
    uint8_t kind;
    uint8_t way;
    int64_t left;
    int64_t right;
} FoxRoadStrip;

/* Ready layouts for building whole roads at once, as the demo city does. */
typedef enum FoxRoadType
{
    /* One lane each way and footways, 50 km/h. */
    FOX_ROAD_STREET = 0,
    /* Two lanes each way and footways, 60 km/h. */
    FOX_ROAD_AVENUE,
    /* Three lanes each way, bike lanes and footways, 70 km/h. */
    FOX_ROAD_BOULEVARD,
    /* Two lanes one way, 50 km/h. */
    FOX_ROAD_ONE_WAY,
    /* Three lanes one way, 110 km/h. */
    FOX_ROAD_HIGHWAY,
    FOX_ROAD_TYPE_COUNT,
} FoxRoadType;

/* A ready layout, and its limit in units per second; a road of
 * FOX_ROAD_STREET for an unknown type. */
FoxRoadLayout FoxRoadLayoutPreset(FoxRoadType type);
int32_t FoxRoadPresetSpeed(FoxRoadType type);
/* A single strip of `kind`, a lane cars drive from nodes[0] for the paving
 * kinds: what a road is when it is first drawn. */
FoxRoadLayout FoxRoadLayoutSingle(FoxLaneKind kind);
/* Whether a layout can be built: some strip, no more car lanes each way
 * than FOX_ROAD_MAX_LANES, a shared lane only alone, known pavings. */
bool FoxRoadLayoutValid(const FoxRoadLayout *layout);
/* Car lanes across the road, a shared one counted once. */
uint32_t FoxRoadLayoutCarLanes(const FoxRoadLayout *layout);
/* Half the width of the car lanes, zero without any. */
int64_t FoxRoadLayoutCarHalfWidth(const FoxRoadLayout *layout);
/* How far the road reaches to the left [0] and right [1] of its centre
 * line, without shoulders. */
void FoxRoadLayoutExtent(const FoxRoadLayout *layout, int64_t outExtent[2]);
/* The strips from left to right, seen from nodes[0], where traffic keeps to
 * `side`; returns how many. */
uint32_t FoxRoadLayoutStrips(const FoxRoadLayout *layout, FoxDriveSide side,
                             FoxRoadStrip outStrips[FOX_ROAD_MAX_STRIPS]);
/* The default limit of a layout, units per second: by its car lanes, a
 * shared lane slower. */
int32_t FoxRoadLayoutSpeed(const FoxRoadLayout *layout);

/* The sign facing the traffic that arrives at a road end. */
typedef enum FoxSign
{
    /* Chosen by the network from the roads that meet. */
    FOX_SIGN_AUTO = 0,
    /* No sign: equal roads, first come first served. */
    FOX_SIGN_NONE,
    FOX_SIGN_PRIORITY,
    FOX_SIGN_YIELD,
    /* Stop at the line, then give way. */
    FOX_SIGN_STOP,
    FOX_SIGN_COUNT,
} FoxSign;

typedef enum FoxLight
{
    /* No traffic lights here. */
    FOX_LIGHT_NONE = 0,
    FOX_LIGHT_GREEN,
    FOX_LIGHT_YELLOW,
    FOX_LIGHT_RED,
} FoxLight;

/* Where the lanes of a road lead at an intersection, as seen by the driver. */
#define FOX_TURN_STRAIGHT 1u
#define FOX_TURN_LEFT 2u
#define FOX_TURN_RIGHT 4u
#define FOX_TURN_BACK 8u

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
    /* A curve tighter than the road's width allows: its inner edge would
     * fold over itself. */
    FOX_ROAD_TOO_SHARP,
} FoxRoadResult;

typedef struct FoxRoadPlan
{
    FoxVec start;
    FoxVec end;
    /* With `curved`, the road bends towards this point like a quadratic
     * Bezier curve. */
    FoxVec control;
    bool curved;
    FoxRoadLayout layout;
    /* The limit, units per second; zero for the layout's. */
    int32_t speedLimit;
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
    /* As built; the lanes may differ where it runs one way into a dead end. */
    FoxRoadLayout layout;
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
    /* The limit in force, units per second, and whether the mayor set it. */
    int32_t speedLimit;
    bool speedSet;
    /* For the traffic arriving at nodes[end]: the sign in force (never
     * FOX_SIGN_AUTO) and the one the mayor chose. */
    FoxSign sign[2];
    FoxSign signSetting[2];
    /* Pedestrian crossings, how far along the centre line each lies: the
     * mayor's and those where a path crosses the road. */
    int64_t crossings[FOX_ROAD_MAX_CROSSINGS];
    uint32_t crossingCount;
    /* The network revision of the road's last change. */
    uint64_t revision;
} FoxRoadView;

typedef struct FoxNodeView
{
    FoxVec position;
    uint32_t roadCount;
    /* Of them, those with car lanes: three make it an intersection for cars. */
    uint32_t carRoadCount;
    uint32_t roads[FOX_NODE_MAX_ROADS];
    /* The radius of the intersection's paved area. */
    int64_t radius;
    bool signals;
    /* Paths alone meeting on a road's centre line: they stop at the road's
     * edge and the node itself is not paved. */
    bool tap;
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
/* A removed road cars still drive off, which FoxNetworkRoad no longer shows:
 * only to be drawn until they are gone. */
bool FoxNetworkClosingRoad(const FoxNetwork *network, uint32_t road, FoxRoadView *outView);
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

/* Whether a road could be built as planned, and why not: everything
 * FoxNetworkBuildRoad checks, without changing anything. */
FoxRoadResult FoxNetworkCheckRoad(const FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan);
/* Builds a road. On anything but FOX_ROAD_OK the network is unchanged. */
FoxRoadResult FoxNetworkBuildRoad(FoxNetwork *network, const FoxPlanet *planet,
                                  const FoxRoadPlan *plan);
/* Removes a road; nodes left without roads go with it. A road with cars on
 * it or on their way onto it closes instead: it is gone from the views,
 * the routes and the choices of everyone else at once, but it carries those
 * cars on as it was until FoxNetworkSettleClosing takes it out. */
bool FoxNetworkRemoveRoad(FoxNetwork *network, uint32_t road);
/* Takes out the closing roads nobody is on or coming onto any more, and
 * with `radius` above zero every one reaching within it of `center`, where
 * the network is about to change; true when one went, and the traffic must
 * then follow the change. */
bool FoxNetworkSettleClosing(FoxNetwork *network, FoxVec center, int64_t radius);

/* Lays a zone with its own drive side over every road whose middle lies
 * within `radius` of `center`; later zones win where zones overlap. False
 * when the zone table is full or the input invalid. */
bool FoxNetworkAddDriveZone(FoxNetwork *network, FoxVec center, int64_t radius, FoxDriveSide side);
uint32_t FoxNetworkDriveZoneCount(const FoxNetwork *network);

/* Adds a strip of `kind` along an edge of a road, [0] its left seen from
 * nodes[0] and [1] its right: a car lane runs `forward` from nodes[0] or
 * back, as does a bike lane, outside the car lanes; a footway goes on the
 * outside. FOX_ROAD_INVALID when the layout would not be valid, or when it
 * would not fit between the roads around. */
FoxRoadResult FoxNetworkWidenRoad(FoxNetwork *network, uint32_t road, uint32_t side,
                                  FoxLaneKind kind, bool forward);
/* Turns the strip of a road at `offset` across it, units right of the
 * centre line: a car lane goes the other way, a lone car lane one way, the
 * other, both ways in turn, and a bike lane the same. False where no car or
 * bike lane lies there. */
bool FoxNetworkTurnStrip(FoxNetwork *network, uint32_t road, int64_t offset);
/* Puts a pedestrian crossing over a road `along` its centre line, or takes
 * the one there away. False for a road without car lanes, too near an end
 * or with FOX_ROAD_MAX_CROSSINGS already. */
/* Takes away the strip `offset` units right of a road's centre line (seen
 * from nodes[0]), shoulders aside: a car lane, a bike lane or a footway. The
 * last car lane, or the last strip of a path, takes the whole road;
 * FOX_ROAD_OVERLAP where a path ending on the road would lose its edge. */
FoxRoadResult FoxNetworkNarrowRoad(FoxNetwork *network, uint32_t road, int64_t offset);
bool FoxNetworkToggleCrossing(FoxNetwork *network, uint32_t road, int64_t along);

/* Sets a road's speed limit, FOX_SPEED_LIMIT_MIN..FOX_SPEED_LIMIT_MAX, or
 * back to its layout's with zero. False for an invalid road or speed. */
bool FoxNetworkSetSpeedLimit(FoxNetwork *network, uint32_t road, int32_t speed);
/* Sets the sign facing the traffic arriving at nodes[end] of a road. */
bool FoxNetworkSetSign(FoxNetwork *network, uint32_t road, uint32_t end, FoxSign sign);
/* Puts traffic lights on an intersection of three or more roads, or takes
 * them away. False for anything else. */
bool FoxNetworkSetSignals(FoxNetwork *network, uint32_t node, bool on);
/* The light shown to the traffic arriving at nodes[end] of a road. */
FoxLight FoxNetworkLight(const FoxNetwork *network, uint32_t road, uint32_t end);
/* The arrow shown beside it for turning across the oncoming lanes, which
 * may go by themselves after the others; FOX_LIGHT_NONE while it is dark. */
FoxLight FoxNetworkArrow(const FoxNetwork *network, uint32_t road, uint32_t end);
/* FOX_TURN_* bits of where a lane leads at its end; zero for anything but a
 * live lane. */
uint32_t FoxNetworkLaneTurns(const FoxNetwork *network, uint32_t lane);

/* Frees the paths retired by the changes so far. The traffic must have moved
 * its cars off them first. */
void FoxNetworkReleaseRetired(FoxNetwork *network);

/* The pieces of road centre lines and the intersections whose ground an
 * edit near them must leave: gathered once per edit, only those the edit
 * can reach, so each sample asks a handful. */
#define FOX_GROUND_GUARD_MAX 512u

typedef struct FoxGuardPiece
{
    FoxVec a;
    FoxVec b;
    /* Up from the planet at its middle, of FOX_UNIT. */
    FoxVec up;
    FoxVec middle;
    /* Kept this far across from the line between them; nothing kept lies
     * further than `bound` from the middle. */
    int64_t across;
    int64_t bound;
} FoxGuardPiece;

typedef struct FoxGroundGuard
{
    FoxGuardPiece pieces[FOX_GROUND_GUARD_MAX];
    uint32_t pieceCount;
    /* More was in reach than fits: the edit keeps everything. */
    bool overflow;
} FoxGroundGuard;

/* Gathers what an edit of `radius` around `center` could reach. */
void FoxNetworkGuardGround(const FoxNetwork *network, FoxVec center, int64_t radius,
                           FoxGroundGuard *outGuard);
/* Whether a road or an intersection keeps the ground at a point: across,
 * within its paved width and a margin; up and down, from a few metres under
 * it to the room a truck needs over it, so it can be neither dug away nor
 * buried, though ground heaped higher roofs it over as a tunnel. */
bool FoxGroundGuardKeeps(const FoxGroundGuard *guard, FoxVec point);
/* Whether a box may hold any ground the guard keeps. */
bool FoxGroundGuardTouches(const FoxGroundGuard *guard, FoxVec min, FoxVec max);

/* Everything the network holds, counted. */
typedef struct FoxNetworkStats
{
    uint32_t roads;
    /* Roads by how many car lanes they carry across, and those of paths only. */
    uint32_t roadsByLanes[2u * FOX_ROAD_MAX_LANES + 1u];
    uint32_t crossings;
    int64_t footwayLength;
    uint32_t nodes;
    /* Nodes by what they are: three or more roads, two, one. */
    uint32_t junctions;
    uint32_t bends;
    uint32_t deadEnds;
    uint32_t signalled;
    /* The signs in force at the road ends of junctions without lights. */
    uint32_t signs[FOX_SIGN_COUNT];
    uint32_t speedLimitsSet;
    uint32_t lanes;
    uint32_t connections;
    /* Pairs of connections that cross or merge, each counted once. */
    uint32_t conflicts;
    int64_t roadLength;
    int64_t laneLength;
} FoxNetworkStats;

FoxNetworkStats FoxNetworkGetStats(const FoxNetwork *network);

uint64_t FoxNetworkHash(const FoxNetwork *network);

#endif
