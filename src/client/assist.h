#ifndef FOX_CLIENT_ASSIST_H
#define FOX_CLIENT_ASSIST_H

#include "sim/network.h"
#include "sim/planet.h"

#include <stdbool.h>
#include <stdint.h>

/* Help with drawing roads: the point under the pointer snaps to what lies
 * near it, as the settings allow. In order: a node; along the angle steps
 * from the start, measured from the road being continued, to a road, a guide
 * or a whole number of length steps; a road; guides, the lines of the roads
 * at nearby nodes and their perpendiculars, and where two of them cross; the
 * grid. Everything is worked out in the plane touching the ground there and
 * put back on the ground. */

/* What a road being drawn snaps to. */
#define FOX_SNAP_NETWORK 1u
#define FOX_SNAP_ANGLE 2u
#define FOX_SNAP_LENGTH 4u
#define FOX_SNAP_GRID 8u
#define FOX_SNAP_GUIDES 16u
/* A curve leaves a road's end the way the road was going. */
#define FOX_SNAP_TANGENT 32u

typedef enum FoxSnapKind
{
    FOX_SNAPPED_FREE = 0,
    FOX_SNAPPED_NODE,
    FOX_SNAPPED_ROAD,
    FOX_SNAPPED_GUIDE,
    FOX_SNAPPED_ANGLE,
    FOX_SNAPPED_GRID,
} FoxSnapKind;

typedef struct FoxAssistRequest
{
    /* The pointer on the ground. */
    FoxVec raw;
    /* The point the road is drawn from, or none yet. */
    bool hasStart;
    FoxVec start;
    /* Choosing the point a curve bends towards. */
    bool curveControl;
    /* FOX_SNAP_* flags, the angle step in degrees, the length and grid step
     * in metres, and how far a snap reaches, metres. */
    uint32_t snaps;
    double angleStep;
    double lengthStep;
    double reach;
} FoxAssistRequest;

/* A line to show the snap by: a guide followed or the angle drawn along. */
typedef struct FoxAssistLine
{
    FoxVec from;
    FoxVec to;
    bool guide;
} FoxAssistLine;

#define FOX_ASSIST_MAX_LINES 3u

typedef struct FoxAssist
{
    FoxVec point;
    FoxSnapKind kind;
    FoxAssistLine lines[FOX_ASSIST_MAX_LINES];
    uint32_t lineCount;
    /* From the start: the straight distance in metres, and the angle in
     * degrees to the way the road would continue, or to north. */
    double metres;
    double degrees;
} FoxAssist;

/* False when there is no ground under the result. */
bool FoxAssistPoint(const FoxNetwork *network, const FoxPlanet *planet,
                    const FoxAssistRequest *request, FoxAssist *out);

#endif
