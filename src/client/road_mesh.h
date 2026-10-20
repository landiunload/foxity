#ifndef FOX_CLIENT_ROAD_MESH_H
#define FOX_CLIENT_ROAD_MESH_H

#include "graphics/graphics_device_v2.h"
#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* Geometry of the road network, in metres relative to an origin.
 *
 * Nothing overlaps: a road's asphalt stops where its lanes do, and the node
 * fills the rest — an intersection with a paved area bounded by curved curbs
 * between its roads, a bend with a piece of road that carries the lanes and
 * markings on, a dead end with a rounded end. The base mesh holds the
 * asphalt; the detail mesh the markings, signs, posts and traffic lights,
 * which only matter up close. Markings follow the road and its signs: lane
 * lines that turn solid before intersections, stop and give-way lines,
 * arrows for where each lane leads. */

typedef struct FoxMesh
{
    LaiueGraphicsVertexV2 *vertices;
    uint32_t count;
    uint32_t capacity;
    bool failed;
} FoxMesh;

void FoxMeshReset(FoxMesh *mesh);
void FoxMeshFree(FoxMesh *mesh);

/* A traffic light: where its lamps are, red, yellow and green from the top,
 * and the road end whose light it shows. */
typedef struct FoxSignalHead
{
    FoxVec lamps[3];
    uint32_t road;
    uint32_t end;
} FoxSignalHead;

typedef struct FoxSignalHeads
{
    FoxSignalHead *items;
    uint32_t count;
    uint32_t capacity;
    bool failed;
} FoxSignalHeads;

/* A road between its trims, with its markings, signs and lights. */
void FoxRoadMeshRoad(const FoxNetwork *network, uint32_t road, FoxVec origin, FoxMesh *base,
                     FoxMesh *detail, FoxSignalHeads *heads);
/* What a node paves: an intersection, a bend or a rounded dead end. */
void FoxRoadMeshNode(const FoxNetwork *network, uint32_t node, FoxVec origin, FoxMesh *base,
                     FoxMesh *detail);
/* A road being planned, in one colour with rounded ends. */
void FoxRoadMeshPreview(const FoxVec *centre, uint32_t count, FoxRoadType type, uint32_t color,
                        FoxVec origin, FoxMesh *mesh);

/* Half the paved width of a road type, metres. */
double FoxRoadMeshHalfWidth(FoxRoadType type);

#endif
