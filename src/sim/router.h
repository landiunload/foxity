#ifndef FOX_SIM_ROUTER_H
#define FOX_SIM_ROUTER_H

#include "sim/space.h"

#include <stdbool.h>
#include <stdint.h>

/* Path finding over the road graph, supplied from outside the simulation:
 * the application implements it with the engine's navigation module. Nodes
 * are road nodes; an edge is one direction of a road, numbered road * 2 +
 * direction. Costs are travel times in milliseconds. The implementation must
 * be deterministic: the same calls give the same routes on every machine,
 * and ties go the same way. Without a router cars choose turns at random. */
typedef struct FoxRouter
{
    void *context;
    /* Adds, moves or (alive false) removes a node. */
    void (*setNode)(void *context, uint32_t node, FoxVec position, bool alive);
    /* Adds or updates an edge; cost 0 removes it. */
    void (*setEdge)(void *context, uint32_t edge, uint32_t fromNode, uint32_t toNode,
                    uint32_t cost);
    /* The cheapest route as edges into `edges`; the number of edges, zero
     * when there is none or it does not fit. */
    uint32_t (*findRoute)(void *context, uint32_t fromNode, uint32_t toNode, uint32_t *edges,
                          uint32_t capacity);
} FoxRouter;

/* The fastest any road may be driven, units per second: routing estimates
 * never exceed the real cost. */
#define FOX_ROUTER_MAX_SPEED (FOX_METRES(25))

#endif
