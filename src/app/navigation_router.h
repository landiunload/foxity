#ifndef FOX_APP_NAVIGATION_ROUTER_H
#define FOX_APP_NAVIGATION_ROUTER_H

#include "navigation/navigation_service.h"
#include "sim/router.h"

#include <stdbool.h>
#include <stdint.h>

/* The simulation's router on the engine's laiue.navigation: road nodes and
 * directions become graph nodes and edges, routes come from its deterministic
 * A*, and congestion updates reach it as cheap edge cost changes. */

typedef struct FoxNavigationEdge
{
    LaiueNavEdge handle;
    uint32_t from;
    uint32_t to;
} FoxNavigationEdge;

typedef struct FoxNavigationRouter
{
    FoxRouter router;
    const LaiueNavigationServiceV1 *service;
    LaiueNavGraph *graph;
    LaiueNavQuery *query;
    LaiueNavNode *nodes;
    uint32_t nodeCapacity;
    FoxNavigationEdge *edges;
    uint32_t edgeCapacity;
    LaiueNavEdge *path;
    uint32_t pathCapacity;
    /* Something could not be stored: routes may be missing, never wrong. */
    bool degraded;
} FoxNavigationRouter;

/* False when the service is missing parts or memory ran out; the router is
 * then unusable and needs no destroy. */
bool FoxNavigationRouterInit(FoxNavigationRouter *router, const LaiueNavigationServiceV1 *service,
                             uint32_t serviceSize);
void FoxNavigationRouterDestroy(FoxNavigationRouter *router);

#endif
