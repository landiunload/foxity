#include "app/navigation_router.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Routing costs are milliseconds and distances units: no road is faster than
 * FOX_ROUTER_MAX_SPEED, so 1000 / FOX_ROUTER_MAX_SPEED milliseconds per unit
 * never overestimates. */
#define HEURISTIC_NUMERATOR 1000u
#define HEURISTIC_DENOMINATOR ((uint32_t)FOX_ROUTER_MAX_SPEED)
#define TRAVEL_MODE 1u

_Static_assert(HEURISTIC_DENOMINATOR <= 65535u, "the navigation heuristic takes 16-bit terms");

static bool Grow(void **array, uint32_t *capacity, uint32_t needed, size_t elementSize)
{
    if (needed <= *capacity)
    {
        return true;
    }
    uint32_t grown = *capacity < 64u ? 64u : *capacity;
    while (grown < needed)
    {
        if (grown > UINT32_MAX / 2u)
        {
            return false;
        }
        grown *= 2u;
    }
    void *bigger = realloc(*array, (size_t)grown * elementSize);
    if (bigger == NULL)
    {
        return false;
    }
    memset((uint8_t *)bigger + (size_t)*capacity * elementSize, 0,
           (size_t)(grown - *capacity) * elementSize);
    *array = bigger;
    *capacity = grown;
    return true;
}

static void RemoveEdge(FoxNavigationRouter *router, uint32_t edge)
{
    if (edge < router->edgeCapacity && router->edges[edge].handle != 0u)
    {
        (void)router->service->graphRemoveEdge(router->graph, router->edges[edge].handle);
        router->edges[edge].handle = 0u;
    }
}

static void SetNode(void *context, uint32_t node, FoxVec position, bool alive)
{
    FoxNavigationRouter *router = (FoxNavigationRouter *)context;
    const LaiueNavPosition place = {position.x, position.y, position.z};
    if (!alive)
    {
        if (node < router->nodeCapacity && router->nodes[node] != 0u)
        {
            /* The graph drops the node's edges with it. */
            (void)router->service->graphRemoveNode(router->graph, router->nodes[node]);
            router->nodes[node] = 0u;
            for (uint32_t edge = 0u; edge < router->edgeCapacity; ++edge)
            {
                if (router->edges[edge].handle != 0u &&
                    (router->edges[edge].from == node || router->edges[edge].to == node))
                {
                    router->edges[edge].handle = 0u;
                }
            }
        }
        return;
    }
    if (!Grow((void **)&router->nodes, &router->nodeCapacity, node + 1u, sizeof(*router->nodes)))
    {
        router->degraded = true;
        return;
    }
    if (router->nodes[node] != 0u)
    {
        (void)router->service->graphSetNodePosition(router->graph, router->nodes[node], &place);
        return;
    }
    if (router->service->graphAddNode(router->graph, &place, node, &router->nodes[node]) !=
        LAIUE_NAV_OK)
    {
        router->nodes[node] = 0u;
        router->degraded = true;
    }
}

static void SetEdge(void *context, uint32_t edge, uint32_t from, uint32_t to, uint32_t cost)
{
    FoxNavigationRouter *router = (FoxNavigationRouter *)context;
    if (cost == 0u)
    {
        RemoveEdge(router, edge);
        return;
    }
    if (!Grow((void **)&router->edges, &router->edgeCapacity, edge + 1u, sizeof(*router->edges)))
    {
        router->degraded = true;
        return;
    }
    FoxNavigationEdge *record = &router->edges[edge];
    if (record->handle != 0u && record->from == from && record->to == to)
    {
        if (router->service->graphSetEdgeCost(router->graph, record->handle, cost) != LAIUE_NAV_OK)
        {
            router->degraded = true;
        }
        return;
    }
    RemoveEdge(router, edge);
    if (from >= router->nodeCapacity || to >= router->nodeCapacity || router->nodes[from] == 0u ||
        router->nodes[to] == 0u)
    {
        router->degraded = true;
        return;
    }
    const LaiueNavEdgeDesc desc = {
        .structSize = sizeof(desc),
        .cost = cost,
        .modes = TRAVEL_MODE,
        .userData = edge,
    };
    if (router->service->graphAddEdge(router->graph, router->nodes[from], router->nodes[to], &desc,
                                      &record->handle) != LAIUE_NAV_OK)
    {
        record->handle = 0u;
        router->degraded = true;
        return;
    }
    record->from = from;
    record->to = to;
}

static uint32_t FindRoute(void *context, uint32_t from, uint32_t to, uint32_t *edges,
                          uint32_t capacity)
{
    FoxNavigationRouter *router = (FoxNavigationRouter *)context;
    if (from >= router->nodeCapacity || to >= router->nodeCapacity || router->nodes[from] == 0u ||
        router->nodes[to] == 0u || capacity == 0u)
    {
        return 0u;
    }
    if (!router->service->graphIsPrepared(router->graph) &&
        router->service->graphPrepare(router->graph) != LAIUE_NAV_OK)
    {
        return 0u;
    }
    if (!Grow((void **)&router->path, &router->pathCapacity, capacity, sizeof(*router->path)))
    {
        return 0u;
    }
    const LaiueNavPathRequest request = {
        .structSize = sizeof(request),
        .start = router->nodes[from],
        .goal = router->nodes[to],
        .modes = TRAVEL_MODE,
        .heuristicNumerator = HEURISTIC_NUMERATOR,
        .heuristicDenominator = HEURISTIC_DENOMINATOR,
    };
    LaiueNavPathResult result = {.structSize = sizeof(result)};
    router->service->findPath(router->graph, router->query, &request, router->path, capacity,
                              &result);
    if (result.pathStatus != LAIUE_NAV_PATH_FOUND || result.count > capacity)
    {
        return 0u;
    }
    for (uint32_t index = 0u; index < result.count; ++index)
    {
        LaiueNavEdgeInfo info = {.structSize = sizeof(info)};
        if (router->service->graphGetEdge(router->graph, router->path[index], &info) !=
            LAIUE_NAV_OK)
        {
            return 0u;
        }
        edges[index] = info.userData;
    }
    return result.count;
}

bool FoxNavigationRouterInit(FoxNavigationRouter *router, const LaiueNavigationServiceV1 *service,
                             uint32_t serviceSize)
{
    if (router == NULL)
    {
        return false;
    }
    memset(router, 0, sizeof(*router));
    if (service == NULL || serviceSize < sizeof(*service) || service->structSize < sizeof(*service) ||
        service->abiVersion != LAIUE_NAVIGATION_SERVICE_ABI_VERSION_1 ||
        service->graphCreate == NULL || service->graphDestroy == NULL ||
        service->graphAddNode == NULL || service->graphRemoveNode == NULL ||
        service->graphSetNodePosition == NULL || service->graphAddEdge == NULL ||
        service->graphRemoveEdge == NULL || service->graphGetEdge == NULL ||
        service->graphSetEdgeCost == NULL || service->graphPrepare == NULL ||
        service->graphIsPrepared == NULL || service->queryCreate == NULL ||
        service->queryDestroy == NULL || service->findPath == NULL)
    {
        return false;
    }
    router->service = service;
    const LaiueNavGraphDesc desc = {.structSize = sizeof(desc)};
    if (service->graphCreate(&desc, &router->graph) != LAIUE_NAV_OK)
    {
        return false;
    }
    if (service->queryCreate(&router->query) != LAIUE_NAV_OK)
    {
        service->graphDestroy(router->graph);
        return false;
    }
    router->router = (FoxRouter){
        .context = router,
        .setNode = SetNode,
        .setEdge = SetEdge,
        .findRoute = FindRoute,
    };
    return true;
}

void FoxNavigationRouterDestroy(FoxNavigationRouter *router)
{
    if (router == NULL || router->service == NULL)
    {
        return;
    }
    router->service->queryDestroy(router->query);
    router->service->graphDestroy(router->graph);
    free(router->nodes);
    free(router->edges);
    free(router->path);
    memset(router, 0, sizeof(*router));
}
