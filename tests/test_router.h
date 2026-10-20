#ifndef FOX_TESTS_TEST_ROUTER_H
#define FOX_TESTS_TEST_ROUTER_H

/* A plain Dijkstra router for tests of the simulation alone. */

#include "sim/router.h"

#include <stdint.h>
#include <string.h>

#define TEST_ROUTER_NODES 512u
#define TEST_ROUTER_EDGES 2048u

typedef struct TestRouter
{
    FoxRouter router;
    uint8_t nodeAlive[TEST_ROUTER_NODES];
    uint32_t from[TEST_ROUTER_EDGES];
    uint32_t to[TEST_ROUTER_EDGES];
    uint32_t cost[TEST_ROUTER_EDGES];
    uint64_t distance[TEST_ROUTER_NODES];
    uint32_t via[TEST_ROUTER_NODES];
    uint8_t done[TEST_ROUTER_NODES];
    uint32_t calls;
} TestRouter;

static void TestRouterSetNode(void *context, uint32_t node, FoxVec position, bool alive)
{
    TestRouter *router = (TestRouter *)context;
    (void)position;
    if (node < TEST_ROUTER_NODES)
    {
        router->nodeAlive[node] = alive ? 1u : 0u;
    }
}

static void TestRouterSetEdge(void *context, uint32_t edge, uint32_t fromNode, uint32_t toNode,
                              uint32_t cost)
{
    TestRouter *router = (TestRouter *)context;
    if (edge < TEST_ROUTER_EDGES)
    {
        router->from[edge] = fromNode;
        router->to[edge] = toNode;
        router->cost[edge] = cost;
    }
}

static uint32_t TestRouterFind(void *context, uint32_t fromNode, uint32_t toNode, uint32_t *edges,
                               uint32_t capacity)
{
    TestRouter *router = (TestRouter *)context;
    ++router->calls;
    if (fromNode >= TEST_ROUTER_NODES || toNode >= TEST_ROUTER_NODES)
    {
        return 0u;
    }
    for (uint32_t node = 0u; node < TEST_ROUTER_NODES; ++node)
    {
        router->distance[node] = UINT64_MAX;
        router->via[node] = UINT32_MAX;
        router->done[node] = 0u;
    }
    router->distance[fromNode] = 0u;
    for (;;)
    {
        uint32_t best = UINT32_MAX;
        for (uint32_t node = 0u; node < TEST_ROUTER_NODES; ++node)
        {
            if (!router->done[node] && router->distance[node] != UINT64_MAX &&
                (best == UINT32_MAX || router->distance[node] < router->distance[best]))
            {
                best = node;
            }
        }
        if (best == UINT32_MAX || best == toNode)
        {
            break;
        }
        router->done[best] = 1u;
        for (uint32_t edge = 0u; edge < TEST_ROUTER_EDGES; ++edge)
        {
            if (router->cost[edge] != 0u && router->from[edge] == best)
            {
                const uint64_t through = router->distance[best] + router->cost[edge];
                if (through < router->distance[router->to[edge]])
                {
                    router->distance[router->to[edge]] = through;
                    router->via[router->to[edge]] = edge;
                }
            }
        }
    }
    if (router->distance[toNode] == UINT64_MAX)
    {
        return 0u;
    }
    uint32_t count = 0u;
    for (uint32_t node = toNode; node != fromNode; node = router->from[router->via[node]])
    {
        ++count;
    }
    if (count > capacity)
    {
        return 0u;
    }
    uint32_t index = count;
    for (uint32_t node = toNode; node != fromNode; node = router->from[router->via[node]])
    {
        edges[--index] = router->via[node];
    }
    return count;
}

static void TestRouterInit(TestRouter *router)
{
    memset(router, 0, sizeof(*router));
    router->router = (FoxRouter){
        .context = router,
        .setNode = TestRouterSetNode,
        .setEdge = TestRouterSetEdge,
        .findRoute = TestRouterFind,
    };
}

#endif
