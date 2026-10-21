#include "sim/network_internal.h"

#include <stddef.h>

/* Traffic lights. Opposite approaches share a phase; each phase shows green
 * for at least MIN_GREEN, keeps it while its cars come and nobody else waits,
 * and hands over when others do after its share of CYCLE, at least
 * MIN_GREEN, so a junction of many phases keeps nobody at red for long.
 * When cars wait to turn across the oncoming lanes after the yellow, an
 * arrow lets them go by themselves for a while, with a yellow of its own.
 * A short all-red clears the junction between phases. A phase nobody waits
 * for is skipped. */

#define TICKS 30u
#define MIN_GREEN (8u * TICKS)
#define CYCLE (40u * TICKS)
#define YELLOW (3u * TICKS)
#define ALL_RED (2u * TICKS)
#define MIN_ARROW (3u * TICKS)
#define MAX_ARROW (6u * TICKS)

void FoxSignalsAssign(FoxNetwork *network, uint32_t nodeIndex)
{
    FoxNode *node = &network->nodes[nodeIndex];
    node->phase = 0u;
    node->stage = STAGE_GREEN;
    node->stageTicks = 0u;
    node->phaseCount = 0u;
    if (!node->signals)
    {
        return;
    }
    FoxVec approach[FOX_NODE_MAX_ROADS];
    bool assigned[FOX_NODE_MAX_ROADS] = {false};
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        const FoxRoad *road = &network->roads[node->roads[slot]];
        const FoxVec origin = FoxRoadEndAt(road, nodeIndex) == 0u
                                  ? road->centre[0]
                                  : road->centre[road->centreCount - 1u];
        const int64_t probe = FoxMin(FOX_METRES(6), road->length / 2);
        const int64_t along = FoxRoadEndAt(road, nodeIndex) == 0u ? probe : road->length - probe;
        const FoxVec point =
            FoxPolylinePoint(road->centre, road->centreLengths, road->centreCount, along, NULL);
        approach[slot] = FoxVecDirection(FoxVecSub(point, origin));
    }
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        if (assigned[slot])
        {
            continue;
        }
        const uint8_t phase = node->phaseCount++;
        assigned[slot] = true;
        node->phases[slot] = phase;
        /* The most nearly opposite road still free joins the phase. */
        uint32_t partner = FOX_NETWORK_NONE;
        int64_t best = -(int64_t)FOX_UNIT * FOX_UNIT * 7 / 10;
        for (uint32_t other = slot + 1u; other < node->roadCount; ++other)
        {
            const int64_t dot = FoxVecDot(approach[slot], approach[other]);
            if (!assigned[other] && dot < best)
            {
                best = dot;
                partner = other;
            }
        }
        if (partner != FOX_NETWORK_NONE)
        {
            assigned[partner] = true;
            node->phases[partner] = phase;
        }
    }
}

/* The longest green a phase keeps while others wait. */
static uint32_t MaxGreen(const FoxNode *node)
{
    const uint32_t share = CYCLE / node->phaseCount;
    return share > MIN_GREEN ? share : MIN_GREEN;
}

/* Whether cars are coming along a road end of the given phase. */
static bool Waiting(const FoxNetwork *network, const FoxNode *node, uint32_t nodeIndex,
                    uint8_t phase)
{
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        if (node->phases[slot] != phase)
        {
            continue;
        }
        const FoxRoad *road = &network->roads[node->roads[slot]];
        /* Lanes arriving at nodes[1] run in direction 0. */
        const uint32_t direction = FoxRoadEndAt(road, nodeIndex) == 1u ? 0u : 1u;
        for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
        {
            const FoxPath *path = &network->paths[road->lanes[direction][lane]];
            if (path->vehicleCount != 0u)
            {
                return true;
            }
        }
    }
    return false;
}

/* The phase of the road end a lane arrives at. */
static uint8_t PhaseOf(const FoxNode *node, uint32_t road)
{
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        if (node->roads[slot] == road)
        {
            return node->phases[slot];
        }
    }
    return UINT8_MAX;
}

/* Whether cars of a phase wait to turn across the oncoming lanes, at their
 * line or inside the junction; and forgets who waited, for the traffic to
 * tell again. */
static bool TurnsWaiting(FoxNetwork *network, const FoxNode *node, uint8_t phase)
{
    bool waiting = false;
    for (uint32_t index = 0u; index < node->connectionCount; ++index)
    {
        FoxPath *connection = &network->paths[node->connections[index]];
        waiting = waiting || (connection->movement == MOVE_FAR &&
                              (connection->waited || connection->vehicleCount != 0u) &&
                              PhaseOf(node, network->paths[connection->from].road) == phase);
        connection->waited = false;
    }
    return waiting;
}

void FoxSignalsStep(FoxNetwork *network)
{
    for (uint32_t index = 0u; index < network->nodeCapacity; ++index)
    {
        FoxNode *node = &network->nodes[index];
        if (!node->alive || !node->signals || node->phaseCount == 0u)
        {
            continue;
        }
        ++node->stageTicks;
        const bool turns = TurnsWaiting(network, node, node->phase);
        switch (node->stage)
        {
        case STAGE_GREEN:
        {
            if (node->stageTicks < MIN_GREEN)
            {
                break;
            }
            bool others = false;
            for (uint8_t phase = 0u; phase < node->phaseCount && !others; ++phase)
            {
                others = phase != node->phase && Waiting(network, node, index, phase);
            }
            if (others &&
                (node->stageTicks >= MaxGreen(node) || !Waiting(network, node, index, node->phase)))
            {
                node->stage = STAGE_YELLOW;
                node->stageTicks = 0u;
            }
            break;
        }
        case STAGE_YELLOW:
            if (node->stageTicks >= YELLOW)
            {
                node->stage = turns ? STAGE_ARROW : STAGE_ALL_RED;
                node->stageTicks = 0u;
            }
            break;
        case STAGE_ARROW:
            if (node->stageTicks >= MAX_ARROW || (node->stageTicks >= MIN_ARROW && !turns))
            {
                node->stage = STAGE_ARROW_YELLOW;
                node->stageTicks = 0u;
            }
            break;
        case STAGE_ARROW_YELLOW:
            if (node->stageTicks >= YELLOW)
            {
                node->stage = STAGE_ALL_RED;
                node->stageTicks = 0u;
            }
            break;
        default:
            if (node->stageTicks >= ALL_RED)
            {
                /* The next phase with someone waiting; the next one without. */
                uint8_t next = (uint8_t)((node->phase + 1u) % node->phaseCount);
                for (uint8_t step = 1u; step <= node->phaseCount; ++step)
                {
                    const uint8_t phase = (uint8_t)((node->phase + step) % node->phaseCount);
                    if (Waiting(network, node, index, phase))
                    {
                        next = phase;
                        break;
                    }
                }
                node->phase = next;
                node->stage = STAGE_GREEN;
                node->stageTicks = 0u;
            }
            break;
        }
    }
}

FoxLight FoxSignalsLight(const FoxNetwork *network, uint32_t roadIndex, uint32_t end)
{
    const FoxRoad *road = &network->roads[roadIndex];
    const uint32_t nodeIndex = road->nodes[end];
    const FoxNode *node = &network->nodes[nodeIndex];
    if (!node->alive || !node->signals)
    {
        return FOX_LIGHT_NONE;
    }
    for (uint32_t slot = 0u; slot < node->roadCount; ++slot)
    {
        if (node->roads[slot] != roadIndex)
        {
            continue;
        }
        if (node->phases[slot] != node->phase || node->stage >= STAGE_ARROW)
        {
            return FOX_LIGHT_RED;
        }
        return node->stage == STAGE_GREEN ? FOX_LIGHT_GREEN : FOX_LIGHT_YELLOW;
    }
    return FOX_LIGHT_NONE;
}

FoxLight FoxSignalsArrow(const FoxNetwork *network, uint32_t roadIndex, uint32_t end)
{
    const FoxNode *node = &network->nodes[network->roads[roadIndex].nodes[end]];
    if (!node->alive || !node->signals || PhaseOf(node, roadIndex) != node->phase)
    {
        return FOX_LIGHT_NONE;
    }
    return node->stage == STAGE_ARROW          ? FOX_LIGHT_GREEN
           : node->stage == STAGE_ARROW_YELLOW ? FOX_LIGHT_YELLOW
                                               : FOX_LIGHT_NONE;
}
