#include "sim/walkers.h"

#include "sim/network_internal.h"
#include "sim/random.h"

#include <stdlib.h>
#include <string.h>

#define NONE FOX_NETWORK_NONE
#define TICKS FOX_TRAFFIC_TICKS_PER_SECOND
/* A walker's own pace, and how far to the right of its way it keeps so two
 * meeting pass each other. */
#define SLOWEST (FOX_METRES(11) / 10)
#define FASTEST (FOX_METRES(17) / 10)
#define KEEP_RIGHT (FOX_METRES(2) / 5)
/* One in so many ways to choose is a crossing a walker goes over. */
#define CROSS_ONE_IN 3u
/* A walker gives up on a crossing after waiting this long. */
#define PATIENCE_TICKS (20u * TICKS)
#define SPAWN_PER_TICK 4u
/* After a change a walker finds the nearest footway within this. */
#define REMAP_REACH FOX_METRES(40)
#define COLORS 8u

typedef struct Walker
{
    bool alive;
    uint8_t state;
    uint8_t color;
    /* +1 walking towards the end of its path, -1 towards its start. */
    int8_t heading;
    uint32_t path;
    uint32_t pathGeneration;
    int64_t s;
    int32_t speed;
    /* Waiting at the kerb: the crossing it wants and the end it steps on at. */
    uint32_t crossing;
    uint8_t crossingEnd;
    uint32_t waitTicks;
    /* Where it stood the tick before, to draw it in between. */
    FoxVec last;
    uint64_t serial;
    FoxRandom random;
} Walker;

struct FoxWalkers
{
    uint64_t seed;
    uint32_t target;
    Walker *walkers;
    uint32_t capacity;
    uint32_t count;
    uint64_t nextSerial;
    uint64_t crossed;
    FoxRandom random;
};

FoxWalkers *FoxWalkersCreate(const FoxWalkersDesc *desc)
{
    if (desc == NULL || desc->target > FOX_WALKERS_MAX)
    {
        return NULL;
    }
    FoxWalkers *walkers = calloc(1u, sizeof(*walkers));
    if (walkers == NULL)
    {
        return NULL;
    }
    walkers->seed = desc->seed;
    walkers->target = desc->target;
    walkers->random = FoxRandomStream(desc->seed, UINT64_C(0x77616C6B));
    return walkers;
}

void FoxWalkersDestroy(FoxWalkers *walkers)
{
    if (walkers != NULL)
    {
        free(walkers->walkers);
        free(walkers);
    }
}

void FoxWalkersSetTarget(FoxWalkers *walkers, uint32_t target)
{
    if (walkers != NULL)
    {
        walkers->target = target < FOX_WALKERS_MAX ? target : FOX_WALKERS_MAX;
    }
}

uint32_t FoxWalkersTarget(const FoxWalkers *walkers)
{
    return walkers == NULL ? 0u : walkers->target;
}

static const FoxPath *Live(const FoxNetwork *network, uint32_t path)
{
    const FoxPath *record = FoxNetworkPathRecord(network, path);
    return record != NULL && !record->retired && record->kind == PATH_WALK ? record : NULL;
}

/* Where a walker on a path stands, a little to the right of its way. */
static FoxVec Place(const FoxNetwork *network, const Walker *walker, FoxVec *outFacing)
{
    FoxVec point = {0, 0, 0};
    FoxVec direction = {0, 0, 0};
    if (!FoxNetworkPathPoint(network, walker->path, walker->s, &point, &direction))
    {
        return walker->last;
    }
    const FoxVec facing = walker->heading > 0 ? direction : FoxVecNegate(direction);
    const FoxVec right = FoxVecDirection(FoxVecCross(facing, FoxPlanetUp(point)));
    if (outFacing != NULL)
    {
        *outFacing = facing;
    }
    return FoxVecAdd(point, FoxVecScale(right, KEEP_RIGHT, FOX_UNIT));
}

static uint32_t AcquireSlot(FoxWalkers *walkers)
{
    for (uint32_t slot = 0u; slot < walkers->capacity; ++slot)
    {
        if (!walkers->walkers[slot].alive)
        {
            return slot;
        }
    }
    const uint32_t capacity = walkers->capacity == 0u ? 64u : walkers->capacity * 2u;
    Walker *grown = realloc(walkers->walkers, (size_t)capacity * sizeof(*grown));
    if (grown == NULL)
    {
        return NONE;
    }
    memset(grown + walkers->capacity, 0, (size_t)(capacity - walkers->capacity) * sizeof(*grown));
    const uint32_t slot = walkers->capacity;
    walkers->walkers = grown;
    walkers->capacity = capacity;
    return slot;
}

/* A footway picked by length, so walkers spread evenly. */
static uint32_t RandomFootway(FoxWalkers *walkers, const FoxNetwork *network)
{
    int64_t total = 0;
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        const FoxPath *path = &network->paths[index];
        total += path->kind == PATH_WALK && !path->retired && path->walkKind == WALK_FOOTWAY
                     ? path->length
                     : 0;
    }
    if (total <= 0)
    {
        return NONE;
    }
    int64_t pick = FoxRandomRange(&walkers->random, 0, total - 1);
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        const FoxPath *path = &network->paths[index];
        const int64_t share =
            path->kind == PATH_WALK && !path->retired && path->walkKind == WALK_FOOTWAY
                ? path->length
                : 0;
        if (pick < share)
        {
            return index;
        }
        pick -= share;
    }
    return NONE;
}

static void Spawn(FoxWalkers *walkers, const FoxNetwork *network)
{
    for (uint32_t spawn = 0u; spawn < SPAWN_PER_TICK && walkers->count < walkers->target; ++spawn)
    {
        const uint32_t path = RandomFootway(walkers, network);
        const uint32_t slot = path == NONE ? NONE : AcquireSlot(walkers);
        if (slot == NONE)
        {
            return;
        }
        const uint64_t serial = ++walkers->nextSerial;
        Walker walker = {
            .alive = true,
            .path = path,
            .pathGeneration = network->paths[path].generation,
            .crossing = NONE,
            .serial = serial,
            .random = FoxRandomStream(walkers->seed, serial ^ UINT64_C(0x70656F706C65)),
        };
        walker.heading = FoxRandomBelow(&walker.random, 2u) == 0u ? 1 : -1;
        walker.s = FoxRandomRange(&walker.random, 0, network->paths[path].length);
        walker.speed = (int32_t)FoxRandomRange(&walker.random, SLOWEST, FASTEST);
        walker.color = (uint8_t)FoxRandomBelow(&walker.random, COLORS);
        walker.last = Place(network, &walker, NULL);
        walkers->walkers[slot] = walker;
        ++walkers->count;
    }
}

/* Steps onto the walk `end` names, at that end, heading inwards; `over`
 * already walked past the end it left. */
static void Enter(const FoxNetwork *network, Walker *walker, uint32_t end, int64_t over)
{
    const uint32_t path = end / 2u;
    const FoxPath *record = &network->paths[path];
    walker->path = path;
    walker->pathGeneration = record->generation;
    walker->heading = end % 2u == 0u ? 1 : -1;
    walker->s = end % 2u == 0u ? FoxMin(over, record->length) : FoxMax(0, record->length - over);
}

/* At an end of its walk: on round a corner or along the next footway, over
 * a crossing after waiting for the cars, or back the way it came. */
static void ChooseOn(FoxWalkers *walkers, const FoxNetwork *network, Walker *walker, uint32_t end,
                     int64_t over)
{
    const FoxPath *path = &network->paths[walker->path];
    uint32_t ways[FOX_WALK_MAX_JOINS];
    uint32_t crossings[FOX_WALK_MAX_JOINS];
    uint32_t wayCount = 0u;
    uint32_t crossingCount = 0u;
    for (uint32_t index = 0u; index < path->joinCount[end]; ++index)
    {
        const uint32_t join = path->joins[end][index];
        const FoxPath *next = Live(network, join / 2u);
        if (next == NULL)
        {
            continue;
        }
        if (next->walkKind == WALK_CROSSING)
        {
            crossings[crossingCount++] = join;
        }
        else
        {
            ways[wayCount++] = join;
        }
    }
    if (crossingCount != 0u &&
        (wayCount == 0u || FoxRandomBelow(&walker->random, CROSS_ONE_IN) == 0u))
    {
        const uint32_t join = crossings[FoxRandomBelow(&walker->random, crossingCount)];
        walker->state = FOX_WALKER_WAITING;
        walker->crossing = join / 2u;
        walker->crossingEnd = (uint8_t)(join % 2u);
        walker->waitTicks = 0u;
        walker->s = end == 0u ? 0 : path->length;
        return;
    }
    if (wayCount != 0u)
    {
        Enter(network, walker, ways[FoxRandomBelow(&walker->random, wayCount)], over);
        return;
    }
    /* Nowhere on: back along the same walk. */
    walker->heading = (int8_t)-walker->heading;
    walker->s = end == 0u ? FoxMin(over, path->length) : FoxMax(0, path->length - over);
    (void)walkers;
}

static void Leave(FoxWalkers *walkers, Walker *walker)
{
    walker->alive = false;
    --walkers->count;
}

static void StepWalker(FoxWalkers *walkers, FoxNetwork *network, const FoxTraffic *traffic,
                       Walker *walker)
{
    walker->last = Place(network, walker, NULL);
    if (walker->state == FOX_WALKER_WAITING)
    {
        if (Live(network, walker->crossing) == NULL || ++walker->waitTicks > PATIENCE_TICKS)
        {
            /* Not today: on along the footway instead. */
            walker->state = FOX_WALKER_WALKING;
            walker->crossing = NONE;
            walker->heading = (int8_t)-walker->heading;
            return;
        }
        if (!FoxTrafficCrossingClear(traffic, network, walker->crossing))
        {
            return;
        }
        FoxPath *crossing = FoxNetworkPathRecordMutable(network, walker->crossing);
        ++crossing->claims;
        walker->state = FOX_WALKER_CROSSING;
        Enter(network, walker, FOX_WALK_END(walker->crossing, walker->crossingEnd), 0);
        walker->crossing = NONE;
        return;
    }
    const FoxPath *path = &network->paths[walker->path];
    const int64_t step = ((int64_t)walker->speed + TICKS / 2) / TICKS;
    walker->s += walker->heading > 0 ? step : -step;
    if (walker->s > 0 && walker->s < path->length)
    {
        return;
    }
    const uint32_t end = walker->s <= 0 ? 0u : 1u;
    const int64_t over = end == 0u ? -walker->s : walker->s - path->length;
    if (walker->state == FOX_WALKER_CROSSING)
    {
        FoxPath *crossing = FoxNetworkPathRecordMutable(network, walker->path);
        crossing->claims = crossing->claims != 0u ? crossing->claims - 1u : 0u;
        walker->state = FOX_WALKER_WALKING;
        ++walkers->crossed;
    }
    else if (walkers->count > walkers->target && path->walkKind == WALK_FOOTWAY)
    {
        /* Home from here. */
        Leave(walkers, walker);
        return;
    }
    ChooseOn(walkers, network, walker, end, over);
}

void FoxWalkersStep(FoxWalkers *walkers, FoxNetwork *network, const FoxTraffic *traffic)
{
    if (walkers == NULL || network == NULL || traffic == NULL)
    {
        return;
    }
    for (uint32_t slot = 0u; slot < walkers->capacity; ++slot)
    {
        Walker *walker = &walkers->walkers[slot];
        if (walker->alive)
        {
            StepWalker(walkers, network, traffic, walker);
        }
    }
    Spawn(walkers, network);
}

/* The nearest live footway to a point, and how far along it. */
static uint32_t NearestFootway(const FoxNetwork *network, FoxVec point, int64_t *outS)
{
    uint32_t best = NONE;
    int64_t bestDistance = REMAP_REACH;
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        const FoxPath *path = &network->paths[index];
        if (path->kind != PATH_WALK || path->retired || path->walkKind != WALK_FOOTWAY ||
            path->count < 2u)
        {
            continue;
        }
        for (uint32_t segment = 0u; segment + 1u < path->count; ++segment)
        {
            const FoxVec a = path->points[segment];
            const FoxVec ab = FoxVecSub(path->points[segment + 1u], a);
            const int64_t lengthSquared = FoxVecDot(ab, ab);
            const int64_t t =
                lengthSquared > 0
                    ? FoxClamp(FoxVecDot(FoxVecSub(point, a), ab) * 1024 / lengthSquared, 0, 1024)
                    : 0;
            const FoxVec closest = FoxVecAdd(a, FoxVecScale(ab, t, 1024));
            const int64_t distance = FoxVecDistance(closest, point);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = index;
                *outS = path->lengths[segment] +
                        (path->lengths[segment + 1u] - path->lengths[segment]) * t / 1024;
            }
        }
    }
    return best;
}

void FoxWalkersRemap(FoxWalkers *walkers, FoxNetwork *network)
{
    if (walkers == NULL || network == NULL)
    {
        return;
    }
    for (uint32_t slot = 0u; slot < walkers->capacity; ++slot)
    {
        Walker *walker = &walkers->walkers[slot];
        if (!walker->alive)
        {
            continue;
        }
        if (walker->state == FOX_WALKER_WAITING && Live(network, walker->crossing) == NULL)
        {
            walker->state = FOX_WALKER_WALKING;
            walker->crossing = NONE;
        }
        const FoxPath *path = FoxNetworkPathRecord(network, walker->path);
        if (path != NULL && !path->retired && path->generation == walker->pathGeneration)
        {
            continue;
        }
        /* Its walk went: to the nearest footway left, or home. */
        int64_t s = 0;
        const uint32_t footway = NearestFootway(network, Place(network, walker, NULL), &s);
        if (footway == NONE)
        {
            Leave(walkers, walker);
            continue;
        }
        walker->path = footway;
        walker->pathGeneration = network->paths[footway].generation;
        walker->s = s;
        walker->state = FOX_WALKER_WALKING;
        walker->crossing = NONE;
    }
    /* Who is on a crossing is counted afresh. */
    for (uint32_t index = 0u; index < network->pathCapacity; ++index)
    {
        if (network->paths[index].kind == PATH_WALK)
        {
            network->paths[index].claims = 0u;
        }
    }
    for (uint32_t slot = 0u; slot < walkers->capacity; ++slot)
    {
        const Walker *walker = &walkers->walkers[slot];
        if (walker->alive && walker->state == FOX_WALKER_CROSSING)
        {
            ++network->paths[walker->path].claims;
        }
    }
}

uint32_t FoxWalkersCapacity(const FoxWalkers *walkers)
{
    return walkers == NULL ? 0u : walkers->capacity;
}

bool FoxWalkersWalker(const FoxWalkers *walkers, uint32_t slot, FoxWalkerView *outView)
{
    if (walkers == NULL || outView == NULL || slot >= walkers->capacity ||
        !walkers->walkers[slot].alive)
    {
        return false;
    }
    const Walker *walker = &walkers->walkers[slot];
    *outView = (FoxWalkerView){
        .serial = walker->serial,
        .path = walker->path,
        .distance = walker->s,
        .state = (FoxWalkerState)walker->state,
        .color = walker->color,
    };
    return true;
}

bool FoxWalkersPose(const FoxWalkers *walkers, const FoxNetwork *network, uint32_t slot,
                    int64_t blend, FoxVec *outPosition, FoxVec *outFacing)
{
    if (walkers == NULL || network == NULL || slot >= walkers->capacity ||
        !walkers->walkers[slot].alive || outPosition == NULL || outFacing == NULL)
    {
        return false;
    }
    const Walker *walker = &walkers->walkers[slot];
    FoxVec facing = {0, 0, 0};
    const FoxVec now = Place(network, walker, &facing);
    *outPosition = FoxVecLerp(walker->last, now, FoxClamp(blend, 0, FOX_UNIT));
    *outFacing = facing;
    return true;
}

FoxWalkersStats FoxWalkersGetStats(const FoxWalkers *walkers)
{
    FoxWalkersStats stats = {0};
    if (walkers == NULL)
    {
        return stats;
    }
    stats.crossed = walkers->crossed;
    for (uint32_t slot = 0u; slot < walkers->capacity; ++slot)
    {
        const Walker *walker = &walkers->walkers[slot];
        if (walker->alive)
        {
            ++stats.walkers;
            stats.waiting += walker->state == FOX_WALKER_WAITING ? 1u : 0u;
            stats.crossing += walker->state == FOX_WALKER_CROSSING ? 1u : 0u;
        }
    }
    return stats;
}

uint64_t FoxWalkersHash(const FoxWalkers *walkers)
{
    uint64_t hash = 0u;
    for (uint32_t slot = 0u; walkers != NULL && slot < walkers->capacity; ++slot)
    {
        const Walker *walker = &walkers->walkers[slot];
        if (walker->alive)
        {
            hash = FoxHashCombine(hash, walker->serial);
            hash = FoxHashCombine(hash, walker->path);
            hash = FoxHashCombine(hash, (uint64_t)walker->s);
            hash = FoxHashCombine(hash, walker->state);
        }
    }
    return hash;
}
