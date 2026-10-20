#include "sim/random.h"

#include <stddef.h>

#define FOX_GOLDEN_GAMMA UINT64_C(0x9E3779B97F4A7C15)

uint64_t FoxHash64(uint64_t value)
{
    value ^= value >> 30;
    value *= UINT64_C(0xBF58476D1CE4E5B9);
    value ^= value >> 27;
    value *= UINT64_C(0x94D049BB133111EB);
    value ^= value >> 31;
    return value;
}

uint64_t FoxHashCombine(uint64_t seed, uint64_t value)
{
    return FoxHash64(seed ^ FoxHash64(value + FOX_GOLDEN_GAMMA));
}

FoxRandom FoxRandomStream(uint64_t seed, uint64_t stream)
{
    return (FoxRandom){.state = FoxHashCombine(seed, stream)};
}

uint64_t FoxRandomNext(FoxRandom *random)
{
    random->state += FOX_GOLDEN_GAMMA;
    return FoxHash64(random->state);
}

uint32_t FoxRandomBelow(FoxRandom *random, uint32_t bound)
{
    if (bound == 0u)
    {
        return 0u;
    }
    /* Lemire's multiply-and-reject: unbiased, usually one draw. */
    uint64_t product = (FoxRandomNext(random) >> 32) * (uint64_t)bound;
    uint32_t low = (uint32_t)product;
    if (low < bound)
    {
        const uint32_t threshold = (0u - bound) % bound;
        while (low < threshold)
        {
            product = (FoxRandomNext(random) >> 32) * (uint64_t)bound;
            low = (uint32_t)product;
        }
    }
    return (uint32_t)(product >> 32);
}

int64_t FoxRandomRange(FoxRandom *random, int64_t minimum, int64_t maximum)
{
    if (maximum <= minimum)
    {
        return minimum;
    }
    const uint64_t span = (uint64_t)maximum - (uint64_t)minimum;
    if (span == UINT64_MAX)
    {
        return (int64_t)FoxRandomNext(random);
    }
    const uint64_t count = span + 1u;
    /* Rejection keeps the result unbiased for any span. */
    const uint64_t limit = UINT64_MAX - UINT64_MAX % count;
    uint64_t value = FoxRandomNext(random);
    while (value >= limit)
    {
        value = FoxRandomNext(random);
    }
    return (int64_t)((uint64_t)minimum + value % count);
}

int FoxRandomChance(FoxRandom *random, uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0u || numerator == 0u)
    {
        return 0;
    }
    if (numerator >= denominator)
    {
        return 1;
    }
    return FoxRandomBelow(random, denominator) < numerator;
}
