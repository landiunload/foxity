#include "sim/space.h"

#include <math.h>

int64_t FoxDivideRounded(int64_t numerator, int64_t denominator)
{
    if (denominator <= 0)
    {
        return 0;
    }
    const int64_t half = denominator / 2;
    return numerator >= 0 ? (numerator + half) / denominator : -((-numerator + half) / denominator);
}

FoxVec FoxVecScale(FoxVec a, int64_t numerator, int64_t denominator)
{
    if (denominator == 0)
    {
        return (FoxVec){0, 0, 0};
    }
    if (denominator < 0)
    {
        numerator = -numerator;
        denominator = -denominator;
    }
    return (FoxVec){
        FoxDivideRounded(a.x * numerator, denominator),
        FoxDivideRounded(a.y * numerator, denominator),
        FoxDivideRounded(a.z * numerator, denominator),
    };
}

FoxVec FoxVecLerp(FoxVec a, FoxVec b, int64_t t)
{
    return FoxVecAdd(a, FoxVecScale(FoxVecSub(b, a), t, FOX_UNIT));
}

uint64_t FoxIntegerSqrt(uint64_t value)
{
    if (value < (UINT64_C(1) << 52))
    {
        /* IEEE square roots are correctly rounded and the input is exact in
         * a double; the corrections make the result exact even if a library
         * were off by an ulp, so every machine agrees. */
        uint64_t root = (uint64_t)sqrt((double)value);
        while (root * root > value)
        {
            --root;
        }
        while ((root + 1u) * (root + 1u) <= value)
        {
            ++root;
        }
        return root;
    }
    /* Bit by bit for the rest of the range. */
    uint64_t root = 0u;
    uint64_t bit = UINT64_C(1) << 62;
    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (value >= root + bit)
        {
            value -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

int64_t FoxVecLength(FoxVec a)
{
    const uint64_t squared = (uint64_t)(a.x * a.x) + (uint64_t)(a.y * a.y) + (uint64_t)(a.z * a.z);
    /* Round to nearest: floor + 1 when (floor + 0.5)^2 < squared. */
    const uint64_t floor = FoxIntegerSqrt(squared);
    return (int64_t)(floor * floor + floor < squared ? floor + 1u : floor);
}

int64_t FoxVecDistance(FoxVec a, FoxVec b)
{
    return FoxVecLength(FoxVecSub(b, a));
}

FoxVec FoxVecResize(FoxVec a, int64_t length)
{
    /* Very long vectors are shortened first so the products below fit. */
    while (FoxAbs(a.x) > (INT64_C(1) << 30) || FoxAbs(a.y) > (INT64_C(1) << 30) ||
           FoxAbs(a.z) > (INT64_C(1) << 30))
    {
        a = (FoxVec){a.x / 2, a.y / 2, a.z / 2};
    }
    const int64_t current = FoxVecLength(a);
    if (current == 0)
    {
        return (FoxVec){0, 0, 0};
    }
    return FoxVecScale(a, length, current);
}
