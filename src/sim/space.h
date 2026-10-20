#ifndef FOX_SIM_SPACE_H
#define FOX_SIM_SPACE_H

#include <stdbool.h>
#include <stdint.h>

/* World space. A position is three signed 64-bit integers in units of 1/1024
 * metre. Space is unbounded for every practical purpose; the planet sits at
 * the origin, so "up" anywhere on it is the direction away from the origin.
 * Integer arithmetic gives every machine the same city; presentation converts
 * to floats only relative to a nearby origin.
 *
 * Directions are vectors of length FOX_UNIT (2^16): a tangent, a normal, the
 * up of a place. */

#define FOX_UNITS_PER_METRE 1024
#define FOX_METRES(metres) ((int64_t)(metres) * FOX_UNITS_PER_METRE)
#define FOX_UNIT INT64_C(65536)

typedef struct FoxVec
{
    int64_t x;
    int64_t y;
    int64_t z;
} FoxVec;

static inline FoxVec FoxVecAdd(FoxVec a, FoxVec b)
{
    return (FoxVec){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline FoxVec FoxVecSub(FoxVec a, FoxVec b)
{
    return (FoxVec){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline FoxVec FoxVecNegate(FoxVec a)
{
    return (FoxVec){-a.x, -a.y, -a.z};
}

static inline bool FoxVecEqual(FoxVec a, FoxVec b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

static inline bool FoxVecIsZero(FoxVec a)
{
    return a.x == 0 && a.y == 0 && a.z == 0;
}

/* The components must stay below 2^31 in magnitude, which every distance and
 * direction in the city does by far. */
static inline int64_t FoxVecDot(FoxVec a, FoxVec b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline FoxVec FoxVecCross(FoxVec a, FoxVec b)
{
    return (FoxVec){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/* a * numerator / denominator per component, rounded to nearest; the
 * products must fit in 64 bits. A zero denominator gives zero. */
FoxVec FoxVecScale(FoxVec a, int64_t numerator, int64_t denominator);

/* a + (b - a) * t / FOX_UNIT, rounded to nearest. */
FoxVec FoxVecLerp(FoxVec a, FoxVec b, int64_t t);

/* floor(sqrt(value)), exact for every 64-bit input. */
uint64_t FoxIntegerSqrt(uint64_t value);

/* Euclidean length, for components below 2^31. */
int64_t FoxVecLength(FoxVec a);
int64_t FoxVecDistance(FoxVec a, FoxVec b);

/* The vector pointing like `a` with the given length, rounded per component;
 * zero for a zero vector. Components of `a` must stay below 2^31. */
FoxVec FoxVecResize(FoxVec a, int64_t length);

/* A direction of length FOX_UNIT; zero for a zero vector. */
static inline FoxVec FoxVecDirection(FoxVec a)
{
    return FoxVecResize(a, FOX_UNIT);
}

/* round(numerator / denominator) for a positive denominator, halves away from
 * zero. */
int64_t FoxDivideRounded(int64_t numerator, int64_t denominator);

/* floor(value / divisor) for a positive divisor. */
static inline int64_t FoxFloorDivide(int64_t value, int64_t divisor)
{
    const int64_t quotient = value / divisor;
    return (value % divisor != 0 && value < 0) ? quotient - 1 : quotient;
}

static inline int64_t FoxClamp(int64_t value, int64_t minimum, int64_t maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

static inline int64_t FoxMin(int64_t a, int64_t b)
{
    return a < b ? a : b;
}

static inline int64_t FoxMax(int64_t a, int64_t b)
{
    return a > b ? a : b;
}

static inline int64_t FoxAbs(int64_t a)
{
    return a < 0 ? -a : a;
}

#endif
