#include "expect.h"
#include "sim/clock.h"
#include "sim/random.h"
#include "sim/space.h"

#include <stdint.h>

static void TestRoots(void)
{
    EXPECT(FoxIntegerSqrt(0u) == 0u && FoxIntegerSqrt(15u) == 3u && FoxIntegerSqrt(16u) == 4u,
           "small roots are exact");
    EXPECT(FoxIntegerSqrt(UINT64_MAX) == UINT64_C(4294967295), "the largest root is exact");
    const uint64_t big = (UINT64_C(1) << 52) - 1u;
    const uint64_t root = FoxIntegerSqrt(big);
    EXPECT(root * root <= big && (root + 1u) * (root + 1u) > big, "roots at the double limit");
    for (uint64_t value = UINT64_C(1) << 40; value < (UINT64_C(1) << 40) + 5000u; ++value)
    {
        const uint64_t r = FoxIntegerSqrt(value);
        EXPECT(r * r <= value && (r + 1u) * (r + 1u) > value, "every root is the floor");
    }
}

static void TestVectors(void)
{
    const FoxVec a = {3, 4, 0};
    EXPECT(FoxVecLength(a) == 5, "a 3-4-5 triangle");
    const FoxVec unit = FoxVecDirection((FoxVec){0, 0, -77});
    EXPECT(unit.x == 0 && unit.y == 0 && unit.z == -FOX_UNIT, "directions keep their sign");
    EXPECT(FoxVecIsZero(FoxVecDirection((FoxVec){0, 0, 0})), "a zero vector has no direction");
    const FoxVec x = {FOX_UNIT, 0, 0};
    const FoxVec y = {0, FOX_UNIT, 0};
    const FoxVec z = FoxVecCross(x, y);
    EXPECT(z.x == 0 && z.y == 0 && z.z == (int64_t)FOX_UNIT * FOX_UNIT, "x cross y is z");
    EXPECT(FoxDivideRounded(5, 2) == 3 && FoxDivideRounded(-5, 2) == -3 &&
               FoxDivideRounded(4, 3) == 1,
           "rounded division is symmetric");
    EXPECT(FoxFloorDivide(-1, 4) == -1 && FoxFloorDivide(-4, 4) == -1 && FoxFloorDivide(7, 4) == 1,
           "floor division rounds down");
    const FoxVec middle = FoxVecLerp((FoxVec){0, 0, 0}, (FoxVec){100, -100, 7}, FOX_UNIT / 2);
    EXPECT(middle.x == 50 && middle.y == -50 && middle.z == 4, "lerp rounds to nearest");
}

static void TestRandomAndClock(void)
{
    FoxRandom random = {.state = 0u};
    EXPECT(FoxRandomNext(&random) == UINT64_C(0xE220A8397B1DCDAF), "SplitMix64 reference");
    FoxRandom first = FoxRandomStream(42u, 1u);
    FoxRandom again = FoxRandomStream(42u, 1u);
    EXPECT(FoxRandomNext(&first) == FoxRandomNext(&again), "streams replay");
    for (int draw = 0; draw < 1000; ++draw)
    {
        EXPECT(FoxRandomBelow(&first, 7u) < 7u, "below stays below");
    }

    FoxClock clock;
    EXPECT(FoxClockInit(&clock, FOX_CLOCK_TICKS_PER_SECOND, 0.25), "the clock starts");
    EXPECT(FoxClockAdvance(&clock, 0.1) == 3u && FoxClockAdvance(&clock, 1.0) == 7u,
           "long gaps are capped");
    EXPECT(FoxClockSetTimeScale(&clock, 0.0) && FoxClockAdvance(&clock, 0.2) == 0u,
           "a paused clock does not tick");
    EXPECT(!FoxClockSetTimeScale(&clock, -1.0), "negative scales are refused");
}

int main(void)
{
    TestRoots();
    TestVectors();
    TestRandomAndClock();
    return 0;
}
