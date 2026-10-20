#ifndef FOX_SIM_RANDOM_H
#define FOX_SIM_RANDOM_H

#include <stdint.h>

/* Deterministic randomness. Every random decision in the world draws from a
 * stream derived from the world seed and a stable stream number (a car, a
 * district), so the same seed and inputs replay the same history on
 * every machine. Streams are eight bytes: millions of cars can each own one.
 * The generator is SplitMix64. */

typedef struct FoxRandom
{
    uint64_t state;
} FoxRandom;

/* Mixes 64 bits into 64 well-distributed bits; equal inputs give equal
 * outputs everywhere. */
uint64_t FoxHash64(uint64_t value);
uint64_t FoxHashCombine(uint64_t seed, uint64_t value);

/* The stream `stream` of the world `seed`. Different streams of one seed and
 * the same stream of different seeds are independent for practical use. */
FoxRandom FoxRandomStream(uint64_t seed, uint64_t stream);

uint64_t FoxRandomNext(FoxRandom *random);
/* Uniform in [0, bound); zero when bound is zero. Unbiased. */
uint32_t FoxRandomBelow(FoxRandom *random, uint32_t bound);
/* Uniform in [minimum, maximum]; minimum when the range is empty. */
int64_t FoxRandomRange(FoxRandom *random, int64_t minimum, int64_t maximum);
/* True with probability numerator / denominator. */
int FoxRandomChance(FoxRandom *random, uint32_t numerator, uint32_t denominator);

#endif
