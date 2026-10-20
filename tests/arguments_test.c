#include "app/arguments.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static void Expect(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "arguments test failed: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    uint64_t number = 7u;
    Expect(FoxParseUint64("0", &number) && number == 0u, "zero parses");
    Expect(FoxParseUint64("18446744073709551615", &number) && number == UINT64_MAX,
           "the largest value parses");
    static const char *const badNumbers[] = {
        "", "-1", "+1", " 1", "1 ", "1x", "0x10", "18446744073709551616", "1.0",
    };
    for (size_t index = 0u; index < sizeof(badNumbers) / sizeof(badNumbers[0]); ++index)
    {
        number = 7u;
        Expect(!FoxParseUint64(badNumbers[index], &number) && number == 7u,
               "a malformed number is rejected and leaves the output alone");
    }
    Expect(!FoxParseUint64(NULL, &number) && !FoxParseUint64("1", NULL), "missing pointers fail");

    double scale = 7.0;
    Expect(FoxParseDouble("2.5", &scale) && scale == 2.5, "a decimal parses");
    Expect(FoxParseDouble("1e3", &scale) && scale == 1000.0, "an exponent parses");
    static const char *const badDoubles[] = {"", " 1", "1 ", "abc", "1.0x", "inf", "nan", "1e999"};
    for (size_t index = 0u; index < sizeof(badDoubles) / sizeof(badDoubles[0]); ++index)
    {
        scale = 7.0;
        Expect(!FoxParseDouble(badDoubles[index], &scale) && scale == 7.0,
               "a malformed or infinite value is rejected and leaves the output alone");
    }
    return 0;
}
