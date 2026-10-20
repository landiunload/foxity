#include "app/arguments.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

bool FoxParseUint64(const char *text, uint64_t *outValue)
{
    if (text == NULL || outValue == NULL || text[0] == '\0')
    {
        return false;
    }
    uint64_t value = 0u;
    for (const char *cursor = text; *cursor != '\0'; ++cursor)
    {
        if (*cursor < '0' || *cursor > '9')
        {
            return false;
        }
        const uint64_t digit = (uint64_t)(*cursor - '0');
        if (value > (UINT64_MAX - digit) / 10u)
        {
            return false;
        }
        value = value * 10u + digit;
    }
    *outValue = value;
    return true;
}

bool FoxParseDouble(const char *text, double *outValue)
{
    if (text == NULL || outValue == NULL || text[0] == '\0' || text[0] == ' ' || text[0] == '\t')
    {
        return false;
    }
    char *end = NULL;
    const double value = strtod(text, &end);
    if (end == NULL || *end != '\0' || !isfinite(value))
    {
        return false;
    }
    *outValue = value;
    return true;
}
