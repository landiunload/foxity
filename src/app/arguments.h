#ifndef FOX_APP_ARGUMENTS_H
#define FOX_APP_ARGUMENTS_H

#include <stdbool.h>
#include <stdint.h>

/* Strict command-line numbers: the whole text must be the number, without
 * sign, spaces or a tail. False leaves the output untouched. */
bool FoxParseUint64(const char *text, uint64_t *outValue);
/* A finite decimal number, sign and exponent allowed. */
bool FoxParseDouble(const char *text, double *outValue);

#endif
