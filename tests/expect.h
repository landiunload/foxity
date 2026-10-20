#ifndef FOX_TESTS_EXPECT_H
#define FOX_TESTS_EXPECT_H

#include <stdio.h>
#include <stdlib.h>

/* Stops the test with a message when a condition does not hold. */
#define EXPECT(condition, message)                                                                 \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, (message));                         \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)

#endif
