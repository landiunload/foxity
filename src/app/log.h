#ifndef FOX_APP_LOG_H
#define FOX_APP_LOG_H

typedef enum FoxLogLevel
{
    FOX_LOG_INFO = 0,
    FOX_LOG_WARNING,
    FOX_LOG_ERROR,
} FoxLogLevel;

/* One line to standard error, or to logcat on Android, prefixed with the game
 * name and the level. The format follows printf; the line break is added
 * here. */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
void FoxLog(FoxLogLevel level, const char *format, ...);

#endif
