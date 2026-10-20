#include "app/log.h"

#include <stdarg.h>
#include <stdio.h>

#if defined(__ANDROID__)
#include <android/log.h>
#endif

void FoxLog(FoxLogLevel level, const char *format, ...)
{
#if defined(__ANDROID__)
    /* An application process has no console: lines go to logcat. */
    static const int priorities[] = {ANDROID_LOG_INFO, ANDROID_LOG_WARN, ANDROID_LOG_ERROR};
    const int priority = (unsigned)level < sizeof(priorities) / sizeof(priorities[0])
                             ? priorities[level]
                             : ANDROID_LOG_ERROR;
    va_list arguments;
    va_start(arguments, format);
    (void)__android_log_vprint(priority, "Foxity", format, arguments);
    va_end(arguments);
#else
    static const char *const levels[] = {"info", "warning", "error"};
    const char *label = (unsigned)level < sizeof(levels) / sizeof(levels[0]) ? levels[level] : "?";
    fprintf(stderr, "foxity %s: ", label);
    va_list arguments;
    va_start(arguments, format);
    vfprintf(stderr, format, arguments);
    va_end(arguments);
    fputc('\n', stderr);
    fflush(stderr);
#endif
}
