#include "app/modules.h"

#include "app/log.h"

#include <stddef.h>

bool FoxModulesOpen(FoxModules *modules)
{
    if (modules == NULL)
    {
        return false;
    }
    modules->host = NULL;
    LaiueModuleHostConfigV1 config;
    LaiueModuleHostConfigInitialize(&config);
    LaiueModuleDiagnostic diagnostic;
    modules->host = LaiueModuleHostCreate(&config, &diagnostic);
    if (modules->host == NULL)
    {
        FoxLog(FOX_LOG_ERROR, "engine module host: %s", diagnostic.message);
        return false;
    }
    return true;
}

void FoxModulesClose(FoxModules *modules)
{
    if (modules == NULL || modules->host == NULL)
    {
        return;
    }
    LaiueModuleHostUnloadAll(modules->host);
    LaiueModuleHostDestroy(modules->host);
    modules->host = NULL;
}

bool FoxModulesLoad(FoxModules *modules, const char *const *names, uint32_t nameCount,
                    const LaiueModuleProviderSelectionV1 *selections, uint32_t selectionCount)
{
    if (modules == NULL || modules->host == NULL || names == NULL || nameCount == 0u ||
        nameCount > FOX_MODULES_MAX_NAMES)
    {
        return false;
    }
    LaiueModuleLoadReportEntryV1 entries[FOX_MODULES_MAX_NAMES];
    LaiueModuleLoadReportV1 report;
    LaiueModuleLoadReportInitialize(&report, entries, nameCount);
    const LaiueModuleNamedProfileV1 profile = {
        .structSize = sizeof(profile),
        .flags = LAIUE_MODULE_PROFILE_ALLOW_PARTIAL,
        .names = names,
        .nameCount = nameCount,
        .providerSelections = selections,
        .providerSelectionCount = selectionCount,
    };
    LaiueModuleDiagnostic diagnostic;
    const LaiueModuleStatus status =
        LaiueModuleHostLoadNamedProfileV1(modules->host, &profile, &report, &diagnostic);
    if (status != LAIUE_MODULE_OK && status != LAIUE_MODULE_PARTIAL)
    {
        FoxLog(FOX_LOG_ERROR, "engine modules: %s", diagnostic.message);
        return false;
    }
    for (uint32_t index = 0u; index < report.count; ++index)
    {
        if ((entries[index].flags & LAIUE_MODULE_PROFILE_ENTRY_LOADED) == 0u)
        {
            FoxLog(FOX_LOG_WARNING, "engine module %s is off: %s", names[index],
                   entries[index].message[0] != '\0' ? entries[index].message : "not found");
        }
    }
    return true;
}

bool FoxModulesLoadStatic(FoxModules *modules, const LaiueModuleApiV1 *const *apis,
                          const char *const *names, uint32_t count)
{
    if (modules == NULL || modules->host == NULL || apis == NULL || names == NULL || count == 0u ||
        count > FOX_MODULES_MAX_NAMES)
    {
        return false;
    }
    LaiueModuleBinaryV1 binaries[FOX_MODULES_MAX_NAMES] = {0};
    for (uint32_t index = 0u; index < count; ++index)
    {
        binaries[index].flags = LAIUE_MODULE_BINARY_STATIC | LAIUE_MODULE_BINARY_OPTIONAL;
        binaries[index].staticApi = apis[index];
    }
    LaiueModuleLoadReportEntryV1 entries[FOX_MODULES_MAX_NAMES];
    LaiueModuleLoadReportV1 report;
    LaiueModuleLoadReportInitialize(&report, entries, count);
    LaiueModuleDiagnostic diagnostic;
    const LaiueModuleStatus status = LaiueModuleHostLoadProfile(
        modules->host, binaries, count, LAIUE_MODULE_PROFILE_ALLOW_PARTIAL, &report, &diagnostic);
    if (status != LAIUE_MODULE_OK && status != LAIUE_MODULE_PARTIAL)
    {
        FoxLog(FOX_LOG_ERROR, "engine modules: %s", diagnostic.message);
        return false;
    }
    for (uint32_t index = 0u; index < report.count; ++index)
    {
        if ((entries[index].flags & LAIUE_MODULE_PROFILE_ENTRY_LOADED) == 0u)
        {
            FoxLog(FOX_LOG_WARNING, "engine module %s is off: %s", names[index],
                   entries[index].message[0] != '\0' ? entries[index].message : "not linked");
        }
    }
    return true;
}

void FoxModulesUnload(FoxModules *modules)
{
    if (modules != NULL && modules->host != NULL)
    {
        LaiueModuleHostUnloadAll(modules->host);
    }
}

const void *FoxModulesQuery(const FoxModules *modules, const char *service, uint32_t minimumVersion,
                            uint32_t minimumSize, uint32_t *outSize)
{
    if (outSize != NULL)
    {
        *outSize = 0u;
    }
    if (modules == NULL || modules->host == NULL || service == NULL)
    {
        return NULL;
    }
    return LaiueModuleHostQueryService(modules->host, service, minimumVersion, minimumSize, NULL,
                                       outSize);
}
