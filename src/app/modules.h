#ifndef FOX_APP_MODULES_H
#define FOX_APP_MODULES_H

#include "mod/module_host.h"

#include <stdbool.h>
#include <stdint.h>

/* Engine technologies are loaded at run time by name and never linked, except
 * on platforms that link the engine statically, which start the linked modules
 * the same way. A missing or broken module switches off only what it
 * provides; the caller learns that from a NULL service and decides how the
 * game continues. */

#define FOX_MODULES_MAX_NAMES 16u

typedef struct FoxModules
{
    LaiueModuleHost *host;
} FoxModules;

bool FoxModulesOpen(FoxModules *modules);
void FoxModulesClose(FoxModules *modules);

/* Loads the named engine modules next to the executable, each one optional.
 * Every module that did not start is logged with the reason. False only when
 * the request itself is invalid or the host refused it; a partial result is
 * success. */
bool FoxModulesLoad(FoxModules *modules, const char *const *names, uint32_t nameCount,
                    const LaiueModuleProviderSelectionV1 *selections, uint32_t selectionCount);

/* Starts engine modules linked into the program, each one optional, where
 * modules cannot be loaded from files: Android, iOS and consoles link the
 * engine statically. `names` label the log. Same results as FoxModulesLoad. */
bool FoxModulesLoadStatic(FoxModules *modules, const LaiueModuleApiV1 *const *apis,
                          const char *const *names, uint32_t count);

/* Stops every loaded module. Services queried earlier become invalid. */
void FoxModulesUnload(FoxModules *modules);

/* The service table, or NULL when no started module publishes it with at
 * least this version and size. outSize receives the published size. */
const void *FoxModulesQuery(const FoxModules *modules, const char *service, uint32_t minimumVersion,
                            uint32_t minimumSize, uint32_t *outSize);

#endif
