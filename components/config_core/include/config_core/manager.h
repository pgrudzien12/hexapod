#pragma once

#include "esp_err.h"
#include "namespace.h"

#ifdef __cplusplus
extern "C" {
#endif

// Core registry and coordination for configuration namespaces.
// Phase 4: add minimal runtime registry and lookup helpers.

esp_err_t config_core_init_wrap(void); // thin wrapper to call existing config_manager_init()

// Unified helpers using the registry (namespaces resolved by name).
esp_err_t config_core_save_namespace(const char *name);
esp_err_t config_core_register_namespace(const config_namespace_descriptor_t *desc);
esp_err_t config_core_get_namespace(const char *name, const config_namespace_descriptor_t **out_desc);

#ifdef __cplusplus
}
#endif
