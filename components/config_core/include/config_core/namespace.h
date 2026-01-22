#pragma once

#include "types.h"
#include "nvs.h"
#include "esp_err.h"

// Dynamic, per-namespace descriptor. Phase 1: not yet used operationally.
// Internal static tables remain inside legacy config_manager.c for now.

typedef struct config_namespace_descriptor_t {
    const char *name;                     // Namespace string identifier
    uint16_t schema_version;              // Per-namespace schema version
    void *runtime_cache_ptr;              // Optional pointer to in-RAM struct
    size_t runtime_cache_size;            // Size of runtime cache

    // Lifecycle & persistence hooks (may be NULL if not applicable in Phase 1)
    esp_err_t (*load_defaults_mem)(void *cache);
    esp_err_t (*init_defaults_nvs)(nvs_handle_t h);
    esp_err_t (*load_from_nvs)(nvs_handle_t h, void *cache);
    esp_err_t (*save_to_nvs)(nvs_handle_t h, const void *cache);
    esp_err_t (*migrate_step)(nvs_handle_t h, uint16_t from_ver, uint16_t to_ver); // optional

    // Dynamic parameter interface (Phase 1: can be NULL; legacy APIs still in use)
    esp_err_t (*list_param_names)(size_t offset, size_t max, const char **names_out, size_t *returned);
    esp_err_t (*find_param)(const char *name, config_param_info_core_t *out_meta);
    esp_err_t (*get_value)(const char *name, config_value_union_t *out_val);
    esp_err_t (*set_value)(const char *name, const config_value_union_t *in_val, bool persist);
    esp_err_t (*validate_value)(const char *name, const config_value_union_t *candidate);
} config_namespace_descriptor_t;

// Registration API provided by config_core manager.
#ifdef __cplusplus
extern "C" {
#endif

// Register a descriptor at runtime.
esp_err_t config_core_register_namespace(const config_namespace_descriptor_t *desc);

// Retrieve descriptor by name.
esp_err_t config_core_get_namespace(const char *name, const config_namespace_descriptor_t **out_desc);

#ifdef __cplusplus
}
#endif
