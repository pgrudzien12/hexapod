#pragma once

#include "esp_err.h"
#include "nvs.h"
#include "config_ns_system/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// NVS initialization of defaults (writes defaults into NVS keys)
esp_err_t system_ns_init_defaults_nvs(nvs_handle_t handle);

// Load system namespace from NVS into provided config struct (read-only operation)
esp_err_t system_ns_load_from_nvs(nvs_handle_t handle, system_config_t *cfg);

// Save provided system config to NVS (persistent write)
esp_err_t system_ns_save_to_nvs(nvs_handle_t handle, const system_config_t *cfg);

// Descriptor exposure and optional registration helper
#include "config_core/namespace.h"
const config_namespace_descriptor_t *system_ns_get_descriptor(void);
esp_err_t system_ns_register_with_core(void);

#ifdef __cplusplus
}
#endif
