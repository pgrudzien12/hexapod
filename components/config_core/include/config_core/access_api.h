#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "config_core/types.h"

// Forward declaration only; actual definition resides in legacy code until fully migrated.
typedef struct config_param_info_t config_param_info_t;

#ifdef __cplusplus
extern "C" {
#endif

// Unified accessors (descriptor-based later; currently stubs in access_api.c)
esp_err_t config_core_get_bool(const char *ns, const char *param, bool *value);
esp_err_t config_core_set_bool(const char *ns, const char *param, bool value, bool persist);
esp_err_t config_core_get_int32(const char *ns, const char *param, int32_t *value);
esp_err_t config_core_set_int32(const char *ns, const char *param, int32_t value, bool persist);
esp_err_t config_core_get_uint32(const char *ns, const char *param, uint32_t *value);
esp_err_t config_core_set_uint32(const char *ns, const char *param, uint32_t value, bool persist);
esp_err_t config_core_get_float(const char *ns, const char *param, float *value);
esp_err_t config_core_set_float(const char *ns, const char *param, float value, bool persist);
esp_err_t config_core_get_string(const char *ns, const char *param, char *buf, size_t max_len);
esp_err_t config_core_set_string(const char *ns, const char *param, const char *value, bool persist);

// Discovery API (will be descriptor-backed later)
esp_err_t config_core_list_namespaces(const char **names, size_t *count);
esp_err_t config_core_list_parameters(const char *ns, const char **param_names, size_t max_params, size_t *count);
// Core uses a lightweight metadata type to avoid legacy typedef conflicts.
esp_err_t config_core_get_parameter_info(const char *ns, const char *param, config_param_info_core_t *info);

#ifdef __cplusplus
}
#endif
