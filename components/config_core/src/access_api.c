#include "config_core/access_api.h"
#include "config_core/namespace.h"
#include "esp_err.h"
#include <string.h>

// Phase 2 stubs: component layer is decoupled from legacy implementation.
// These will be wired to descriptor logic in later phases.

static inline esp_err_t resolve(const char *ns, const config_namespace_descriptor_t **out)
{
    return config_core_get_namespace(ns, out);
}

esp_err_t config_core_get_bool(const char *ns, const char *param, bool *value) {
    if (!ns || !param || !value) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->get_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    err = d->get_value(param, &u);
    if (err != ESP_OK) return err;
    *value = u.b;
    return ESP_OK;
}
esp_err_t config_core_set_bool(const char *ns, const char *param, bool value, bool persist) {
    if (!ns || !param) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->set_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    u.b = value;
    return d->set_value(param, &u, persist);
}
esp_err_t config_core_get_int32(const char *ns, const char *param, int32_t *value) {
    if (!ns || !param || !value) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->get_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    err = d->get_value(param, &u);
    if (err != ESP_OK) return err;
    *value = u.i32;
    return ESP_OK;
}
esp_err_t config_core_set_int32(const char *ns, const char *param, int32_t value, bool persist) {
    if (!ns || !param) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->set_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    u.i32 = value;
    return d->set_value(param, &u, persist);
}
esp_err_t config_core_get_uint32(const char *ns, const char *param, uint32_t *value) {
    if (!ns || !param || !value) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->get_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    err = d->get_value(param, &u);
    if (err != ESP_OK) return err;
    *value = u.u32;
    return ESP_OK;
}
esp_err_t config_core_set_uint32(const char *ns, const char *param, uint32_t value, bool persist) {
    if (!ns || !param) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->set_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    u.u32 = value;
    return d->set_value(param, &u, persist);
}
esp_err_t config_core_get_float(const char *ns, const char *param, float *value) {
    if (!ns || !param || !value) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->get_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    err = d->get_value(param, &u);
    if (err != ESP_OK) return err;
    *value = u.f;
    return ESP_OK;
}
esp_err_t config_core_set_float(const char *ns, const char *param, float value, bool persist) {
    if (!ns || !param) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->set_value) return ESP_ERR_NOT_SUPPORTED;
    config_value_union_t u = {0};
    u.f = value;
    return d->set_value(param, &u, persist);
}
esp_err_t config_core_get_string(const char *ns, const char *param, char *buf, size_t max_len) {
    (void)ns; (void)param; (void)buf; (void)max_len;
    // String routing will be implemented when descriptors expose string getters
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t config_core_set_string(const char *ns, const char *param, const char *value, bool persist) {
    (void)ns; (void)param; (void)value; (void)persist;
    // String routing will be implemented when descriptors expose string setters
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t config_core_list_namespaces(const char **names, size_t *count) {
    // Minimal: not implemented until we add iteration API to registry
    (void)names; (void)count; return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t config_core_list_parameters(const char *ns, const char **param_names, size_t max_params, size_t *count) {
    if (!ns || !param_names || !count) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->list_param_names) return ESP_ERR_NOT_SUPPORTED;
    return d->list_param_names(0, max_params, param_names, count);
}
esp_err_t config_core_get_parameter_info(const char *ns, const char *param, config_param_info_core_t *info) {
    if (!ns || !param || !info) return ESP_ERR_INVALID_ARG;
    const config_namespace_descriptor_t *d = NULL;
    esp_err_t err = resolve(ns, &d);
    if (err != ESP_OK || !d || !d->find_param) return ESP_ERR_NOT_SUPPORTED;
    return d->find_param(param, info);
}
