#include "config_core/manager.h"
#include <string.h>

#define CONFIG_CORE_MAX_NAMESPACES 8

static const config_namespace_descriptor_t *s_registry[CONFIG_CORE_MAX_NAMESPACES];
static size_t s_registry_count = 0;

esp_err_t config_core_register_namespace(const config_namespace_descriptor_t *desc)
{
    if (!desc || !desc->name) {
        return ESP_ERR_INVALID_ARG;
    }
    // Prevent duplicates by name
    for (size_t i = 0; i < s_registry_count; ++i) {
        if (strcmp(s_registry[i]->name, desc->name) == 0) {
            s_registry[i] = desc; // allow update/override
            return ESP_OK;
        }
    }
    if (s_registry_count >= CONFIG_CORE_MAX_NAMESPACES) {
        return ESP_ERR_NO_MEM;
    }
    s_registry[s_registry_count++] = desc;
    return ESP_OK;
}

esp_err_t config_core_get_namespace(const char *name, const config_namespace_descriptor_t **out_desc)
{
    if (!name || !out_desc) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < s_registry_count; ++i) {
        if (strcmp(s_registry[i]->name, name) == 0) {
            *out_desc = s_registry[i];
            return ESP_OK;
        }
    }
    *out_desc = NULL;
    return ESP_ERR_NOT_FOUND;
}

// Thin wrappers retained for backward compatibility
esp_err_t config_core_init_wrap(void)
{
    extern esp_err_t config_manager_init(void);
    return config_manager_init();
}

esp_err_t config_core_save_namespace(const char *name)
{
    // Placeholder: future implementation will locate descriptor and invoke save_to_nvs
    (void)name;
    return ESP_ERR_NOT_SUPPORTED;
}