#include "config_ns_system/system_namespace.h"
#include "config_core/manager.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "sys_ns";

// NVS parameter keys (moved from legacy file)
#define SYS_KEY_EMERGENCY_STOP      "emerg_stop"
#define SYS_KEY_AUTO_DISARM_TIMEOUT "auto_disarm"
#define SYS_KEY_SAFETY_VOLTAGE_MIN  "volt_min"
#define SYS_KEY_TEMP_LIMIT_MAX      "temp_max"
#define SYS_KEY_MOTION_TIMEOUT      "motion_timeout"
#define SYS_KEY_STARTUP_DELAY       "startup_delay"
#define SYS_KEY_MAX_CONTROL_FREQ    "max_ctrl_freq"
#define SYS_KEY_ROBOT_ID            "robot_id"
#define SYS_KEY_ROBOT_NAME          "robot_name"
#define SYS_KEY_CONFIG_VERSION      "config_ver"
#define SYS_KEY_CONTROLLER_TYPE     "ctrl_type"

// Schema version for system namespace (mirrors global version for now)
#define SYSTEM_SCHEMA_VERSION 1

// Internal helper (optional) to write float as blob
static inline esp_err_t nvs_set_float_blob(nvs_handle_t h, const char *key, const float *val) {
    return nvs_set_blob(h, key, val, sizeof(float));
}

esp_err_t system_ns_init_defaults_nvs(nvs_handle_t handle) {
    ESP_LOGI(TAG, "Initializing system namespace defaults to NVS (component)");

    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_EMERGENCY_STOP, 1));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, 30));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MOTION_TIMEOUT, 1000));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_STARTUP_DELAY, 2000));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, 100));

    float default_voltage = 6.5f;
    float default_temp = 80.0f;
    ESP_ERROR_CHECK(nvs_set_float_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &default_voltage));
    ESP_ERROR_CHECK(nvs_set_float_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &default_temp));

    char robot_id[32];
    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err == ESP_OK) {
        snprintf(robot_id, sizeof(robot_id), "HEXAPOD_%02X%02X%02X", mac[3], mac[4], mac[5]);
    } else {
        strcpy(robot_id, "HEXAPOD_DEFAULT");
    }
    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_ID, robot_id));
    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_NAME, "My Hexapod Robot"));

    ESP_ERROR_CHECK(nvs_set_u16(handle, SYS_KEY_CONFIG_VERSION, SYSTEM_SCHEMA_VERSION));

    // Default controller type (matching legacy default)
    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_CONTROLLER_TYPE, (uint8_t)CONTROLLER_DRIVER_FLYSKY_IBUS));

    ESP_ERROR_CHECK(nvs_commit(handle));
    ESP_LOGI(TAG, "System defaults written to NVS (component)");
    return ESP_OK;
}

esp_err_t system_ns_load_from_nvs(nvs_handle_t handle, system_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    size_t required_size = 0;

    ESP_LOGI(TAG, "Loading system config from NVS (component)");

    uint8_t temp_bool = 0;
    err = nvs_get_u8(handle, SYS_KEY_EMERGENCY_STOP, &temp_bool);
    cfg->emergency_stop_enabled = (err == ESP_OK) ? (temp_bool != 0) : cfg->emergency_stop_enabled;

    nvs_get_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, &cfg->auto_disarm_timeout);
    nvs_get_u32(handle, SYS_KEY_MOTION_TIMEOUT, &cfg->motion_timeout_ms);
    nvs_get_u32(handle, SYS_KEY_STARTUP_DELAY, &cfg->startup_delay_ms);
    nvs_get_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, &cfg->max_control_frequency);

    required_size = sizeof(float);
    nvs_get_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &cfg->safety_voltage_min, &required_size);
    required_size = sizeof(float);
    nvs_get_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &cfg->temperature_limit_max, &required_size);

    required_size = sizeof(cfg->robot_id);
    nvs_get_str(handle, SYS_KEY_ROBOT_ID, cfg->robot_id, &required_size);
    required_size = sizeof(cfg->robot_name);
    nvs_get_str(handle, SYS_KEY_ROBOT_NAME, cfg->robot_name, &required_size);

    uint16_t ns_ver = 0;
    if (nvs_get_u16(handle, SYS_KEY_CONFIG_VERSION, &ns_ver) == ESP_OK) {
        cfg->config_version = ns_ver;
    }

    uint8_t ctrl_type = 0;
    if (nvs_get_u8(handle, SYS_KEY_CONTROLLER_TYPE, &ctrl_type) == ESP_OK) {
        cfg->controller_type = (controller_driver_type_e)ctrl_type;
    }

    return ESP_OK;
}

esp_err_t system_ns_save_to_nvs(nvs_handle_t handle, const system_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Saving system config to NVS (component)");

    uint8_t temp_bool = cfg->emergency_stop_enabled ? 1 : 0;
    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_EMERGENCY_STOP, temp_bool));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, cfg->auto_disarm_timeout));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MOTION_TIMEOUT, cfg->motion_timeout_ms));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_STARTUP_DELAY, cfg->startup_delay_ms));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, cfg->max_control_frequency));

    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &cfg->safety_voltage_min, sizeof(float)));
    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &cfg->temperature_limit_max, sizeof(float)));

    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_ID, cfg->robot_id));
    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_NAME, cfg->robot_name));

    ESP_ERROR_CHECK(nvs_set_u16(handle, SYS_KEY_CONFIG_VERSION, cfg->config_version));
    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_CONTROLLER_TYPE, (uint8_t)cfg->controller_type));

    esp_err_t err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Commit failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

// Descriptor instance
static esp_err_t sys_load_defaults_mem(void *cache)
{
    if (!cache) return ESP_ERR_INVALID_ARG;
    system_config_t *c = (system_config_t *)cache;
    c->emergency_stop_enabled = true;
    c->auto_disarm_timeout = 30;
    c->motion_timeout_ms = 1000;
    c->startup_delay_ms = 2000;
    c->max_control_frequency = 100;
    c->safety_voltage_min = 6.5f;
    c->temperature_limit_max = 80.0f;
    strncpy(c->robot_id, "HEXAPOD_DEFAULT", sizeof(c->robot_id));
    strncpy(c->robot_name, "My Hexapod Robot", sizeof(c->robot_name));
    c->config_version = SYSTEM_SCHEMA_VERSION;
    c->controller_type = CONTROLLER_DRIVER_FLYSKY_IBUS;
    return ESP_OK;
}

static const config_namespace_descriptor_t SYSTEM_NAMESPACE_DESCRIPTOR = {
    .name = "system",
    .schema_version = SYSTEM_SCHEMA_VERSION,
    .runtime_cache_ptr = NULL,
    .runtime_cache_size = sizeof(system_config_t),
    .load_defaults_mem = sys_load_defaults_mem,
    .init_defaults_nvs = system_ns_init_defaults_nvs,
    .load_from_nvs = (esp_err_t (*)(nvs_handle_t, void*))system_ns_load_from_nvs,
    .save_to_nvs = (esp_err_t (*)(nvs_handle_t, const void*))system_ns_save_to_nvs,
    .migrate_step = NULL,
    .list_param_names = NULL,
    .find_param = NULL,
    .get_value = NULL,
    .set_value = NULL,
    .validate_value = NULL,
};

const config_namespace_descriptor_t *system_ns_get_descriptor(void)
{
    return &SYSTEM_NAMESPACE_DESCRIPTOR;
}

esp_err_t system_ns_register_with_core(void)
{
    const config_namespace_descriptor_t *desc = system_ns_get_descriptor();
    return config_core_register_namespace(desc);
}
