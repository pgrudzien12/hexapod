#include "config_ns_joint_cal/joint_cal_namespace.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "joint_ns";

// NVS keys (per parameter and per joint will be formatted dynamically)
#define JOINT_KEY_PREFIX "joint" // keys will be joint<leg>_<joint>_<field>
#define JOINT_NS_VERSION_KEY "config_ver"

// Simple defaults loader (memory only)
static esp_err_t joint_cal_load_defaults_mem(void *cache)
{
    if (!cache) return ESP_ERR_INVALID_ARG;
    joint_calib_config_t *c = (joint_calib_config_t*)cache;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        for (int j = 0; j < NUM_JOINTS_PER_LEG; ++j) {
            joint_calib_t *jc = &c->joints[leg][j];
            jc->zero_offset_rad = 0.0f;
            jc->invert_sign = 1;
            jc->min_rad = -1.57f;
            jc->max_rad = 1.57f;
            jc->pwm_min_us = 1000;
            jc->pwm_max_us = 2000;
            jc->neutral_us = 1500;
        }
    }
    c->config_version = JOINT_CAL_SCHEMA_VERSION;
    return ESP_OK;
}

// Write defaults to NVS (coarse-grained; store entire struct as blob for now)
esp_err_t joint_cal_ns_init_defaults_nvs(nvs_handle_t handle)
{
    joint_calib_config_t temp;
    joint_cal_load_defaults_mem(&temp);
    ESP_ERROR_CHECK(nvs_set_blob(handle, "joint_calib", &temp, sizeof(temp)));
    ESP_ERROR_CHECK(nvs_set_u16(handle, JOINT_NS_VERSION_KEY, JOINT_CAL_SCHEMA_VERSION));
    ESP_ERROR_CHECK(nvs_commit(handle));
    ESP_LOGI(TAG, "Joint calibration defaults written to NVS");
    return ESP_OK;
}

esp_err_t joint_cal_ns_load_from_nvs(nvs_handle_t handle, joint_calib_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    size_t sz = sizeof(*cfg);
    esp_err_t err = nvs_get_blob(handle, "joint_calib", cfg, &sz);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Retry by initializing defaults
        ESP_LOGW(TAG, "Joint calib blob missing, initializing defaults in NVS");
        ESP_ERROR_CHECK(joint_cal_ns_init_defaults_nvs(handle));
        sz = sizeof(*cfg);
        err = nvs_get_blob(handle, "joint_calib", cfg, &sz);
    }
    if (err != ESP_OK) return err;
    uint16_t ver = 0;
    if (nvs_get_u16(handle, JOINT_NS_VERSION_KEY, &ver) == ESP_OK) {
        cfg->config_version = ver;
    }
    return ESP_OK;
}

esp_err_t joint_cal_ns_save_to_nvs(nvs_handle_t handle, const joint_calib_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    ESP_ERROR_CHECK(nvs_set_blob(handle, "joint_calib", cfg, sizeof(*cfg)));
    ESP_ERROR_CHECK(nvs_set_u16(handle, JOINT_NS_VERSION_KEY, cfg->config_version));
    return nvs_commit(handle);
}

// Descriptor (dynamic param hooks deferred)
static const config_namespace_descriptor_t JOINT_CAL_NAMESPACE_DESCRIPTOR = {
    .name = "joint_cal",
    .schema_version = JOINT_CAL_SCHEMA_VERSION,
    .runtime_cache_ptr = NULL,
    .runtime_cache_size = sizeof(joint_calib_config_t),
    .load_defaults_mem = joint_cal_load_defaults_mem,
    .init_defaults_nvs = joint_cal_ns_init_defaults_nvs,
    .load_from_nvs = (esp_err_t (*)(nvs_handle_t, void*))joint_cal_ns_load_from_nvs,
    .save_to_nvs = (esp_err_t (*)(nvs_handle_t, const void*))joint_cal_ns_save_to_nvs,
    .migrate_step = NULL,
    .list_param_names = NULL,
    .find_param = NULL,
    .get_value = NULL,
    .set_value = NULL,
    .validate_value = NULL,
};

const config_namespace_descriptor_t *joint_cal_ns_get_descriptor(void)
{
    return &JOINT_CAL_NAMESPACE_DESCRIPTOR;
}

esp_err_t joint_cal_ns_register_with_core(void)
{
    return config_core_register_namespace(joint_cal_ns_get_descriptor());
}
