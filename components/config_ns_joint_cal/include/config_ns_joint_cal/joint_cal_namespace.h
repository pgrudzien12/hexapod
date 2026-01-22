#pragma once

#include "esp_err.h"
#include "nvs.h"
#include "config_ns_joint_cal/joint_cal_config.h"
#include "config_core/namespace.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize defaults into NVS for joint calibration namespace
esp_err_t joint_cal_ns_init_defaults_nvs(nvs_handle_t handle);

// Load joint calibration namespace from NVS into provided struct
esp_err_t joint_cal_ns_load_from_nvs(nvs_handle_t handle, joint_calib_config_t *cfg);

// Save joint calibration namespace to NVS
esp_err_t joint_cal_ns_save_to_nvs(nvs_handle_t handle, const joint_calib_config_t *cfg);

// Descriptor accessor & registration helper
const config_namespace_descriptor_t *joint_cal_ns_get_descriptor(void);
esp_err_t joint_cal_ns_register_with_core(void);

#ifdef __cplusplus
}
#endif
