/*
 * Configuration Manager
 * 
 * NVS-based configuration persistence with dual-method API for live tuning.
 * Supports memory-only updates for immediate robot response and persistent
 * updates for permanent storage.
 * 
 * License: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "nvs_flash.h"
#include "controller.h"
#include "types/joint_types.h"
#include "robot_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration Namespaces
// =============================================================================

typedef enum {
    CONFIG_NS_SYSTEM = 0,        // System-wide settings and safety
    CONFIG_NS_JOINT_CALIB = 1,   // Joint calibration per leg/joint
    CONFIG_NS_COUNT              // Keep this last
} config_namespace_t;

// Namespace string mappings (must match enum order)
extern const char* CONFIG_NAMESPACE_NAMES[CONFIG_NS_COUNT];

// =============================================================================
// Joint Calibration Configuration Structure
// =============================================================================



// Complete joint calibration configuration for all legs and joints
typedef struct {
    // Joint calibration data indexed by [leg][joint]
    // leg: 0-5 (LEG_LEFT_FRONT to LEG_RIGHT_REAR)
    // joint: 0-2 (LEG_SERVO_COXA, LEG_SERVO_FEMUR, LEG_SERVO_TIBIA)
    joint_calib_t joints[NUM_LEGS][NUM_JOINTS_PER_LEG];
} joint_calib_config_t;

// =============================================================================
// System Configuration Structure (moved to component header)
// =============================================================================
#include "config_ns_system/system_config.h"

// =============================================================================
// Configuration Manager State
// =============================================================================

typedef struct {
    bool namespace_dirty[CONFIG_NS_COUNT];   // Which namespaces have unsaved changes
    bool namespace_loaded[CONFIG_NS_COUNT];  // Which namespaces are in memory cache
    bool initialized;                        // Manager initialization state
    nvs_handle_t nvs_handles[CONFIG_NS_COUNT]; // NVS handles per namespace
} config_manager_state_t;

// =============================================================================
// Core Configuration Manager API
// =============================================================================

/**
 * @brief Initialize configuration manager
 * 
 * Opens NVS handles for all namespaces and loads system configuration.
 * Must be called before any other config operations.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_manager_init(void);

/**
 * @brief Get current configuration manager state
 * 
 * @param[out] state Pointer to state structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if state is NULL
 */
esp_err_t config_manager_get_state(config_manager_state_t *state);

/**
 * @brief Check if any namespace has unsaved changes
 * 
 * @return true if there are unsaved changes, false otherwise
 */
bool config_manager_has_dirty_data(void);

/**
 * @brief Save specific namespace to NVS
 * 
 * @param ns Namespace to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_manager_save_namespace(config_namespace_t ns);

// =============================================================================
// System Configuration API
// =============================================================================

/**
 * @brief Get system configuration (from memory cache)
 * 
 * @return Pointer to system configuration structure (read-only)
 */
const system_config_t* config_get_system(void);

/**
 * @brief Set complete system configuration structure (always persistent)
 * 
 * @param config Pointer to system configuration structure
 * @return ESP_OK on success, error code on NVS write failure
 */
esp_err_t config_set_system(const system_config_t* config);

// =============================================================================
// Joint Calibration Configuration API
// =============================================================================

/**
 * @brief Get joint calibration configuration (from memory cache)
 * 
 * @return Pointer to joint calibration configuration structure (read-only)
 */
const joint_calib_config_t* config_get_joint_calib(void);

/**
 * @brief Set complete joint calibration configuration structure (always persistent)
 * 
 * @param config Pointer to joint calibration configuration structure
 * @return ESP_OK on success, error code on NVS write failure
 */
esp_err_t config_set_joint_calib(const joint_calib_config_t* config);

/**
 * @brief Get calibration data for a specific joint (memory-only)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param[out] calib_data Pointer to store calibration data
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_get_joint_calib_data(int leg_index, int joint, joint_calib_t* calib_data);

/**
 * @brief Set calibration data for a specific joint (memory-only)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param calib_data Pointer to calibration data
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_calib_data_memory(int leg_index, int joint, const joint_calib_t* calib_data);

/**
 * @brief Set calibration data for a specific joint (persistent)
 * 
 * @param leg_index Leg index (0-2: coxa, femur, tibia)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param calib_data Pointer to calibration data
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_calib_data_persist(int leg_index, int joint, const joint_calib_t* calib_data);

/**
 * @brief Set joint offset (memory-only)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param offset_rad Offset in radians
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_offset_memory(int leg_index, int joint, float offset_rad);

/**
 * @brief Set joint offset (persistent)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param offset_rad Offset in radians
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_offset_persist(int leg_index, int joint, float offset_rad);

/**
 * @brief Set joint angle limits (memory-only)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param min_rad Minimum angle in radians
 * @param max_rad Maximum angle in radians
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_limits_memory(int leg_index, int joint, float min_rad, float max_rad);

/**
 * @brief Set joint angle limits (persistent)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param min_rad Minimum angle in radians
 * @param max_rad Maximum angle in radians
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_limits_persist(int leg_index, int joint, float min_rad, float max_rad);

/**
 * @brief Set joint PWM values (memory-only)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param pwm_min_us PWM value at minimum angle (microseconds)
 * @param pwm_max_us PWM value at maximum angle (microseconds)
 * @param pwm_neutral_us PWM value at neutral position (microseconds)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_pwm_memory(int leg_index, int joint, int32_t pwm_min_us, int32_t pwm_max_us, int32_t pwm_neutral_us);

/**
 * @brief Set joint PWM values (persistent)
 * 
 * @param leg_index Leg index (0-5)
 * @param joint Joint type (0-2: coxa, femur, tibia)
 * @param pwm_min_us PWM value at minimum angle (microseconds)
 * @param pwm_max_us PWM value at maximum angle (microseconds)
 * @param pwm_neutral_us PWM value at neutral position (microseconds)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid indices
 */
esp_err_t config_set_joint_pwm_persist(int leg_index, int joint, int32_t pwm_min_us, int32_t pwm_max_us, int32_t pwm_neutral_us);

// =============================================================================
// Parameter Types and Metadata
// =============================================================================

typedef enum {
    CONFIG_TYPE_BOOL = 0,
    CONFIG_TYPE_INT32,
    CONFIG_TYPE_UINT16,
    CONFIG_TYPE_UINT32,
    CONFIG_TYPE_FLOAT,
    CONFIG_TYPE_STRING,
    CONFIG_TYPE_COUNT
} config_param_type_t;

typedef struct {
    const char* name;                   // Parameter name (e.g., "emergency_stop_enabled")
    config_param_type_t type;          // Data type
    size_t offset;                     // Offset in config struct
    size_t size;                       // Size in bytes
    union {
        struct { int32_t min, max; } int_range;
        struct { uint32_t min, max; } uint_range;
        struct { float min, max; } float_range;
        struct { size_t max_length; } string;
    } constraints;
} config_param_info_t;

// =============================================================================
// Hybrid Parameter API (Approach B - Individual Parameters)
// =============================================================================

/**
 * @brief Get boolean parameter value
 * 
 * @param namespace_str Namespace name (e.g., "system")
 * @param param_name Parameter name (e.g., "emergency_stop_enabled")
 * @param[out] value Pointer to store the value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_get_bool(const char* namespace_str, const char* param_name, bool* value);

/**
 * @brief Set boolean parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param value New value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_set_bool(const char* namespace_str, const char* param_name, bool value, bool persist);

/**
 * @brief Get 32-bit integer parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param[out] value Pointer to store the value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_get_int32(const char* namespace_str, const char* param_name, int32_t* value);

/**
 * @brief Set 32-bit integer parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param value New value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_set_int32(const char* namespace_str, const char* param_name, int32_t value, bool persist);

/**
 * @brief Get unsigned 32-bit integer parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param[out] value Pointer to store the value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_get_uint32(const char* namespace_str, const char* param_name, uint32_t* value);

/**
 * @brief Set unsigned 32-bit integer parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param value New value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_set_uint32(const char* namespace_str, const char* param_name, uint32_t value, bool persist);

/**
 * @brief Get float parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param[out] value Pointer to store the value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_get_float(const char* namespace_str, const char* param_name, float* value);

/**
 * @brief Set float parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param value New value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_set_float(const char* namespace_str, const char* param_name, float value, bool persist);

/**
 * @brief Get string parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param[out] value Buffer to store the string
 * @param max_len Maximum length of the buffer (including null terminator)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_get_string(const char* namespace_str, const char* param_name, char* value, size_t max_len);

/**
 * @brief Set string parameter value
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param value New string value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hexapod_config_set_string(const char* namespace_str, const char* param_name, const char* value, bool persist);

// =============================================================================
// Parameter Discovery and Metadata API
// =============================================================================

/**
 * @brief List all available namespaces
 * 
 * @param[out] namespace_names Array to store namespace name pointers
 * @param[out] count Number of namespaces returned
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_list_namespaces(const char** namespace_names, size_t* count);

/**
 * @brief List all parameters in a namespace
 * 
 * @param namespace_str Namespace name
 * @param[out] param_names Array to store parameter name pointers
 * @param max_params Maximum number of parameters to return
 * @param[out] count Number of parameters returned
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_list_parameters(const char* namespace_str, const char** param_names, 
                                size_t max_params, size_t* count);

/**
 * @brief Get parameter metadata and constraints
 * 
 * @param namespace_str Namespace name
 * @param param_name Parameter name
 * @param[out] info Parameter information structure to fill
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_get_parameter_info(const char* namespace_str, const char* param_name, 
                                   config_param_info_t* info);

// =============================================================================
// Legacy Generic Parameter API (kept for backward compatibility)
// =============================================================================

/**
 * @brief Get parameter value by string key (generic void* interface)
 * 
 * @param namespace_str Namespace name
 * @param key Parameter key
 * @param value_out Buffer for value (must be large enough for parameter type)
 * @param value_size Size of value buffer
 * @return ESP_OK on success, error code on failure
 * 
 * @note Prefer type-specific functions (hexapod_config_get_bool, hexapod_config_get_int32, hexapod_config_get_uint32, etc.)
 */
esp_err_t config_get_parameter(const char* namespace_str, const char* key, 
                               void* value_out, size_t value_size);

/**
 * @brief Set parameter value by string key (generic void* interface)
 * 
 * @param namespace_str Namespace name
 * @param key Parameter key
 * @param value Pointer to new value
 * @param value_size Size of value
 * @param persist true to save to NVS, false for memory-only
 * @return ESP_OK on success, error code on failure
 * 
 * @note Prefer type-specific functions (hexapod_config_set_bool, hexapod_config_set_int32, hexapod_config_set_uint32, etc.)
 */
esp_err_t config_set_parameter(const char* namespace_str, const char* key,
                               const void* value, size_t value_size, bool persist);

// =============================================================================
// Configuration Defaults and Factory Reset
// =============================================================================

/**
 * @brief Load factory default system configuration
 * 
 * @param config Pointer to system config structure to fill with defaults
 */
void config_load_system_defaults(system_config_t* config);

/**
 * @brief Load factory default joint calibration configuration
 * 
 * @param config Pointer to joint calibration config structure to fill with defaults
 */
void config_load_joint_calib_defaults(joint_calib_config_t* config);

/**
 * @brief Factory reset - restore all configuration to defaults
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_factory_reset(void);

// =============================================================================
// WiFi Partition Utilities (Optional)
// =============================================================================

/**
 * Read WiFi credentials from the default ESP-IDF WiFi partition
 * This is separate from robot configuration storage
 * 
 * @param ssid Buffer for WiFi SSID
 * @param ssid_len Size of SSID buffer  
 * @param password Buffer for WiFi password
 * @param password_len Size of password buffer
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if no credentials stored
 */
esp_err_t config_get_wifi_credentials(char* ssid, size_t ssid_len, char* password, size_t password_len);

#ifdef __cplusplus
}
#endif