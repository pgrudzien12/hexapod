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

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration Namespaces
// =============================================================================

typedef enum {
    CONFIG_NS_SYSTEM = 0,        // System-wide settings and safety
    CONFIG_NS_COUNT              // Keep this last
} config_namespace_t;

// Namespace string mappings (must match enum order)
extern const char* CONFIG_NAMESPACE_NAMES[CONFIG_NS_COUNT];

// =============================================================================
// System Configuration Structure
// =============================================================================

typedef struct {
    // Safety settings
    bool emergency_stop_enabled;     // Emergency stop functionality
    uint32_t auto_disarm_timeout;    // Auto-disarm timeout (seconds)
    float safety_voltage_min;        // Minimum battery voltage (volts)
    float temperature_limit_max;     // Maximum operating temperature (°C)
    uint32_t motion_timeout_ms;      // Motion command timeout
    uint32_t startup_delay_ms;       // Startup safety delay
    uint32_t max_control_frequency;  // Maximum control loop frequency (Hz)
    
    // System identification
    char robot_id[32];               // Unique robot identifier
    char robot_name[64];             // Human-readable robot name
    uint16_t config_version;         // Configuration schema version
} system_config_t;

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
 * @brief Force reload namespace from NVS (discards memory changes)
 * 
 * @param ns Namespace to reload
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_manager_reload_namespace(config_namespace_t ns);

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
 * @brief Get system configuration (mutable, for memory-only changes)
 * 
 * @return Pointer to system configuration structure (writable)
 */
system_config_t* config_get_system_mutable(void);

// Dual-method API for individual system parameters

/**
 * @brief Set emergency stop enable (memory only)
 * 
 * @param enabled Emergency stop enabled state
 * @return ESP_OK on success
 */
esp_err_t config_set_emergency_stop_memory(bool enabled);

/**
 * @brief Set emergency stop enable (persistent)
 * 
 * @param enabled Emergency stop enabled state
 * @return ESP_OK on success, error code on NVS write failure
 */
esp_err_t config_set_emergency_stop_persist(bool enabled);

/**
 * @brief Set auto-disarm timeout (memory only)
 * 
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success
 */
esp_err_t config_set_auto_disarm_timeout_memory(uint32_t timeout_sec);

/**
 * @brief Set auto-disarm timeout (persistent)
 * 
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code on NVS write failure
 */
esp_err_t config_set_auto_disarm_timeout_persist(uint32_t timeout_sec);

/**
 * @brief Set safety voltage minimum (memory only)
 * 
 * @param voltage_min Minimum voltage in volts
 * @return ESP_OK on success
 */
esp_err_t config_set_safety_voltage_min_memory(float voltage_min);

/**
 * @brief Set safety voltage minimum (persistent)
 * 
 * @param voltage_min Minimum voltage in volts
 * @return ESP_OK on success, error code on NVS write failure
 */
esp_err_t config_set_safety_voltage_min_persist(float voltage_min);

/**
 * @brief Set robot name (memory only)
 * 
 * @param name Robot name string (max 63 chars)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if name too long
 */
esp_err_t config_set_robot_name_memory(const char* name);

/**
 * @brief Set robot name (persistent)
 * 
 * @param name Robot name string (max 63 chars)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_set_robot_name_persist(const char* name);

/**
 * @brief Set robot ID (memory only)
 * 
 * @param id Robot ID string (max 31 chars)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if id too long
 */
esp_err_t config_set_robot_id_memory(const char* id);

/**
 * @brief Set robot ID (persistent)
 * 
 * @param id Robot ID string (max 31 chars)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_set_robot_id_persist(const char* id);

// =============================================================================
// Generic Parameter API (for RPC integration)
// =============================================================================

/**
 * @brief Get parameter value by string key
 * 
 * @param namespace_str Namespace name
 * @param key Parameter key
 * @param value_out Buffer for value (must be large enough for parameter type)
 * @param value_size Size of value buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_get_parameter(const char* namespace_str, const char* key, 
                               void* value_out, size_t value_size);

/**
 * @brief Set parameter value by string key (memory only)
 * 
 * @param namespace_str Namespace name
 * @param key Parameter key
 * @param value Pointer to new value
 * @param value_size Size of value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_set_parameter_memory(const char* namespace_str, const char* key,
                                      const void* value, size_t value_size);

/**
 * @brief Set parameter value by string key (persistent)
 * 
 * @param namespace_str Namespace name
 * @param key Parameter key
 * @param value Pointer to new value
 * @param value_size Size of value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_set_parameter_persist(const char* namespace_str, const char* key,
                                       const void* value, size_t value_size);

/**
 * @brief Enumerate parameter keys in namespace
 * 
 * @param namespace_str Namespace name
 * @param key_list Array of string pointers to fill
 * @param max_keys Maximum number of keys to return
 * @param actual_keys[out] Actual number of keys returned
 * @return ESP_OK on success, error code on failure
 */
esp_err_t config_enumerate_keys(const char* namespace_str, const char* key_list[], 
                                size_t max_keys, size_t* actual_keys);

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