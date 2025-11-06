/*
 * Configuration Manager Implementation
 * 
 * NVS-based configuration persistence with dual-method API.
 * 
 * License: Apache-2.0
 */

#include "config_manager.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "config_mgr";

// Configuration version for migration support
#define CONFIG_SCHEMA_VERSION 1

// NVS partition names
#define NVS_PARTITION_WIFI   "nvs"        // Default ESP-IDF partition for WiFi
#define NVS_PARTITION_ROBOT  "nvs_robot"  // Custom partition for robot config

// =============================================================================
// Global State
// =============================================================================

// Namespace string mappings (must match config_namespace_t enum order)
const char* CONFIG_NAMESPACE_NAMES[CONFIG_NS_COUNT] = {
    "system"
};

// Global manager state
static config_manager_state_t g_manager_state = {0};

// Configuration cache - system namespace
static system_config_t g_system_config = {0};

// =============================================================================
// NVS Parameter Keys for System Namespace
// =============================================================================

// System configuration parameter keys
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

// =============================================================================
// Helper Functions
// =============================================================================

static esp_err_t load_system_config_from_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    esp_err_t err;
    
    ESP_LOGI(TAG, "Loading system configuration from NVS");
    
    // Load each parameter with fallback to defaults
    size_t required_size = 0;
    
    // Boolean parameters
    uint8_t temp_bool = g_system_config.emergency_stop_enabled ? 1 : 0;
    err = nvs_get_u8(handle, SYS_KEY_EMERGENCY_STOP, &temp_bool);
    if (err == ESP_OK) {
        g_system_config.emergency_stop_enabled = (temp_bool != 0);
    }
    
    // Integer parameters
    nvs_get_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, &g_system_config.auto_disarm_timeout);
    nvs_get_u32(handle, SYS_KEY_MOTION_TIMEOUT, &g_system_config.motion_timeout_ms);
    nvs_get_u32(handle, SYS_KEY_STARTUP_DELAY, &g_system_config.startup_delay_ms);
    nvs_get_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, &g_system_config.max_control_frequency);
    
    // Float parameters (stored as blobs for precision)
    required_size = sizeof(float);
    nvs_get_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &g_system_config.safety_voltage_min, &required_size);
    required_size = sizeof(float);
    nvs_get_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &g_system_config.temperature_limit_max, &required_size);
    
    // String parameters
    required_size = sizeof(g_system_config.robot_id);
    nvs_get_str(handle, SYS_KEY_ROBOT_ID, g_system_config.robot_id, &required_size);
    
    required_size = sizeof(g_system_config.robot_name);
    nvs_get_str(handle, SYS_KEY_ROBOT_NAME, g_system_config.robot_name, &required_size);
    
    // Version parameter
    uint16_t version = CONFIG_SCHEMA_VERSION;
    err = nvs_get_u16(handle, SYS_KEY_CONFIG_VERSION, &version);
    g_system_config.config_version = version;
    
    // TODO: Handle version migration here when needed
    if (version != CONFIG_SCHEMA_VERSION) {
        ESP_LOGW(TAG, "Config version mismatch: stored=%d, current=%d", version, CONFIG_SCHEMA_VERSION);
    }
    
    g_manager_state.namespace_loaded[CONFIG_NS_SYSTEM] = true;
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = false;
    
    ESP_LOGI(TAG, "System config loaded - robot_id=%s, robot_name=%s", 
             g_system_config.robot_id, g_system_config.robot_name);
    
    return ESP_OK;
}

static esp_err_t save_system_config_to_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    esp_err_t err;
    
    ESP_LOGI(TAG, "Saving system configuration to NVS");
    
    // Save each parameter
    uint8_t temp_bool = g_system_config.emergency_stop_enabled ? 1 : 0;
    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_EMERGENCY_STOP, temp_bool));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, g_system_config.auto_disarm_timeout));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MOTION_TIMEOUT, g_system_config.motion_timeout_ms));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_STARTUP_DELAY, g_system_config.startup_delay_ms));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, g_system_config.max_control_frequency));
    
    // Float parameters as blobs
    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &g_system_config.safety_voltage_min, sizeof(float)));
    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &g_system_config.temperature_limit_max, sizeof(float)));
    
    // String parameters
    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_ID, g_system_config.robot_id));
    ESP_ERROR_CHECK(nvs_set_str(handle, SYS_KEY_ROBOT_NAME, g_system_config.robot_name));
    
    // Version
    ESP_ERROR_CHECK(nvs_set_u16(handle, SYS_KEY_CONFIG_VERSION, g_system_config.config_version));
    
    // Commit changes
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit system config to NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = false;
    ESP_LOGI(TAG, "System configuration saved successfully");
    
    return ESP_OK;
}

// =============================================================================
// Core Configuration Manager API
// =============================================================================

esp_err_t config_manager_init(void) {
    esp_err_t err;
    
    if (g_manager_state.initialized) {
        ESP_LOGW(TAG, "Configuration manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing configuration manager");
    
    // Initialize default NVS partition (for WiFi)
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Default NVS partition needs to be erased, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // Initialize robot NVS partition
    err = nvs_flash_init_partition(NVS_PARTITION_ROBOT);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Robot NVS partition needs to be erased, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase_partition(NVS_PARTITION_ROBOT));
        err = nvs_flash_init_partition(NVS_PARTITION_ROBOT);
    }
    ESP_ERROR_CHECK(err);
    
    ESP_LOGI(TAG, "Both NVS partitions initialized successfully");
    
    // Verify robot partition exists
    const esp_partition_t* robot_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
                                                                      ESP_PARTITION_SUBTYPE_DATA_NVS, 
                                                                      NVS_PARTITION_ROBOT);
    if (robot_partition == NULL) {
        ESP_LOGE(TAG, "Robot NVS partition '%s' not found in partition table!", NVS_PARTITION_ROBOT);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Robot partition found: label='%s', size=%lu bytes, offset=0x%lx", 
             robot_partition->label, (unsigned long)robot_partition->size, 
             (unsigned long)robot_partition->address);
    
    // Open NVS handles for all namespaces in the robot partition
    for (int i = 0; i < CONFIG_NS_COUNT; i++) {
        ESP_LOGI(TAG, "Opening namespace '%s' in partition '%s'...", 
                 CONFIG_NAMESPACE_NAMES[i], NVS_PARTITION_ROBOT);
        
        err = nvs_open_from_partition(NVS_PARTITION_ROBOT, CONFIG_NAMESPACE_NAMES[i], 
                                      NVS_READWRITE, &g_manager_state.nvs_handles[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open NVS namespace '%s' from partition '%s': %s", 
                     CONFIG_NAMESPACE_NAMES[i], NVS_PARTITION_ROBOT, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "✓ Opened NVS namespace: %s (partition: %s)", 
                 CONFIG_NAMESPACE_NAMES[i], NVS_PARTITION_ROBOT);
    }
    
    // Load default configurations
    config_load_system_defaults(&g_system_config);
    
    // Load configurations from NVS (will use defaults for missing values)
    load_system_config_from_nvs();
    
    g_manager_state.initialized = true;
    ESP_LOGI(TAG, "Configuration manager initialized successfully");
    
    return ESP_OK;
}

esp_err_t config_manager_get_state(config_manager_state_t *state) {
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *state = g_manager_state;
    return ESP_OK;
}

bool config_manager_has_dirty_data(void) {
    for (int i = 0; i < CONFIG_NS_COUNT; i++) {
        if (g_manager_state.namespace_dirty[i]) {
            return true;
        }
    }
    return false;
}

esp_err_t config_manager_reload_namespace(config_namespace_t ns) {
    if (ns >= CONFIG_NS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_manager_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Reloading namespace: %s", CONFIG_NAMESPACE_NAMES[ns]);
    
    switch (ns) {
        case CONFIG_NS_SYSTEM:
            config_load_system_defaults(&g_system_config);
            return load_system_config_from_nvs();
        
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t config_manager_save_namespace(config_namespace_t ns) {
    if (ns >= CONFIG_NS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_manager_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Saving namespace: %s", CONFIG_NAMESPACE_NAMES[ns]);
    
    switch (ns) {
        case CONFIG_NS_SYSTEM:
            return save_system_config_to_nvs();
        
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

// =============================================================================
// System Configuration API
// =============================================================================

const system_config_t* config_get_system(void) {
    return &g_system_config;
}

system_config_t* config_get_system_mutable(void) {
    return &g_system_config;
}

// =============================================================================
// Dual-Method System Parameter API
// =============================================================================

esp_err_t config_set_emergency_stop_memory(bool enabled) {
    g_system_config.emergency_stop_enabled = enabled;
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    ESP_LOGD(TAG, "Set emergency stop (memory): %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t config_set_emergency_stop_persist(bool enabled) {
    esp_err_t err = config_set_emergency_stop_memory(enabled);
    if (err != ESP_OK) {
        return err;
    }
    return config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

esp_err_t config_set_auto_disarm_timeout_memory(uint32_t timeout_sec) {
    g_system_config.auto_disarm_timeout = timeout_sec;
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    ESP_LOGD(TAG, "Set auto-disarm timeout (memory): %lu seconds", (unsigned long)timeout_sec);
    return ESP_OK;
}

esp_err_t config_set_auto_disarm_timeout_persist(uint32_t timeout_sec) {
    esp_err_t err = config_set_auto_disarm_timeout_memory(timeout_sec);
    if (err != ESP_OK) {
        return err;
    }
    return config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

esp_err_t config_set_safety_voltage_min_memory(float voltage_min) {
    g_system_config.safety_voltage_min = voltage_min;
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    ESP_LOGD(TAG, "Set safety voltage min (memory): %.2f V", voltage_min);
    return ESP_OK;
}

esp_err_t config_set_safety_voltage_min_persist(float voltage_min) {
    esp_err_t err = config_set_safety_voltage_min_memory(voltage_min);
    if (err != ESP_OK) {
        return err;
    }
    return config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

esp_err_t config_set_robot_name_memory(const char* name) {
    if (!name || strlen(name) >= sizeof(g_system_config.robot_name)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(g_system_config.robot_name, name, sizeof(g_system_config.robot_name) - 1);
    g_system_config.robot_name[sizeof(g_system_config.robot_name) - 1] = '\0';
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    ESP_LOGD(TAG, "Set robot name (memory): %s", name);
    return ESP_OK;
}

esp_err_t config_set_robot_name_persist(const char* name) {
    esp_err_t err = config_set_robot_name_memory(name);
    if (err != ESP_OK) {
        return err;
    }
    return config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

esp_err_t config_set_robot_id_memory(const char* id) {
    if (!id || strlen(id) >= sizeof(g_system_config.robot_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(g_system_config.robot_id, id, sizeof(g_system_config.robot_id) - 1);
    g_system_config.robot_id[sizeof(g_system_config.robot_id) - 1] = '\0';
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    ESP_LOGD(TAG, "Set robot ID (memory): %s", id);
    return ESP_OK;
}

esp_err_t config_set_robot_id_persist(const char* id) {
    esp_err_t err = config_set_robot_id_memory(id);
    if (err != ESP_OK) {
        return err;
    }
    return config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

// =============================================================================
// Configuration Defaults
// =============================================================================

void config_load_system_defaults(system_config_t* config) {
    if (!config) {
        return;
    }
    
    memset(config, 0, sizeof(system_config_t));
    
    // Safety defaults
    config->emergency_stop_enabled = true;
    config->auto_disarm_timeout = 30;      // 30 seconds
    config->safety_voltage_min = 6.5f;     // 6.5V minimum for 2S LiPo
    config->temperature_limit_max = 80.0f; // 80°C maximum
    config->motion_timeout_ms = 1000;      // 1 second motion timeout
    config->startup_delay_ms = 2000;       // 2 second startup delay
    config->max_control_frequency = 100;   // 100 Hz max control frequency
    
    // Generate default robot ID from MAC address
    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err == ESP_OK) {
        snprintf(config->robot_id, sizeof(config->robot_id), "HEXAPOD_%02X%02X%02X", 
                 mac[3], mac[4], mac[5]);
    } else {
        strcpy(config->robot_id, "HEXAPOD_DEFAULT");
    }
    
    // Default robot name
    strcpy(config->robot_name, "My Hexapod Robot");
    
    // Configuration version
    config->config_version = CONFIG_SCHEMA_VERSION;
    
    ESP_LOGD(TAG, "Loaded system defaults - robot_id=%s", config->robot_id);
}

esp_err_t config_factory_reset(void) {
    ESP_LOGW(TAG, "Performing factory reset - robot configuration will be lost!");
    ESP_LOGI(TAG, "Note: WiFi credentials in default partition will be preserved");
    
    // Clear all robot configuration namespaces (preserves WiFi partition)
    for (int i = 0; i < CONFIG_NS_COUNT; i++) {
        esp_err_t err = nvs_erase_all(g_manager_state.nvs_handles[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase namespace %s: %s", 
                     CONFIG_NAMESPACE_NAMES[i], esp_err_to_name(err));
            return err;
        }
        nvs_commit(g_manager_state.nvs_handles[i]);
    }
    
    // Reload defaults
    config_load_system_defaults(&g_system_config);
    
    // Clear dirty flags
    for (int i = 0; i < CONFIG_NS_COUNT; i++) {
        g_manager_state.namespace_dirty[i] = false;
        g_manager_state.namespace_loaded[i] = true;
    }
    
    ESP_LOGI(TAG, "Factory reset completed");
    return ESP_OK;
}

// =============================================================================
// Generic Parameter API (Basic Implementation)
// =============================================================================

esp_err_t config_get_parameter(const char* namespace_str, const char* key, 
                               void* value_out, size_t value_size) {
    // Basic implementation for system namespace only
    if (!namespace_str || !key || !value_out) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        // Handle system parameters
        if (strcmp(key, "emergency_stop_enabled") == 0 && value_size >= sizeof(bool)) {
            *(bool*)value_out = g_system_config.emergency_stop_enabled;
            return ESP_OK;
        } else if (strcmp(key, "robot_name") == 0 && value_size >= strlen(g_system_config.robot_name) + 1) {
            strcpy((char*)value_out, g_system_config.robot_name);
            return ESP_OK;
        }
        // Add more parameters as needed
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t config_set_parameter_memory(const char* namespace_str, const char* key,
                                      const void* value, size_t value_size) {
    // Basic implementation for system namespace only
    if (!namespace_str || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        if (strcmp(key, "emergency_stop_enabled") == 0 && value_size == sizeof(bool)) {
            return config_set_emergency_stop_memory(*(const bool*)value);
        } else if (strcmp(key, "robot_name") == 0) {
            return config_set_robot_name_memory((const char*)value);
        }
        // Add more parameters as needed
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t config_set_parameter_persist(const char* namespace_str, const char* key,
                                       const void* value, size_t value_size) {
    esp_err_t err = config_set_parameter_memory(namespace_str, key, value, value_size);
    if (err != ESP_OK) {
        return err;
    }
    
    // Determine which namespace to save
    if (strcmp(namespace_str, "system") == 0) {
        return config_manager_save_namespace(CONFIG_NS_SYSTEM);
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t config_enumerate_keys(const char* namespace_str, const char* key_list[], 
                                size_t max_keys, size_t* actual_keys) {
    if (!namespace_str || !key_list || !actual_keys) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *actual_keys = 0;
    
    if (strcmp(namespace_str, "system") == 0) {
        // List system configuration keys
        const char* system_keys[] = {
            "emergency_stop_enabled",
            "auto_disarm_timeout", 
            "safety_voltage_min",
            "temperature_limit_max",
            "motion_timeout_ms",
            "startup_delay_ms",
            "max_control_frequency",
            "robot_id",
            "robot_name",
            "config_version"
        };
        
        size_t num_keys = sizeof(system_keys) / sizeof(system_keys[0]);
        size_t keys_to_copy = (num_keys < max_keys) ? num_keys : max_keys;
        
        for (size_t i = 0; i < keys_to_copy; i++) {
            key_list[i] = system_keys[i];
        }
        
        *actual_keys = keys_to_copy;
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

// =============================================================================
// Utility Functions for WiFi Partition Access (if needed)
// =============================================================================

esp_err_t config_get_wifi_credentials(char* ssid, size_t ssid_len, char* password, size_t password_len) {
    nvs_handle_t wifi_handle;
    esp_err_t err;
    
    // Open the default WiFi NVS partition  
    err = nvs_open_from_partition(NVS_PARTITION_WIFI, "nvs", NVS_READONLY, &wifi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open WiFi NVS partition: %s", esp_err_to_name(err));
        return err;
    }
    
    // Read WiFi credentials (ESP-IDF standard keys)
    size_t required_size = ssid_len;
    err = nvs_get_str(wifi_handle, "wifi.ssid", ssid, &required_size);
    if (err == ESP_OK) {
        required_size = password_len;
        err = nvs_get_str(wifi_handle, "wifi.pswd", password, &required_size);
    }
    
    nvs_close(wifi_handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "WiFi credentials not found in NVS");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading WiFi credentials: %s", esp_err_to_name(err));
    }
    
    return err;
}