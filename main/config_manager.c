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
#include <stddef.h>

static const char *TAG = "config_mgr";

// Configuration version for migration support
#define CONFIG_SCHEMA_VERSION 1

// Global configuration version key (stored in system namespace for now)
#define GLOBAL_CONFIG_VERSION_KEY "global_ver"  // Max 15 chars for NVS

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
// Global Version Management
// =============================================================================

static esp_err_t read_global_config_version(uint16_t* version) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    esp_err_t err = nvs_get_u16(handle, GLOBAL_CONFIG_VERSION_KEY, version);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *version = 0;  // Fresh partition
        return ESP_OK;
    }
    
    return err;
}

static esp_err_t save_global_config_version(uint16_t version) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    ESP_ERROR_CHECK(nvs_set_u16(handle, GLOBAL_CONFIG_VERSION_KEY, version));
    ESP_ERROR_CHECK(nvs_commit(handle));
    return ESP_OK;
}

// =============================================================================
// Migration System
// =============================================================================

static esp_err_t init_system_defaults_to_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    
    ESP_LOGI(TAG, "Initializing system namespace defaults to NVS");
    
    // Write default values to NVS (will be loaded later)
    ESP_ERROR_CHECK(nvs_set_u8(handle, SYS_KEY_EMERGENCY_STOP, 1));  // true
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, 30));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MOTION_TIMEOUT, 1000));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_STARTUP_DELAY, 2000));
    ESP_ERROR_CHECK(nvs_set_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, 100));
    
    // Float defaults as blobs
    float default_voltage = 6.5f;
    float default_temp = 80.0f;
    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &default_voltage, sizeof(float)));
    ESP_ERROR_CHECK(nvs_set_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &default_temp, sizeof(float)));
    
    // Generate robot ID from MAC
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
    
    // Schema version for this namespace
    ESP_ERROR_CHECK(nvs_set_u16(handle, SYS_KEY_CONFIG_VERSION, CONFIG_SCHEMA_VERSION));
    
    ESP_ERROR_CHECK(nvs_commit(handle));
    ESP_LOGI(TAG, "System defaults written to NVS");
    
    return ESP_OK;
}

static esp_err_t migrate_v0_to_v1(void) {
    ESP_LOGI(TAG, "Migrating v0 -> v1: Initializing fresh configuration");
    
    // Initialize all namespaces with defaults
    ESP_ERROR_CHECK(init_system_defaults_to_nvs());
    
    // Future: Add other namespace initialization here
    // ESP_ERROR_CHECK(init_motion_limits_defaults_to_nvs());
    // ESP_ERROR_CHECK(init_joint_calib_defaults_to_nvs());
    
    ESP_LOGI(TAG, "Migration v0 -> v1 completed");
    return ESP_OK;
}

static esp_err_t config_migrate_version(uint16_t from, uint16_t to) {
    ESP_LOGI(TAG, "Migrating configuration schema: v%d -> v%d", from, to);
    
    switch (from) {
        case 0:
            if (to == 1) {
                return migrate_v0_to_v1();
            }
            break;
            
        case 1:
            // Future: v1 -> v2 migration
            ESP_LOGW(TAG, "Migration v1 -> v%d not yet implemented", to);
            return ESP_ERR_NOT_SUPPORTED;
            
        default:
            ESP_LOGE(TAG, "Unknown migration path: v%d -> v%d", from, to);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t config_migrate_all(uint16_t from_version, uint16_t to_version) {
    ESP_LOGI(TAG, "Starting configuration migration: v%d -> v%d", from_version, to_version);
    
    if (from_version == to_version) {
        ESP_LOGI(TAG, "No migration needed - versions match");
        return ESP_OK;
    }
    
    if (from_version > to_version) {
        ESP_LOGE(TAG, "Downgrade not supported: v%d -> v%d", from_version, to_version);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Run migrations sequentially
    for (uint16_t current_version = from_version; current_version < to_version; current_version++) {
        esp_err_t err = config_migrate_version(current_version, current_version + 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Migration failed at v%d -> v%d: %s", 
                     current_version, current_version + 1, esp_err_to_name(err));
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Migration completed successfully");
    return ESP_OK;
}

// =============================================================================
// Helper Functions  
// =============================================================================

static esp_err_t migrate_system_config(uint16_t from_version, uint16_t to_version) {
    ESP_LOGI(TAG, "Migrating system config from version %d to %d", from_version, to_version);
    
    // Migration logic will be implemented here when schema changes occur
    // For now, this is a placeholder that handles version transitions
    
    switch (from_version) {
        case 0:
            // Fresh partition - no migration needed, defaults will be used
            ESP_LOGI(TAG, "Fresh partition - using default values");
            break;
            
        case 1:
            // Future: Add migration logic for v1 -> v2 when needed
            // Example: rename keys, convert formats, etc.
            ESP_LOGW(TAG, "Migration from v1 not yet implemented - using defaults");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown version %d - using defaults", from_version);
            break;
    }
    
    return ESP_OK;
}

static esp_err_t load_system_config_from_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_SYSTEM];
    esp_err_t err;
    
    ESP_LOGI(TAG, "Loading system configuration from NVS (read-only)");
    
    // PURE READ FUNCTION - No migration, no writes, assumes correct schema
    size_t required_size = 0;
    
    // Boolean parameters
    uint8_t temp_bool = 0;
    err = nvs_get_u8(handle, SYS_KEY_EMERGENCY_STOP, &temp_bool);
    if (err == ESP_OK) {
        g_system_config.emergency_stop_enabled = (temp_bool != 0);
    } else {
        ESP_LOGW(TAG, "Failed to read emergency_stop, using default: %s", esp_err_to_name(err));
    }
    
    // Integer parameters
    err = nvs_get_u32(handle, SYS_KEY_AUTO_DISARM_TIMEOUT, &g_system_config.auto_disarm_timeout);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read auto_disarm_timeout: %s", esp_err_to_name(err));
    }
    
    err = nvs_get_u32(handle, SYS_KEY_MOTION_TIMEOUT, &g_system_config.motion_timeout_ms);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read motion_timeout: %s", esp_err_to_name(err));
    }
    
    err = nvs_get_u32(handle, SYS_KEY_STARTUP_DELAY, &g_system_config.startup_delay_ms);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read startup_delay: %s", esp_err_to_name(err));
    }
    
    err = nvs_get_u32(handle, SYS_KEY_MAX_CONTROL_FREQ, &g_system_config.max_control_frequency);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read max_control_freq: %s", esp_err_to_name(err));
    }
    
    // Float parameters (stored as blobs for precision)
    required_size = sizeof(float);
    err = nvs_get_blob(handle, SYS_KEY_SAFETY_VOLTAGE_MIN, &g_system_config.safety_voltage_min, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read safety_voltage_min: %s", esp_err_to_name(err));
    }
    
    required_size = sizeof(float);
    err = nvs_get_blob(handle, SYS_KEY_TEMP_LIMIT_MAX, &g_system_config.temperature_limit_max, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read temp_limit_max: %s", esp_err_to_name(err));
    }
    
    // String parameters
    required_size = sizeof(g_system_config.robot_id);
    err = nvs_get_str(handle, SYS_KEY_ROBOT_ID, g_system_config.robot_id, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read robot_id: %s", esp_err_to_name(err));
    }
    
    required_size = sizeof(g_system_config.robot_name);
    err = nvs_get_str(handle, SYS_KEY_ROBOT_NAME, g_system_config.robot_name, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read robot_name: %s", esp_err_to_name(err));
    }
    
    // Read namespace version
    uint16_t namespace_version = 0;
    err = nvs_get_u16(handle, SYS_KEY_CONFIG_VERSION, &namespace_version);
    if (err == ESP_OK) {
        g_system_config.config_version = namespace_version;
    } else {
        ESP_LOGW(TAG, "Failed to read namespace config version: %s", esp_err_to_name(err));
        g_system_config.config_version = CONFIG_SCHEMA_VERSION;  // Assume current
    }
    
    g_manager_state.namespace_loaded[CONFIG_NS_SYSTEM] = true;
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = false;
    
    ESP_LOGI(TAG, "System config loaded - robot_id=%s, robot_name=%s, version=%d", 
             g_system_config.robot_id, g_system_config.robot_name, g_system_config.config_version);
    
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
    
    // STEP 1: Read global configuration version
    uint16_t stored_version = 0;
    err = read_global_config_version(&stored_version);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read global config version: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Global config version: stored=%d, current=%d", stored_version, CONFIG_SCHEMA_VERSION);
    
    // STEP 2: Run migration FIRST if needed (before any loading)
    if (stored_version != CONFIG_SCHEMA_VERSION) {
        ESP_LOGI(TAG, "Configuration migration required");
        err = config_migrate_all(stored_version, CONFIG_SCHEMA_VERSION);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Migration failed: %s", esp_err_to_name(err));
            return err;
        }
        
        // Save updated global version
        err = save_global_config_version(CONFIG_SCHEMA_VERSION);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save new global version: %s", esp_err_to_name(err));
            return err;
        }
        
        ESP_LOGI(TAG, "Configuration migration completed successfully");
    } else {
        ESP_LOGI(TAG, "No migration needed - configuration up to date");
    }
    
    // STEP 3: Load default configurations into memory cache
    config_load_system_defaults(&g_system_config);
    
    // STEP 4: Load configurations from NVS (guaranteed correct schema now)
    err = load_system_config_from_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load system config from NVS: %s", esp_err_to_name(err));
        return err;
    }
    
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

// =============================================================================
// Parameter Tables and Metadata
// =============================================================================

// System namespace parameter table
static const config_param_info_t g_system_param_table[] = {
    {
        .name = "emergency_stop_enabled",
        .type = CONFIG_TYPE_BOOL,
        .offset = offsetof(system_config_t, emergency_stop_enabled),
        .size = sizeof(bool),
        .description = "Emergency stop functionality enabled",
        .constraints = { .int_range = { 0, 1 } }
    },
    {
        .name = "auto_disarm_timeout",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, auto_disarm_timeout),
        .size = sizeof(uint32_t),
        .description = "Auto-disarm timeout in seconds",
        .constraints = { .uint_range = { 5, 300 } }
    },
    {
        .name = "safety_voltage_min",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(system_config_t, safety_voltage_min),
        .size = sizeof(float),
        .description = "Minimum battery voltage in volts",
        .constraints = { .float_range = { 3.0f, 12.0f } }
    },
    {
        .name = "temperature_limit_max",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(system_config_t, temperature_limit_max),
        .size = sizeof(float),
        .description = "Maximum operating temperature in Celsius",
        .constraints = { .float_range = { 40.0f, 100.0f } }
    },
    {
        .name = "motion_timeout_ms",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, motion_timeout_ms),
        .size = sizeof(uint32_t),
        .description = "Motion command timeout in milliseconds",
        .constraints = { .uint_range = { 100, 5000 } }
    },
    {
        .name = "startup_delay_ms",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, startup_delay_ms),
        .size = sizeof(uint32_t),
        .description = "Startup safety delay in milliseconds",
        .constraints = { .uint_range = { 0, 10000 } }
    },
    {
        .name = "max_control_frequency",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, max_control_frequency),
        .size = sizeof(uint32_t),
        .description = "Maximum control loop frequency in Hz",
        .constraints = { .uint_range = { 50, 1000 } }
    },
    {
        .name = "robot_id",
        .type = CONFIG_TYPE_STRING,
        .offset = offsetof(system_config_t, robot_id),
        .size = sizeof(((system_config_t*)0)->robot_id),
        .description = "Unique robot identifier",
        .constraints = { .string = { 32 } }
    },
    {
        .name = "robot_name",
        .type = CONFIG_TYPE_STRING,
        .offset = offsetof(system_config_t, robot_name),
        .size = sizeof(((system_config_t*)0)->robot_name),
        .description = "Human-readable robot name",
        .constraints = { .string = { 64 } }
    },
    {
        .name = "config_version",
        .type = CONFIG_TYPE_UINT16,
        .offset = offsetof(system_config_t, config_version),
        .size = sizeof(uint16_t),
        .description = "Configuration schema version",
        .constraints = { .uint_range = { 1, 65535 } }
    }
};

static const size_t g_system_param_count = sizeof(g_system_param_table) / sizeof(g_system_param_table[0]);

// Helper function to find parameter in table
static const config_param_info_t* find_system_param(const char* param_name) {
    for (size_t i = 0; i < g_system_param_count; i++) {
        if (strcmp(g_system_param_table[i].name, param_name) == 0) {
            return &g_system_param_table[i];
        }
    }
    return NULL;
}

// Helper function to get parameter pointer in config struct
static void* get_system_param_ptr(const config_param_info_t* param) {
    return (uint8_t*)&g_system_config + param->offset;
}

// =============================================================================
// Bulk Configuration API (Approach A)
// =============================================================================

esp_err_t config_set_system(const system_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&g_system_config, config, sizeof(system_config_t));
    g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
    
    esp_err_t err = config_manager_save_namespace(CONFIG_NS_SYSTEM);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "System configuration updated (bulk operation)");
    }
    return err;
}

// =============================================================================
// Hybrid Parameter API (Approach B)
// =============================================================================

esp_err_t hexapod_config_get_bool(const char* namespace_str, const char* param_name, bool* value) {
    if (!namespace_str || !param_name || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_BOOL) {
            return ESP_ERR_NOT_FOUND;
        }
        
        bool* param_ptr = (bool*)get_system_param_ptr(param);
        *value = *param_ptr;
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_set_bool(const char* namespace_str, const char* param_name, bool value, bool persist) {
    if (!namespace_str || !param_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_BOOL) {
            return ESP_ERR_NOT_FOUND;
        }
        
        bool* param_ptr = (bool*)get_system_param_ptr(param);
        *param_ptr = value;
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %s %s", namespace_str, param_name, 
                 value ? "true" : "false", persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            return config_manager_save_namespace(CONFIG_NS_SYSTEM);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_get_int32(const char* namespace_str, const char* param_name, int32_t* value) {
    if (!namespace_str || !param_name || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_INT32) {
            return ESP_ERR_NOT_FOUND;
        }
        
        int32_t* param_ptr = (int32_t*)get_system_param_ptr(param);
        *value = *param_ptr;
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_set_int32(const char* namespace_str, const char* param_name, int32_t value, bool persist) {
    if (!namespace_str || !param_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_INT32) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Validate constraints
        if (value < param->constraints.int_range.min || value > param->constraints.int_range.max) {
            ESP_LOGE(TAG, "Value %ld for %s.%s out of range [%ld, %ld]", 
                     (long)value, namespace_str, param_name,
                     (long)param->constraints.int_range.min, 
                     (long)param->constraints.int_range.max);
            return ESP_ERR_INVALID_ARG;
        }
        
        int32_t* param_ptr = (int32_t*)get_system_param_ptr(param);
        *param_ptr = value;
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %ld %s", namespace_str, param_name, 
                 (long)value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            return config_manager_save_namespace(CONFIG_NS_SYSTEM);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_get_uint32(const char* namespace_str, const char* param_name, uint32_t* value) {
    if (!namespace_str || !param_name || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || (param->type != CONFIG_TYPE_UINT32 && param->type != CONFIG_TYPE_UINT16)) {
            return ESP_ERR_NOT_FOUND;
        }
        
        if (param->type == CONFIG_TYPE_UINT32) {
            uint32_t* param_ptr = (uint32_t*)get_system_param_ptr(param);
            *value = *param_ptr;
        } else { // CONFIG_TYPE_UINT16
            uint16_t* param_ptr = (uint16_t*)get_system_param_ptr(param);
            *value = (uint32_t)*param_ptr;
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_set_uint32(const char* namespace_str, const char* param_name, uint32_t value, bool persist) {
    if (!namespace_str || !param_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || (param->type != CONFIG_TYPE_UINT32 && param->type != CONFIG_TYPE_UINT16)) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Validate constraints
        if (value < param->constraints.uint_range.min || value > param->constraints.uint_range.max) {
            ESP_LOGE(TAG, "Value %lu for %s.%s out of range [%lu, %lu]", 
                     (unsigned long)value, namespace_str, param_name,
                     (unsigned long)param->constraints.uint_range.min, 
                     (unsigned long)param->constraints.uint_range.max);
            return ESP_ERR_INVALID_ARG;
        }
        
        if (param->type == CONFIG_TYPE_UINT32) {
            uint32_t* param_ptr = (uint32_t*)get_system_param_ptr(param);
            *param_ptr = value;
        } else { // CONFIG_TYPE_UINT16
            uint16_t* param_ptr = (uint16_t*)get_system_param_ptr(param);
            *param_ptr = (uint16_t)value;
        }
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %lu %s", namespace_str, param_name, 
                 (unsigned long)value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            return config_manager_save_namespace(CONFIG_NS_SYSTEM);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_get_float(const char* namespace_str, const char* param_name, float* value) {
    if (!namespace_str || !param_name || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_FLOAT) {
            return ESP_ERR_NOT_FOUND;
        }
        
        float* param_ptr = (float*)get_system_param_ptr(param);
        *value = *param_ptr;
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_set_float(const char* namespace_str, const char* param_name, float value, bool persist) {
    if (!namespace_str || !param_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_FLOAT) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Validate constraints
        if (value < param->constraints.float_range.min || value > param->constraints.float_range.max) {
            ESP_LOGE(TAG, "Value %.3f for %s.%s out of range [%.3f, %.3f]", 
                     value, namespace_str, param_name,
                     param->constraints.float_range.min, param->constraints.float_range.max);
            return ESP_ERR_INVALID_ARG;
        }
        
        float* param_ptr = (float*)get_system_param_ptr(param);
        *param_ptr = value;
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %.3f %s", namespace_str, param_name, 
                 value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            return config_manager_save_namespace(CONFIG_NS_SYSTEM);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_get_string(const char* namespace_str, const char* param_name, char* value, size_t max_len) {
    if (!namespace_str || !param_name || !value || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_STRING) {
            return ESP_ERR_NOT_FOUND;
        }
        
        char* param_ptr = (char*)get_system_param_ptr(param);
        strncpy(value, param_ptr, max_len - 1);
        value[max_len - 1] = '\0';
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hexapod_config_set_string(const char* namespace_str, const char* param_name, const char* value, bool persist) {
    if (!namespace_str || !param_name || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param || param->type != CONFIG_TYPE_STRING) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Validate length
        size_t value_len = strlen(value);
        if (value_len >= param->constraints.string.max_length) {
            ESP_LOGE(TAG, "String too long for %s.%s: %zu >= %zu", 
                     namespace_str, param_name, value_len, param->constraints.string.max_length);
            return ESP_ERR_INVALID_ARG;
        }
        
        char* param_ptr = (char*)get_system_param_ptr(param);
        strncpy(param_ptr, value, param->size - 1);
        param_ptr[param->size - 1] = '\0';
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = \"%s\" %s", namespace_str, param_name, 
                 value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            return config_manager_save_namespace(CONFIG_NS_SYSTEM);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

// =============================================================================
// Parameter Discovery and Metadata API
// =============================================================================

esp_err_t config_list_namespaces(const char** namespace_names, size_t* count) {
    if (!namespace_names || !count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (int i = 0; i < CONFIG_NS_COUNT; i++) {
        namespace_names[i] = CONFIG_NAMESPACE_NAMES[i];
    }
    
    *count = CONFIG_NS_COUNT;
    return ESP_OK;
}

esp_err_t config_list_parameters(const char* namespace_str, const char** param_names, 
                                size_t max_params, size_t* count) {
    if (!namespace_str || !param_names || !count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *count = 0;
    
    if (strcmp(namespace_str, "system") == 0) {
        size_t params_to_copy = (g_system_param_count < max_params) ? g_system_param_count : max_params;
        
        for (size_t i = 0; i < params_to_copy; i++) {
            param_names[i] = g_system_param_table[i].name;
        }
        
        *count = params_to_copy;
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t config_get_parameter_info(const char* namespace_str, const char* param_name, 
                                   config_param_info_t* info) {
    if (!namespace_str || !param_name || !info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(param_name);
        if (!param) {
            return ESP_ERR_NOT_FOUND;
        }
        
        memcpy(info, param, sizeof(config_param_info_t));
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
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
// Legacy Generic Parameter API (reimplemented using new system)
// =============================================================================

esp_err_t config_get_parameter(const char* namespace_str, const char* key, 
                               void* value_out, size_t value_size) {
    if (!namespace_str || !key || !value_out) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(key);
        if (!param) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Check buffer size
        if (value_size < param->size) {
            return ESP_ERR_INVALID_SIZE;
        }
        
        void* param_ptr = get_system_param_ptr(param);
        memcpy(value_out, param_ptr, param->size);
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t config_set_parameter(const char* namespace_str, const char* key,
                               const void* value, size_t value_size, bool persist) {
    if (!namespace_str || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(namespace_str, "system") == 0) {
        const config_param_info_t* param = find_system_param(key);
        if (!param) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Check value size
        if (value_size != param->size) {
            return ESP_ERR_INVALID_SIZE;
        }
        
        // Use type-specific functions for validation
        esp_err_t err = ESP_OK;
        switch (param->type) {
            case CONFIG_TYPE_BOOL:
                err = hexapod_config_set_bool(namespace_str, key, *(const bool*)value, persist);
                break;
            case CONFIG_TYPE_INT32:
                err = hexapod_config_set_int32(namespace_str, key, *(const int32_t*)value, persist);
                break;
            case CONFIG_TYPE_UINT32:
                err = hexapod_config_set_uint32(namespace_str, key, *(const uint32_t*)value, persist);
                break;
            case CONFIG_TYPE_UINT16:
                err = hexapod_config_set_uint32(namespace_str, key, *(const uint16_t*)value, persist);
                break;
            case CONFIG_TYPE_FLOAT:
                err = hexapod_config_set_float(namespace_str, key, *(const float*)value, persist);
                break;
            case CONFIG_TYPE_STRING:
                err = hexapod_config_set_string(namespace_str, key, (const char*)value, persist);
                break;
            default:
                err = ESP_ERR_NOT_SUPPORTED;
                break;
        }
        
        return err;
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