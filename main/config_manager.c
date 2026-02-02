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
#include "config_ns_system/system_namespace.h"
#include "config_ns_joint_cal/joint_cal_namespace.h"

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
    "system",
    "joint_cal"
};

// Global manager state
static config_manager_state_t g_manager_state = {0};

// Configuration cache - system namespace
static system_config_t g_system_config = {0};

// Configuration cache - joint calibration namespace
static joint_calib_config_t g_joint_calib_config = {0};

// System namespace keys moved to component `config_ns_system`

// =============================================================================
// NVS Parameter Keys for Joint Calibration Namespace
// =============================================================================

// Joint calibration parameter keys
// Format: leg{0-5}_{coxa|femur|tibia}_{parameter}
// We'll use leg indices 0-5 and joint names coxa, femur, tibia
#define JOINT_KEY_OFFSET_FORMAT      "l%d_%s_off"     // zero_offset_rad
#define JOINT_KEY_INVERT_FORMAT      "l%d_%s_inv"     // invert_sign
#define JOINT_KEY_MIN_FORMAT         "l%d_%s_min"     // min_rad
#define JOINT_KEY_MAX_FORMAT         "l%d_%s_max"     // max_rad
#define JOINT_KEY_PWM_MIN_FORMAT     "l%d_%s_pmin"    // pwm_min_us
#define JOINT_KEY_PWM_MAX_FORMAT     "l%d_%s_pmax"    // pwm_max_us
#define JOINT_KEY_NEUTRAL_FORMAT     "l%d_%s_neut"    // neutral_us

// Joint names for NVS key generation (shortened for key length limits)
static const char* JOINT_NAMES[] = {
    "c",     // LEG_SERVO_COXA = 0 (coxa)
    "f",     // LEG_SERVO_FEMUR = 1 (femur)
    "t"      // LEG_SERVO_TIBIA = 2 (tibia)
};

// Full joint names for user-facing parameter names
static const char* JOINT_FULL_NAMES[] = {
    "coxa",     // LEG_SERVO_COXA = 0
    "femur",    // LEG_SERVO_FEMUR = 1
    "tibia"     // LEG_SERVO_TIBIA = 2
};

// =============================================================================
// Joint Calibration Parameter Parsing Helpers
// =============================================================================

// Parse joint calibration parameter name like "leg2_femur_offset"
// Returns ESP_OK if valid, fills leg_index, joint_index, and param_type
typedef enum {
    JOINT_PARAM_OFFSET = 0,
    JOINT_PARAM_INVERT,
    JOINT_PARAM_MIN,
    JOINT_PARAM_MAX,
    JOINT_PARAM_PWM_MIN,
    JOINT_PARAM_PWM_MAX,
    JOINT_PARAM_NEUTRAL,
    JOINT_PARAM_INVALID
} joint_param_type_t;

static esp_err_t parse_joint_param_name(const char* param_name, int* leg_index, int* joint_index, joint_param_type_t* param_type) {
    if (!param_name || !leg_index || !joint_index || !param_type) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Expected format: "leg{0-5}_{coxa|femur|tibia}_{offset|invert|min|max|pwm_min|pwm_max|neutral}"
    int leg, joint_found = -1;
    char joint_name[16];
    char param_suffix[16];
    
    // Try to parse the pattern
    int parsed = sscanf(param_name, "leg%d_%15[^_]_%15s", &leg, joint_name, param_suffix);
    if (parsed != 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate leg index
    if (leg < 0 || leg >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find joint index (check both full and short names)
    for (int i = 0; i < NUM_JOINTS_PER_LEG; i++) {
        if (strcmp(joint_name, JOINT_FULL_NAMES[i]) == 0 || strcmp(joint_name, JOINT_NAMES[i]) == 0) {
            joint_found = i;
            break;
        }
    }
    if (joint_found < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find parameter type
    joint_param_type_t type = JOINT_PARAM_INVALID;
    if (strcmp(param_suffix, "offset") == 0) {
        type = JOINT_PARAM_OFFSET;
    } else if (strcmp(param_suffix, "invert") == 0) {
        type = JOINT_PARAM_INVERT;
    } else if (strcmp(param_suffix, "min") == 0) {
        type = JOINT_PARAM_MIN;
    } else if (strcmp(param_suffix, "max") == 0) {
        type = JOINT_PARAM_MAX;
    } else if (strcmp(param_suffix, "pwm_min") == 0) {
        type = JOINT_PARAM_PWM_MIN;
    } else if (strcmp(param_suffix, "pwm_max") == 0) {
        type = JOINT_PARAM_PWM_MAX;
    } else if (strcmp(param_suffix, "neutral") == 0) {
        type = JOINT_PARAM_NEUTRAL;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    
    *leg_index = leg;
    *joint_index = joint_found;
    *param_type = type;
    return ESP_OK;
}

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

// System init defaults moved to component (system_ns_init_defaults_nvs)

// =============================================================================
// Joint Calibration Default Values (Single Source of Truth)
// =============================================================================

/**
 * @brief Get default calibration values for a specific joint type
 * 
 * This is the single source of truth for joint calibration defaults.
 * Used by both memory defaults and NVS initialization functions.
 * 
 * @param joint Joint type (0=coxa, 1=femur, 2=tibia)
 * @param[out] calib Pointer to calibration structure to fill with defaults
 */
static void get_joint_calib_defaults(int joint, joint_calib_t* calib) {
    if (!calib || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return;
    }
    
    // Default offset (no offset)
    calib->zero_offset_rad = 0.0f;
    
    // Default invert sign: +1 (no inversion) for coxa, -1 for femur/tibia
    calib->invert_sign = (joint == 0) ? 1 : -1;
    
    // Default limits (conservative ±90 degrees)
    calib->min_rad = -1.57f;  // -π/2 radians
    calib->max_rad = 1.57f;   // +π/2 radians
    
    // Default PWM values (standard servo range)
    calib->pwm_min_us = 1000;   // 1ms
    calib->pwm_max_us = 2000;   // 2ms  
    calib->neutral_us = 1500;   // 1.5ms
}

static esp_err_t init_joint_calib_defaults_to_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_JOINT_CALIB];
    char key[32];
    
    ESP_LOGI(TAG, "Initializing joint calibration namespace defaults to NVS");
    
    // Write default values for all legs and joints
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
            const char* joint_name = JOINT_NAMES[joint];
            
            // Get default values from common function
            joint_calib_t default_calib;
            get_joint_calib_defaults(joint, &default_calib);
            
            // Write offset (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_OFFSET_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &default_calib.zero_offset_rad, sizeof(float)));
            
            // Write invert sign
            snprintf(key, sizeof(key), JOINT_KEY_INVERT_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i8(handle, key, default_calib.invert_sign));
            
            // Write min limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MIN_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &default_calib.min_rad, sizeof(float)));
            
            // Write max limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MAX_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &default_calib.max_rad, sizeof(float)));
            
            // Write PWM values
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MIN_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, default_calib.pwm_min_us));
            
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MAX_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, default_calib.pwm_max_us));
            
            snprintf(key, sizeof(key), JOINT_KEY_NEUTRAL_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, default_calib.neutral_us));
        }
    }

    ESP_ERROR_CHECK(nvs_commit(handle));
    ESP_LOGI(TAG, "Joint calibration defaults written to NVS");
    
    return ESP_OK;
}

static esp_err_t migrate_v0_to_v1(void) {
    ESP_LOGI(TAG, "Migrating v0 -> v1: Initializing fresh configuration");
    
    // Initialize all namespaces with defaults
    // Initialize system namespace defaults via component
    ESP_ERROR_CHECK(system_ns_init_defaults_nvs(g_manager_state.nvs_handles[CONFIG_NS_SYSTEM]));
    ESP_ERROR_CHECK(init_joint_calib_defaults_to_nvs());
    
    // Future: Add other namespace initialization here
    // ESP_ERROR_CHECK(init_motion_limits_defaults_to_nvs());
    
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

// System load moved to component (system_ns_load_from_nvs)

static esp_err_t load_joint_calib_config_from_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_JOINT_CALIB];
    esp_err_t err;
    char key[32];
    
    ESP_LOGI(TAG, "Loading joint calibration configuration from NVS (read-only)");
    
    // Load all legs and joints
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
            const char* joint_name = JOINT_NAMES[joint];
            joint_calib_t* calib = &g_joint_calib_config.joints[leg][joint];
            
            // Load offset (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_OFFSET_FORMAT, leg, joint_name);
            size_t required_size = sizeof(float);
            err = nvs_get_blob(handle, key, &calib->zero_offset_rad, &required_size);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load offset for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load invert sign
            snprintf(key, sizeof(key), JOINT_KEY_INVERT_FORMAT, leg, joint_name);
            err = nvs_get_i8(handle, key, &calib->invert_sign);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load invert for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load min limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MIN_FORMAT, leg, joint_name);
            required_size = sizeof(float);
            err = nvs_get_blob(handle, key, &calib->min_rad, &required_size);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load min limit for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load max limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MAX_FORMAT, leg, joint_name);
            required_size = sizeof(float);
            err = nvs_get_blob(handle, key, &calib->max_rad, &required_size);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load max limit for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load PWM min
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MIN_FORMAT, leg, joint_name);
            err = nvs_get_i32(handle, key, &calib->pwm_min_us);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load PWM min for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load PWM max
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MAX_FORMAT, leg, joint_name);
            err = nvs_get_i32(handle, key, &calib->pwm_max_us);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load PWM max for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
            
            // Load PWM neutral
            snprintf(key, sizeof(key), JOINT_KEY_NEUTRAL_FORMAT, leg, joint_name);
            err = nvs_get_i32(handle, key, &calib->neutral_us);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "Failed to load PWM neutral for leg %d joint %s: %s", leg, joint_name, esp_err_to_name(err));
            }
        }
    }

    g_manager_state.namespace_loaded[CONFIG_NS_JOINT_CALIB] = true;
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    
    ESP_LOGI(TAG, "Joint calibration config loaded");
    
    return ESP_OK;
}

// System save moved to component (system_ns_save_to_nvs)

static esp_err_t save_joint_calib_config_to_nvs(void) {
    nvs_handle_t handle = g_manager_state.nvs_handles[CONFIG_NS_JOINT_CALIB];
    esp_err_t err;
    char key[32];
    
    ESP_LOGI(TAG, "Saving joint calibration configuration to NVS");
    
    // Save all legs and joints
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
            const char* joint_name = JOINT_NAMES[joint];
            const joint_calib_t* calib = &g_joint_calib_config.joints[leg][joint];
            
            // Save offset (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_OFFSET_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &calib->zero_offset_rad, sizeof(float)));
            
            // Save invert sign
            snprintf(key, sizeof(key), JOINT_KEY_INVERT_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i8(handle, key, calib->invert_sign));
            
            // Save min limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MIN_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &calib->min_rad, sizeof(float)));
            
            // Save max limit (float as blob)
            snprintf(key, sizeof(key), JOINT_KEY_MAX_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_blob(handle, key, &calib->max_rad, sizeof(float)));
            
            // Save PWM min
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MIN_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, calib->pwm_min_us));
            
            // Save PWM max
            snprintf(key, sizeof(key), JOINT_KEY_PWM_MAX_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, calib->pwm_max_us));
            
            // Save PWM neutral
            snprintf(key, sizeof(key), JOINT_KEY_NEUTRAL_FORMAT, leg, joint_name);
            ESP_ERROR_CHECK(nvs_set_i32(handle, key, calib->neutral_us));
        }
    }

    // Commit changes
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit joint calibration config to NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    ESP_LOGI(TAG, "Joint calibration configuration saved successfully");
    
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

    // Register descriptors in core registry (Phase 4 minimal)
    extern esp_err_t system_ns_register_with_core(void);
    extern esp_err_t joint_cal_ns_register_with_core(void);
    system_ns_register_with_core();
    joint_cal_ns_register_with_core();
    
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
    const config_namespace_descriptor_t *joint_cal_desc = joint_cal_ns_get_descriptor();
    joint_cal_desc->load_defaults_mem(&g_joint_calib_config);
    
    // STEP 4: Load configurations from NVS (guaranteed correct schema now)
    // Load system config via component
    err = system_ns_load_from_nvs(g_manager_state.nvs_handles[CONFIG_NS_SYSTEM], &g_system_config);
    if (err == ESP_OK) {
        g_manager_state.namespace_loaded[CONFIG_NS_SYSTEM] = true;
        g_manager_state.namespace_dirty[CONFIG_NS_SYSTEM] = false;
        ESP_LOGI(TAG, "System config loaded - robot_id=%s, robot_name=%s, version=%d", 
                 g_system_config.robot_id, g_system_config.robot_name, g_system_config.config_version);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load system config from NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    extern esp_err_t joint_cal_ns_load_from_nvs(nvs_handle_t, joint_calib_config_t*);
    err = joint_cal_ns_load_from_nvs(g_manager_state.nvs_handles[CONFIG_NS_JOINT_CALIB], &g_joint_calib_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load joint calibration config from NVS: %s", esp_err_to_name(err));
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
            return system_ns_save_to_nvs(g_manager_state.nvs_handles[CONFIG_NS_SYSTEM], &g_system_config);
        
        case CONFIG_NS_JOINT_CALIB:
            return save_joint_calib_config_to_nvs();
        
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
// Joint Calibration Configuration API
// =============================================================================

const joint_calib_config_t* config_get_joint_calib(void) {
    return &g_joint_calib_config;
}

esp_err_t config_set_joint_calib(const joint_calib_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&g_joint_calib_config, config, sizeof(joint_calib_config_t));
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
    
    esp_err_t err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
    if (err == ESP_OK) {
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    }
    return err;
}

esp_err_t config_get_joint_calib_data(int leg_index, int joint, joint_calib_t* calib_data) {
    if (!calib_data || leg_index < 0 || leg_index >= NUM_LEGS || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *calib_data = g_joint_calib_config.joints[leg_index][joint];
    return ESP_OK;
}

esp_err_t config_set_joint_calib_data_memory(int leg_index, int joint, const joint_calib_t* calib_data) {
    if (!calib_data || leg_index < 0 || leg_index >= NUM_LEGS || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_joint_calib_config.joints[leg_index][joint] = *calib_data;
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
    
    return ESP_OK;
}

esp_err_t config_set_joint_calib_data_persist(int leg_index, int joint, const joint_calib_t* calib_data) {
    esp_err_t err = config_set_joint_calib_data_memory(leg_index, joint, calib_data);
    if (err != ESP_OK) {
        return err;
    }
    
    err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
    if (err == ESP_OK) {
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    }
    return err;
}

esp_err_t config_set_joint_offset_memory(int leg_index, int joint, float offset_rad) {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_joint_calib_config.joints[leg_index][joint].zero_offset_rad = offset_rad;
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
    
    return ESP_OK;
}

esp_err_t config_set_joint_offset_persist(int leg_index, int joint, float offset_rad) {
    esp_err_t err = config_set_joint_offset_memory(leg_index, joint, offset_rad);
    if (err != ESP_OK) {
        return err;
    }
    
    err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
    if (err == ESP_OK) {
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    }
    return err;
}

// Additional convenience functions for common joint calibration operations

esp_err_t config_set_joint_limits_memory(int leg_index, int joint, float min_rad, float max_rad) {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_joint_calib_config.joints[leg_index][joint].min_rad = min_rad;
    g_joint_calib_config.joints[leg_index][joint].max_rad = max_rad;
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
    
    return ESP_OK;
}

esp_err_t config_set_joint_limits_persist(int leg_index, int joint, float min_rad, float max_rad) {
    esp_err_t err = config_set_joint_limits_memory(leg_index, joint, min_rad, max_rad);
    if (err != ESP_OK) {
        return err;
    }
    
    err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
    if (err == ESP_OK) {
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    }
    return err;
}

esp_err_t config_set_joint_pwm_memory(int leg_index, int joint, int32_t pwm_min_us, int32_t pwm_max_us, int32_t pwm_neutral_us) {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint < 0 || joint >= NUM_JOINTS_PER_LEG) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_joint_calib_config.joints[leg_index][joint].pwm_min_us = pwm_min_us;
    g_joint_calib_config.joints[leg_index][joint].pwm_max_us = pwm_max_us;
    g_joint_calib_config.joints[leg_index][joint].neutral_us = pwm_neutral_us;
    g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
    
    return ESP_OK;
}

esp_err_t config_set_joint_pwm_persist(int leg_index, int joint, int32_t pwm_min_us, int32_t pwm_max_us, int32_t pwm_neutral_us) {
    esp_err_t err = config_set_joint_pwm_memory(leg_index, joint, pwm_min_us, pwm_max_us, pwm_neutral_us);
    if (err != ESP_OK) {
        return err;
    }
    
    err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
    if (err == ESP_OK) {
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = false;
    }
    return err;
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
        // .description = "Emergency stop functionality enabled",
        .constraints = { .int_range = { 0, 1 } }
    },
    {
        .name = "auto_disarm_timeout",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, auto_disarm_timeout),
        .size = sizeof(uint32_t),
        // .description = "Auto-disarm timeout in seconds",
        .constraints = { .uint_range = { 5, 300 } }
    },
    {
        .name = "safety_voltage_min",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(system_config_t, safety_voltage_min),
        .size = sizeof(float),
        // .description = "Minimum battery voltage in volts",
        .constraints = { .float_range = { 3.0f, 12.0f } }
    },
    {
        .name = "temperature_limit_max",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(system_config_t, temperature_limit_max),
        .size = sizeof(float),
        // .description = "Maximum operating temperature in Celsius",
        .constraints = { .float_range = { 40.0f, 100.0f } }
    },
    {
        .name = "motion_timeout_ms",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, motion_timeout_ms),
        .size = sizeof(uint32_t),
        // .description = "Motion command timeout in milliseconds",
        .constraints = { .uint_range = { 100, 5000 } }
    },
    {
        .name = "startup_delay_ms",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, startup_delay_ms),
        .size = sizeof(uint32_t),
        // .description = "Startup safety delay in milliseconds",
        .constraints = { .uint_range = { 0, 10000 } }
    },
    {
        .name = "max_control_frequency",
        .type = CONFIG_TYPE_UINT32,
        .offset = offsetof(system_config_t, max_control_frequency),
        .size = sizeof(uint32_t),
        // .description = "Maximum control loop frequency in Hz",
        .constraints = { .uint_range = { 50, 1000 } }
    },
    {
        .name = "robot_id",
        .type = CONFIG_TYPE_STRING,
        .offset = offsetof(system_config_t, robot_id),
        .size = sizeof(((system_config_t*)0)->robot_id),
        // .description = "Unique robot identifier",
        .constraints = { .string = { 32 } }
    },
    {
        .name = "robot_name",
        .type = CONFIG_TYPE_STRING,
        .offset = offsetof(system_config_t, robot_name),
        .size = sizeof(((system_config_t*)0)->robot_name),
        // .description = "Human-readable robot name",
        .constraints = { .string = { 64 } }
    },
    {
        .name = "config_version",
        .type = CONFIG_TYPE_UINT16,
        .offset = offsetof(system_config_t, config_version),
        .size = sizeof(uint16_t),
        // .description = "Configuration schema version",
        .constraints = { .uint_range = { 1, 65535 } }
    },
    {
        .name = "controller_type",
        .type = CONFIG_TYPE_UINT16, // Using UINT16 for enum
        .offset = offsetof(system_config_t, controller_type),
        .size = sizeof(controller_driver_type_e),
        // .description = "Default controller driver type",
        .constraints = { .uint_range = { 0, 4 } } // Corresponds to enum values
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
// Joint Calibration Parameter Table and Metadata
// =============================================================================

// Joint calibration parameter metadata for a single joint
typedef struct {
    const char* param_suffix;         // "offset", "invert", "min", etc.
    config_param_type_t type;
    size_t offset;                    // Offset within joint_calib_t structure
    size_t size;
    union {
        struct { int32_t min, max; } int_range;
        struct { uint32_t min, max; } uint_range;
        struct { float min, max; } float_range;
        struct { size_t max_length; } string;
    } constraints;
} joint_param_meta_t;

// Joint calibration parameter metadata table
static const joint_param_meta_t g_joint_param_table[] = {
    {
        .param_suffix = "offset",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(joint_calib_t, zero_offset_rad),
        .size = sizeof(float),
        // .description = "Mechanical zero offset in radians",
        .constraints = { .float_range = { -6.28f, 6.28f } } // ±2π radians
    },
    {
        .param_suffix = "invert", 
        .type = CONFIG_TYPE_INT32,
        .offset = offsetof(joint_calib_t, invert_sign),
        .size = sizeof(int8_t),
        // .description = "Direction inversion: +1 or -1",
        .constraints = { .int_range = { -1, 1 } }
    },
    {
        .param_suffix = "min",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(joint_calib_t, min_rad),
        .size = sizeof(float), 
        // .description = "Minimum joint angle in radians",
        .constraints = { .float_range = { -6.28f, 6.28f } }
    },
    {
        .param_suffix = "max",
        .type = CONFIG_TYPE_FLOAT,
        .offset = offsetof(joint_calib_t, max_rad),
        .size = sizeof(float),
        // .description = "Maximum joint angle in radians", 
        .constraints = { .float_range = { -6.28f, 6.28f } }
    },
    {
        .param_suffix = "pwm_min",
        .type = CONFIG_TYPE_INT32,
        .offset = offsetof(joint_calib_t, pwm_min_us),
        .size = sizeof(int32_t),
        // .description = "PWM pulse width at minimum angle (microseconds)",
        .constraints = { .int_range = { 500, 3000 } }
    },
    {
        .param_suffix = "pwm_max",
        .type = CONFIG_TYPE_INT32,
        .offset = offsetof(joint_calib_t, pwm_max_us),
        .size = sizeof(int32_t),
        // .description = "PWM pulse width at maximum angle (microseconds)",
        .constraints = { .int_range = { 500, 3000 } }
    },
    {
        .param_suffix = "neutral",
        .type = CONFIG_TYPE_INT32,
        .offset = offsetof(joint_calib_t, neutral_us),
        .size = sizeof(int32_t),
        // .description = "PWM pulse width at neutral position (microseconds)",
        .constraints = { .int_range = { 500, 3000 } }
    }
};

#define g_joint_param_count (sizeof(g_joint_param_table) / sizeof(g_joint_param_table[0]))

// Helper function to find joint parameter metadata by suffix
static const joint_param_meta_t* find_joint_param_meta(const char* param_suffix) {
    for (size_t i = 0; i < g_joint_param_count; i++) {
        if (strcmp(g_joint_param_table[i].param_suffix, param_suffix) == 0) {
            return &g_joint_param_table[i];
        }
    }
    return NULL;
}



// Helper function to build joint parameter info from parsed components
static esp_err_t build_joint_param_info(int leg_index, int joint_index, const char* param_suffix, 
                                        config_param_info_t* info) {
    if (!param_suffix || !info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const joint_param_meta_t* meta = find_joint_param_meta(param_suffix);
    if (!meta) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Use static buffer for parameter name (since info->name is const char*)
    static char param_name_buffer[32];
    snprintf(param_name_buffer, sizeof(param_name_buffer), "leg%d_%s_%s", 
             leg_index, JOINT_FULL_NAMES[joint_index], param_suffix);
    
    info->name = param_name_buffer;  // Point to static buffer
    info->type = meta->type;
    info->offset = meta->offset;  // Offset within joint_calib_t
    info->size = meta->size;
    
    // Copy constraints based on type
    switch (meta->type) {
        case CONFIG_TYPE_INT32:
            info->constraints.int_range.min = meta->constraints.int_range.min;
            info->constraints.int_range.max = meta->constraints.int_range.max;
            break;
        case CONFIG_TYPE_FLOAT:
            info->constraints.float_range.min = meta->constraints.float_range.min;
            info->constraints.float_range.max = meta->constraints.float_range.max;
            break;
        default:
            break;
    }
    
    return ESP_OK;
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        int leg_index, joint_index;
        joint_param_type_t param_type;
        esp_err_t err = parse_joint_param_name(param_name, &leg_index, &joint_index, &param_type);
        if (err != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        
        const joint_calib_t* calib = &g_joint_calib_config.joints[leg_index][joint_index];
        
        switch (param_type) {
            case JOINT_PARAM_INVERT:
                *value = calib->invert_sign;
                break;
            case JOINT_PARAM_PWM_MIN:
                *value = calib->pwm_min_us;
                break;
            case JOINT_PARAM_PWM_MAX:
                *value = calib->pwm_max_us;
                break;
            case JOINT_PARAM_NEUTRAL:
                *value = calib->neutral_us;
                break;
            default:
                return ESP_ERR_NOT_FOUND;  // Non-int32 parameter
        }
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        int leg_index, joint_index;
        joint_param_type_t param_type;
        esp_err_t err = parse_joint_param_name(param_name, &leg_index, &joint_index, &param_type);
        if (err != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        
        joint_calib_t* calib = &g_joint_calib_config.joints[leg_index][joint_index];
        
        // Basic validation
        switch (param_type) {
            case JOINT_PARAM_INVERT:
                if (value != -1 && value != 1) {
                    ESP_LOGE(TAG, "Invert value %ld for %s must be -1 or 1", (long)value, param_name);
                    return ESP_ERR_INVALID_ARG;
                }
                calib->invert_sign = (int8_t)value;
                break;
            case JOINT_PARAM_PWM_MIN:
                if (value < 500 || value > 3000) {  // Basic PWM range validation
                    ESP_LOGE(TAG, "PWM min value %ld for %s out of range [500, 3000] us", (long)value, param_name);
                    return ESP_ERR_INVALID_ARG;
                }
                calib->pwm_min_us = value;
                break;
            case JOINT_PARAM_PWM_MAX:
                if (value < 500 || value > 3000) {  // Basic PWM range validation
                    ESP_LOGE(TAG, "PWM max value %ld for %s out of range [500, 3000] us", (long)value, param_name);
                    return ESP_ERR_INVALID_ARG;
                }
                calib->pwm_max_us = value;
                break;
            case JOINT_PARAM_NEUTRAL:
                if (value < 500 || value > 3000) {  // Basic PWM range validation
                    ESP_LOGE(TAG, "PWM neutral value %ld for %s out of range [500, 3000] us", (long)value, param_name);
                    return ESP_ERR_INVALID_ARG;
                }
                calib->neutral_us = value;
                break;
            default:
                return ESP_ERR_NOT_FOUND;  // Non-int32 parameter
        }
        
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %ld %s", namespace_str, param_name, 
                 (long)value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            esp_err_t err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
            return err;
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        int leg_index, joint_index;
        joint_param_type_t param_type;
        esp_err_t err = parse_joint_param_name(param_name, &leg_index, &joint_index, &param_type);
        if (err != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        
        const joint_calib_t* calib = &g_joint_calib_config.joints[leg_index][joint_index];
        
        switch (param_type) {
            case JOINT_PARAM_OFFSET:
                *value = calib->zero_offset_rad;
                break;
            case JOINT_PARAM_MIN:
                *value = calib->min_rad;
                break;
            case JOINT_PARAM_MAX:
                *value = calib->max_rad;
                break;
            default:
                return ESP_ERR_NOT_FOUND;  // Non-float parameter
        }
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        int leg_index, joint_index;
        joint_param_type_t param_type;
        esp_err_t err = parse_joint_param_name(param_name, &leg_index, &joint_index, &param_type);
        if (err != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        
        joint_calib_t* calib = &g_joint_calib_config.joints[leg_index][joint_index];
        
        // Basic validation for angle parameters (radians)
        bool is_angle_param = (param_type == JOINT_PARAM_OFFSET || param_type == JOINT_PARAM_MIN || param_type == JOINT_PARAM_MAX);
        if (is_angle_param && (value < -6.28f || value > 6.28f)) {  // ±2π radians
            ESP_LOGE(TAG, "Angle value %.3f for %s out of range [-6.28, 6.28] radians", value, param_name);
            return ESP_ERR_INVALID_ARG;
        }
        
        switch (param_type) {
            case JOINT_PARAM_OFFSET:
                calib->zero_offset_rad = value;
                break;
            case JOINT_PARAM_MIN:
                calib->min_rad = value;
                break;
            case JOINT_PARAM_MAX:
                calib->max_rad = value;
                break;
            default:
                return ESP_ERR_NOT_FOUND;  // Non-float parameter
        }
        
        g_manager_state.namespace_dirty[CONFIG_NS_JOINT_CALIB] = true;
        
        ESP_LOGD(TAG, "Set %s.%s = %.3f %s", namespace_str, param_name, 
                 value, persist ? "(persistent)" : "(memory-only)");
        
        if (persist) {
            esp_err_t err = config_manager_save_namespace(CONFIG_NS_JOINT_CALIB);
            return err;
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
        ESP_LOGE(TAG, "Invalid arguments to config_list_parameters");
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        // Calculate total available parameters  
        size_t total_joint_params = NUM_LEGS * NUM_JOINTS_PER_LEG * g_joint_param_count;
        
        // Use static buffer for parameter name generation
        // Support up to 9 parameters as originally intended
        static char param_name_buffer[10][20];  // 10 parameters, 20 chars each (more than enough for 18-char names)
        const size_t MAX_STATIC_PARAMS = 9; // we reserve one slot for "MORE PARAMS..."
        
        // Limit to the smallest of: total available, caller's max, our static buffer size
        size_t params_to_copy = total_joint_params;
        if (params_to_copy > max_params) {
            params_to_copy = max_params;
        }
        if (params_to_copy > MAX_STATIC_PARAMS) {
            params_to_copy = MAX_STATIC_PARAMS;
        }
        
        size_t param_idx = 0;
        
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
                for (size_t meta_idx = 0; meta_idx < g_joint_param_count; meta_idx++) {
                    // Check if we've reached our limit
                    if (param_idx >= params_to_copy || param_idx >= MAX_STATIC_PARAMS) {
                        param_names[param_idx] = "MORE PARAMS...";
                        param_idx++;
                        goto done_generating;
                    }
                    
                    // Generate parameter name in static buffer using full names for user readability
                    snprintf(param_name_buffer[param_idx], sizeof(param_name_buffer[0]), 
                             "leg%d_%s_%s", leg, JOINT_FULL_NAMES[joint], g_joint_param_table[meta_idx].param_suffix);
                    
                    param_names[param_idx] = param_name_buffer[param_idx];
                    param_idx++;
                }
            }
        }
        
    done_generating:
        *count = param_idx;
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
    
    if (strcmp(namespace_str, "joint_cal") == 0) {
        // Parse joint calibration parameter name
        int leg_index, joint_index;
        joint_param_type_t param_type;
        esp_err_t err = parse_joint_param_name(param_name, &leg_index, &joint_index, &param_type);
        if (err != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        
        // Convert param_type to suffix string
        const char* param_suffix = NULL;
        switch (param_type) {
            case JOINT_PARAM_OFFSET: param_suffix = "offset"; break;
            case JOINT_PARAM_INVERT: param_suffix = "invert"; break;
            case JOINT_PARAM_MIN: param_suffix = "min"; break;
            case JOINT_PARAM_MAX: param_suffix = "max"; break;
            case JOINT_PARAM_PWM_MIN: param_suffix = "pwm_min"; break;
            case JOINT_PARAM_PWM_MAX: param_suffix = "pwm_max"; break;
            case JOINT_PARAM_NEUTRAL: param_suffix = "neutral"; break;
            default: return ESP_ERR_NOT_FOUND;
        }
        
        // Build parameter info from metadata
        return build_joint_param_info(leg_index, joint_index, param_suffix, info);
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
    
    // Default controller
    config->controller_type = CONTROLLER_DRIVER_FLYSKY_IBUS;

    ESP_LOGD(TAG, "Loaded system defaults - robot_id=%s", config->robot_id);
}

void config_load_joint_calib_defaults(joint_calib_config_t* config) {
    if (!config) {
        return;
    }
    
    memset(config, 0, sizeof(joint_calib_config_t));
    
    // Set default calibration for all legs and joints using common function
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
            joint_calib_t* calib = &config->joints[leg][joint];
            
            // Get defaults from single source of truth
            get_joint_calib_defaults(joint, calib);
        }
    }

    ESP_LOGD(TAG, "Loaded joint calibration defaults for %d legs, %d joints each", 
             NUM_LEGS, NUM_JOINTS_PER_LEG);
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
    config_load_joint_calib_defaults(&g_joint_calib_config);
    
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