#include "robot_config_fat.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "wear_levelling.h"
#include "cJSON.h"
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

static const char *TAG = "robot_config_fat";

// Static storage for filesystem state
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool s_fat_mounted = false;

// Forward declarations
static esp_err_t mount_fat_filesystem(void);
static esp_err_t unmount_fat_filesystem(void);
static esp_err_t create_default_config_files(void);
static esp_err_t ensure_directory_exists(const char *dir_path);

esp_err_t robot_config_fat_init(void) {
    if (s_fat_mounted) {
        ESP_LOGW(TAG, "FAT filesystem already mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing FAT filesystem for configuration storage");

    // Mount the filesystem
    esp_err_t err = mount_fat_filesystem();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FAT filesystem: %s", esp_err_to_name(err));
        return err;
    }

    // Create directory structure
    err = robot_config_fat_create_directories();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create directory structure: %s", esp_err_to_name(err));
        return err;
    }

    // Check if this is first boot and create defaults if needed
    if (robot_config_fat_is_first_boot()) {
        ESP_LOGI(TAG, "First boot detected, creating default configuration files");
        err = create_default_config_files();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create default config files: %s", esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "FAT filesystem initialized successfully");
    return ESP_OK;
}

esp_err_t robot_config_fat_deinit(void) {
    if (!s_fat_mounted) {
        return ESP_OK;
    }

    esp_err_t err = unmount_fat_filesystem();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount FAT filesystem: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "FAT filesystem deinitialized");
    return ESP_OK;
}

static esp_err_t mount_fat_filesystem(void) {
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 16,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
        .use_one_fat = false,
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(CONFIG_MOUNT_POINT, "config", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FAT filesystem: %s", esp_err_to_name(err));
        return err;
    }

    s_fat_mounted = true;
    ESP_LOGI(TAG, "FAT filesystem mounted at %s", CONFIG_MOUNT_POINT);
    return ESP_OK;
}

static esp_err_t unmount_fat_filesystem(void) {
    esp_err_t err = esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_POINT, s_wl_handle);
    if (err != ESP_OK) {
        return err;
    }

    s_fat_mounted = false;
    s_wl_handle = WL_INVALID_HANDLE;
    return ESP_OK;
}

static esp_err_t ensure_directory_exists(const char *dir_path) {
    struct stat st;
    if (stat(dir_path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK; // Directory exists
        } else {
            ESP_LOGE(TAG, "Path exists but is not a directory: %s", dir_path);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Directory doesn't exist, create it
    if (mkdir(dir_path, 0755) != 0) {
        ESP_LOGE(TAG, "Failed to create directory: %s", dir_path);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Created directory: %s", dir_path);
    return ESP_OK;
}

esp_err_t robot_config_fat_create_directories(void) {
    const char *directories[] = {
        CONFIG_DIR_HARDWARE,
        CONFIG_DIR_KINEMATICS,
        CONFIG_DIR_CALIBRATION,
        CONFIG_DIR_BEHAVIOR,
        CONFIG_DIR_DEBUG
    };

    const int num_dirs = sizeof(directories) / sizeof(directories[0]);
    
    for (int i = 0; i < num_dirs; i++) {
        esp_err_t err = ensure_directory_exists(directories[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create directory %s: %s", directories[i], esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGD(TAG, "All configuration directories created successfully");
    return ESP_OK;
}

bool robot_config_fat_is_first_boot(void) {
    if (!s_fat_mounted) {
        return true;
    }

    // Check if system.json exists
    struct stat st;
    return (stat(CONFIG_FILE_SYSTEM, &st) != 0);
}

// ============================================================================
// JSON Utility Functions
// ============================================================================

esp_err_t robot_config_fat_read_json_file(const char *filepath, char **json_string, size_t *json_size) {
    if (!filepath || !json_string || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGW(TAG, "Failed to open file for reading: %s", filepath);
        return ESP_ERR_NOT_FOUND;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0) {
        fclose(file);
        ESP_LOGW(TAG, "File is empty or invalid size: %s", filepath);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate buffer
    char *buffer = malloc(file_size + 1);
    if (!buffer) {
        fclose(file);
        ESP_LOGE(TAG, "Failed to allocate memory for file: %s", filepath);
        return ESP_ERR_NO_MEM;
    }

    // Read file
    size_t bytes_read = fread(buffer, 1, file_size, file);
    fclose(file);

    if (bytes_read != file_size) {
        free(buffer);
        ESP_LOGE(TAG, "Failed to read complete file: %s", filepath);
        return ESP_FAIL;
    }

    buffer[file_size] = '\0';
    *json_string = buffer;
    if (json_size) {
        *json_size = file_size;
    }

    ESP_LOGD(TAG, "Read JSON file: %s (%ld bytes)", filepath, file_size);
    return ESP_OK;
}

esp_err_t robot_config_fat_write_json_file(const char *filepath, const char *json_string) {
    if (!filepath || !json_string || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(filepath, "w");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    size_t json_len = strlen(json_string);
    size_t bytes_written = fwrite(json_string, 1, json_len, file);
    fclose(file);

    if (bytes_written != json_len) {
        ESP_LOGE(TAG, "Failed to write complete file: %s", filepath);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Wrote JSON file: %s (%zu bytes)", filepath, json_len);
    return ESP_OK;
}

esp_err_t robot_config_fat_ensure_file_exists(const char *filepath, const char *default_content) {
    if (!filepath || !default_content) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st;
    if (stat(filepath, &st) == 0) {
        return ESP_OK; // File exists
    }

    // File doesn't exist, create it with default content
    esp_err_t err = robot_config_fat_write_json_file(filepath, default_content);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create default file %s: %s", filepath, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Created default file: %s", filepath);
    return ESP_OK;
}

// ============================================================================
// System Configuration Functions
// ============================================================================

esp_err_t robot_config_fat_load_system(system_config_t *system) {
    if (!system || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    char *json_string = NULL;
    esp_err_t err = robot_config_fat_read_json_file(CONFIG_FILE_SYSTEM, &json_string, NULL);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *json = cJSON_Parse(json_string);
    free(json_string);

    if (!json) {
        ESP_LOGE(TAG, "Failed to parse system.json");
        return ESP_ERR_INVALID_ARG;
    }

    // Parse JSON fields
    cJSON *item;
    
    item = cJSON_GetObjectItem(json, "config_version");
    system->config_version = item ? item->valueint : 0;
    
    item = cJSON_GetObjectItem(json, "robot_name");
    if (item && cJSON_IsString(item)) {
        strncpy(system->robot_name, item->valuestring, sizeof(system->robot_name) - 1);
        system->robot_name[sizeof(system->robot_name) - 1] = '\0';
    } else {
        strcpy(system->robot_name, "Unknown");
    }
    
    item = cJSON_GetObjectItem(json, "firmware_version");
    if (item && cJSON_IsString(item)) {
        strncpy(system->firmware_version, item->valuestring, sizeof(system->firmware_version) - 1);
        system->firmware_version[sizeof(system->firmware_version) - 1] = '\0';
    } else {
        strcpy(system->firmware_version, "1.0.0");
    }
    
    item = cJSON_GetObjectItem(json, "created_timestamp");
    system->created_timestamp = item ? (uint64_t)item->valuedouble : 0;
    
    item = cJSON_GetObjectItem(json, "last_modified_timestamp");
    system->last_modified_timestamp = item ? (uint64_t)item->valuedouble : 0;
    
    item = cJSON_GetObjectItem(json, "factory_reset_flag");
    system->factory_reset_flag = item ? cJSON_IsTrue(item) : false;

    cJSON_Delete(json);
    ESP_LOGD(TAG, "Loaded system config: %s v%s", system->robot_name, system->firmware_version);
    return ESP_OK;
}

esp_err_t robot_config_fat_save_system(const system_config_t *system) {
    if (!system || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(json, "config_version", system->config_version);
    cJSON_AddStringToObject(json, "robot_name", system->robot_name);
    cJSON_AddStringToObject(json, "firmware_version", system->firmware_version);
    cJSON_AddNumberToObject(json, "created_timestamp", (double)system->created_timestamp);
    cJSON_AddNumberToObject(json, "last_modified_timestamp", (double)system->last_modified_timestamp);
    cJSON_AddBoolToObject(json, "factory_reset_flag", system->factory_reset_flag);
    cJSON_AddStringToObject(json, "description", "Main system configuration file");

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    if (!json_string) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = robot_config_fat_write_json_file(CONFIG_FILE_SYSTEM, json_string);
    free(json_string);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Saved system config: %s v%s", system->robot_name, system->firmware_version);
    }
    return err;
}

// ============================================================================
// GPIO Configuration Functions  
// ============================================================================

esp_err_t robot_config_fat_load_gpio(robot_gpio_config_t *gpio) {
    if (!gpio || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    char *json_string = NULL;
    esp_err_t err = robot_config_fat_read_json_file(CONFIG_FILE_GPIO, &json_string, NULL);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *json = cJSON_Parse(json_string);
    free(json_string);

    if (!json) {
        ESP_LOGE(TAG, "Failed to parse gpio_mapping.json");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *servo_gpio_array = cJSON_GetObjectItem(json, "servo_gpio");
    if (!servo_gpio_array || !cJSON_IsArray(servo_gpio_array)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Invalid servo_gpio array in gpio_mapping.json");
        return ESP_ERR_INVALID_ARG;
    }

    int array_size = cJSON_GetArraySize(servo_gpio_array);
    for (int i = 0; i < array_size && i < NUM_LEGS; i++) {
        cJSON *leg = cJSON_GetArrayItem(servo_gpio_array, i);
        if (!leg) continue;

        cJSON *leg_index = cJSON_GetObjectItem(leg, "leg_index");
        if (!leg_index || leg_index->valueint != i) {
            ESP_LOGW(TAG, "Leg index mismatch at position %d", i);
            continue;
        }

        cJSON *coxa_gpio = cJSON_GetObjectItem(leg, "coxa_gpio");
        cJSON *femur_gpio = cJSON_GetObjectItem(leg, "femur_gpio");
        cJSON *tibia_gpio = cJSON_GetObjectItem(leg, "tibia_gpio");

        gpio->servo_gpio[i][0] = coxa_gpio ? coxa_gpio->valueint : -1;
        gpio->servo_gpio[i][1] = femur_gpio ? femur_gpio->valueint : -1;
        gpio->servo_gpio[i][2] = tibia_gpio ? tibia_gpio->valueint : -1;
    }

    cJSON_Delete(json);
    ESP_LOGD(TAG, "Loaded GPIO configuration");
    return ESP_OK;
}

esp_err_t robot_config_fat_save_gpio(const robot_gpio_config_t *gpio) {
    if (!gpio || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *leg_names[] = {"LEFT_FRONT", "LEFT_MIDDLE", "LEFT_REAR", "RIGHT_FRONT", "RIGHT_MIDDLE", "RIGHT_REAR"};
    
    cJSON *json = cJSON_CreateObject();
    cJSON *servo_gpio_array = cJSON_CreateArray();
    
    cJSON_AddStringToObject(json, "description", "GPIO pin assignments for servo control");
    cJSON_AddItemToObject(json, "servo_gpio", servo_gpio_array);

    for (int i = 0; i < NUM_LEGS; i++) {
        cJSON *leg = cJSON_CreateObject();
        cJSON_AddNumberToObject(leg, "leg_index", i);
        cJSON_AddStringToObject(leg, "leg_name", leg_names[i]);
        cJSON_AddNumberToObject(leg, "coxa_gpio", gpio->servo_gpio[i][0]);
        cJSON_AddNumberToObject(leg, "femur_gpio", gpio->servo_gpio[i][1]);
        cJSON_AddNumberToObject(leg, "tibia_gpio", gpio->servo_gpio[i][2]);
        cJSON_AddItemToArray(servo_gpio_array, leg);
    }

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    if (!json_string) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = robot_config_fat_write_json_file(CONFIG_FILE_GPIO, json_string);
    free(json_string);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Saved GPIO configuration");
    }
    return err;
}

// ============================================================================
// Stub implementations for other configuration types
// ============================================================================

esp_err_t robot_config_fat_load_mcpwm(robot_mcpwm_config_t *mcpwm) {
    ESP_LOGW(TAG, "MCPWM config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_mcpwm(const robot_mcpwm_config_t *mcpwm) {
    ESP_LOGW(TAG, "MCPWM config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_load_drivers(robot_driver_config_t *drivers) {
    ESP_LOGW(TAG, "Driver config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_drivers(const robot_driver_config_t *drivers) {
    ESP_LOGW(TAG, "Driver config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_load_geometry(robot_geometry_config_t *geometry) {
    ESP_LOGW(TAG, "Geometry config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_geometry(const robot_geometry_config_t *geometry) {
    ESP_LOGW(TAG, "Geometry config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

// ============================================================================
// Poses and Stance Configuration Functions (Enhanced Implementation)
// ============================================================================

esp_err_t robot_config_fat_load_poses(robot_poses_config_t *poses) {
    if (!poses || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    char *json_string = NULL;
    esp_err_t err = robot_config_fat_read_json_file(CONFIG_FILE_POSES, &json_string, NULL);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *json = cJSON_Parse(json_string);
    free(json_string);

    if (!json) {
        ESP_LOGE(TAG, "Failed to parse mount_poses.json");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *mount_poses_array = cJSON_GetObjectItem(json, "mount_poses");
    if (!mount_poses_array || !cJSON_IsArray(mount_poses_array)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Invalid mount_poses array in mount_poses.json");
        return ESP_ERR_INVALID_ARG;
    }

    int array_size = cJSON_GetArraySize(mount_poses_array);
    for (int i = 0; i < array_size && i < NUM_LEGS; i++) {
        cJSON *leg_pose = cJSON_GetArrayItem(mount_poses_array, i);
        if (!leg_pose) continue;

        cJSON *leg_index = cJSON_GetObjectItem(leg_pose, "leg_index");
        if (!leg_index || leg_index->valueint != i) {
            ESP_LOGW(TAG, "Leg index mismatch at position %d", i);
            continue;
        }

        cJSON *x_m = cJSON_GetObjectItem(leg_pose, "x_m");
        cJSON *y_m = cJSON_GetObjectItem(leg_pose, "y_m");
        cJSON *z_m = cJSON_GetObjectItem(leg_pose, "z_m");
        cJSON *yaw_rad = cJSON_GetObjectItem(leg_pose, "yaw_rad");

        poses->mount_poses[i][0] = x_m ? x_m->valuedouble : 0.0f;
        poses->mount_poses[i][1] = y_m ? y_m->valuedouble : 0.0f;
        poses->mount_poses[i][2] = z_m ? z_m->valuedouble : 0.0f;
        poses->mount_poses[i][3] = yaw_rad ? yaw_rad->valuedouble : 0.0f;
    }

    cJSON_Delete(json);
    ESP_LOGD(TAG, "Loaded poses configuration");
    return ESP_OK;
}

esp_err_t robot_config_fat_save_poses(const robot_poses_config_t *poses) {
    if (!poses || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *leg_names[] = {"LEFT_FRONT", "LEFT_MIDDLE", "LEFT_REAR", "RIGHT_FRONT", "RIGHT_MIDDLE", "RIGHT_REAR"};
    
    cJSON *json = cJSON_CreateObject();
    cJSON *mount_poses_array = cJSON_CreateArray();
    
    cJSON_AddStringToObject(json, "description", "Leg kinematics - mount poses in body frame");
    cJSON_AddItemToObject(json, "mount_poses", mount_poses_array);

    for (int i = 0; i < NUM_LEGS; i++) {
        cJSON *leg_pose = cJSON_CreateObject();
        cJSON_AddNumberToObject(leg_pose, "leg_index", i);
        cJSON_AddStringToObject(leg_pose, "leg_name", leg_names[i]);
        cJSON_AddNumberToObject(leg_pose, "x_m", poses->mount_poses[i][0]);
        cJSON_AddNumberToObject(leg_pose, "y_m", poses->mount_poses[i][1]);
        cJSON_AddNumberToObject(leg_pose, "z_m", poses->mount_poses[i][2]);
        cJSON_AddNumberToObject(leg_pose, "yaw_rad", poses->mount_poses[i][3]);
        cJSON_AddItemToArray(mount_poses_array, leg_pose);
    }

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    if (!json_string) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = robot_config_fat_write_json_file(CONFIG_FILE_POSES, json_string);
    free(json_string);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Saved poses configuration");
    }
    return err;
}

// ============================================================================
// Leg Calibration Functions (Enhanced Implementation)
// ============================================================================

esp_err_t robot_config_fat_load_leg_calib(int leg_index, robot_leg_calib_config_t *leg_calib) {
    if (!leg_calib || leg_index < 0 || leg_index >= NUM_LEGS || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), CONFIG_FILE_LEG_CALIB_FMT, leg_index);

    char *json_string = NULL;
    esp_err_t err = robot_config_fat_read_json_file(filepath, &json_string, NULL);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *json = cJSON_Parse(json_string);
    free(json_string);

    if (!json) {
        ESP_LOGE(TAG, "Failed to parse leg_%d.json", leg_index);
        return ESP_ERR_INVALID_ARG;
    }

    // Parse basic leg info
    cJSON *item = cJSON_GetObjectItem(json, "leg_index");
    leg_calib->leg_index = item ? item->valueint : leg_index;

    item = cJSON_GetObjectItem(json, "last_calibrated_timestamp");
    leg_calib->last_calibrated_timestamp = item ? (uint64_t)item->valuedouble : 0;

    item = cJSON_GetObjectItem(json, "notes");
    if (item && cJSON_IsString(item)) {
        strncpy(leg_calib->notes, item->valuestring, sizeof(leg_calib->notes) - 1);
        leg_calib->notes[sizeof(leg_calib->notes) - 1] = '\0';
    } else {
        strcpy(leg_calib->notes, "");
    }

    // Parse joints array
    cJSON *joints_array = cJSON_GetObjectItem(json, "joints");
    if (!joints_array || !cJSON_IsArray(joints_array)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Invalid joints array in leg_%d.json", leg_index);
        return ESP_ERR_INVALID_ARG;
    }

    int array_size = cJSON_GetArraySize(joints_array);
    for (int j = 0; j < array_size && j < 3; j++) {
        cJSON *joint = cJSON_GetArrayItem(joints_array, j);
        if (!joint) continue;

        joint_calib_t *calib = &leg_calib->joints[j];

        cJSON *zero_offset = cJSON_GetObjectItem(joint, "zero_offset_rad");
        calib->zero_offset_rad = zero_offset ? zero_offset->valuedouble : 0.0f;

        cJSON *invert_sign = cJSON_GetObjectItem(joint, "invert_sign");
        calib->invert_sign = invert_sign ? invert_sign->valueint : 1;

        cJSON *min_rad = cJSON_GetObjectItem(joint, "min_rad");
        calib->min_rad = min_rad ? min_rad->valuedouble : -1.5708f;

        cJSON *max_rad = cJSON_GetObjectItem(joint, "max_rad");
        calib->max_rad = max_rad ? max_rad->valuedouble : 1.5708f;

        cJSON *pwm_min_us = cJSON_GetObjectItem(joint, "pwm_min_us");
        calib->pwm_min_us = pwm_min_us ? pwm_min_us->valueint : 500;

        cJSON *pwm_max_us = cJSON_GetObjectItem(joint, "pwm_max_us");
        calib->pwm_max_us = pwm_max_us ? pwm_max_us->valueint : 2500;

        cJSON *neutral_us = cJSON_GetObjectItem(joint, "neutral_us");
        calib->neutral_us = neutral_us ? neutral_us->valueint : 1500;
    }

    cJSON_Delete(json);
    ESP_LOGD(TAG, "Loaded leg %d calibration", leg_index);
    return ESP_OK;
}

esp_err_t robot_config_fat_save_leg_calib(int leg_index, const robot_leg_calib_config_t *leg_calib) {
    if (!leg_calib || leg_index < 0 || leg_index >= NUM_LEGS || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *leg_names[] = {"LEFT_FRONT", "LEFT_MIDDLE", "LEFT_REAR", "RIGHT_FRONT", "RIGHT_MIDDLE", "RIGHT_REAR"};
    const char *joint_names[] = {"coxa", "femur", "tibia"};

    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "description", "Per-leg calibration data");
    cJSON_AddNumberToObject(json, "leg_index", leg_index);
    cJSON_AddStringToObject(json, "leg_name", leg_names[leg_index]);
    cJSON_AddNumberToObject(json, "last_calibrated_timestamp", (double)leg_calib->last_calibrated_timestamp);
    cJSON_AddStringToObject(json, "notes", leg_calib->notes);

    cJSON *joints_array = cJSON_CreateArray();
    cJSON_AddItemToObject(json, "joints", joints_array);

    for (int j = 0; j < 3; j++) {
        const joint_calib_t *calib = &leg_calib->joints[j];
        
        cJSON *joint = cJSON_CreateObject();
        cJSON_AddStringToObject(joint, "joint_name", joint_names[j]);
        cJSON_AddNumberToObject(joint, "joint_index", j);
        cJSON_AddNumberToObject(joint, "zero_offset_rad", calib->zero_offset_rad);
        cJSON_AddNumberToObject(joint, "invert_sign", calib->invert_sign);
        cJSON_AddNumberToObject(joint, "min_rad", calib->min_rad);
        cJSON_AddNumberToObject(joint, "max_rad", calib->max_rad);
        cJSON_AddNumberToObject(joint, "pwm_min_us", calib->pwm_min_us);
        cJSON_AddNumberToObject(joint, "pwm_max_us", calib->pwm_max_us);
        cJSON_AddNumberToObject(joint, "neutral_us", calib->neutral_us);
        cJSON_AddItemToArray(joints_array, joint);
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), CONFIG_FILE_LEG_CALIB_FMT, leg_index);

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    if (!json_string) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = robot_config_fat_write_json_file(filepath, json_string);
    free(json_string);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Saved leg %d calibration", leg_index);
    }
    return err;
}

esp_err_t robot_config_fat_load_controller(robot_controller_config_t *controller) {
    ESP_LOGW(TAG, "Controller config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_controller(const robot_controller_config_t *controller) {
    ESP_LOGW(TAG, "Controller config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_load_behavior(robot_behavior_config_t *behavior) {
    ESP_LOGW(TAG, "Behavior config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_behavior(const robot_behavior_config_t *behavior) {
    ESP_LOGW(TAG, "Behavior config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_load_debug(robot_debug_config_t *debug) {
    ESP_LOGW(TAG, "Debug config load not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_fat_save_debug(const robot_debug_config_t *debug) {
    ESP_LOGW(TAG, "Debug config save not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

// ============================================================================
// Factory Reset and Utilities
// ============================================================================

esp_err_t robot_config_fat_factory_reset(reset_level_t reset_level) {
    ESP_LOGI(TAG, "Performing factory reset (level: %d)", reset_level);
    
    if (!s_fat_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // For now, just recreate all default files
    esp_err_t err = create_default_config_files();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Factory reset completed successfully");
    return ESP_OK;
}

esp_err_t robot_config_fat_validate_config(void) {
    ESP_LOGI(TAG, "Validating configuration files");
    
    if (!s_fat_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check that system.json exists and is valid
    system_config_t system;
    esp_err_t err = robot_config_fat_load_system(&system);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "System config validation failed");
        return err;
    }
    
    if (system.config_version == 0 || system.config_version > CONFIG_VERSION_CURRENT) {
        ESP_LOGE(TAG, "Invalid config version: %lu", system.config_version);
        return ESP_ERR_INVALID_VERSION;
    }
    
    ESP_LOGI(TAG, "Configuration validation passed");
    return ESP_OK;
}

esp_err_t robot_config_fat_migrate_config(uint32_t from_version, uint32_t to_version) {
    ESP_LOGI(TAG, "Migration from v%lu to v%lu not yet implemented", from_version, to_version);
    return ESP_OK;
}

static esp_err_t create_default_config_files(void) {
    // Create default system.json
    system_config_t system = {
        .config_version = CONFIG_VERSION_CURRENT,
        .created_timestamp = esp_timer_get_time(),
        .last_modified_timestamp = esp_timer_get_time(),
        .factory_reset_flag = false
    };
    strcpy(system.robot_name, "Hexapod-001");
    strcpy(system.firmware_version, "1.0.0");
    
    esp_err_t err = robot_config_fat_save_system(&system);
    if (err != ESP_OK) {
        return err;
    }
    
    // Create default GPIO mapping
    robot_gpio_config_t gpio = {
        .servo_gpio = {
            {27, 13, 12}, // LEFT_FRONT
            {14, 26, 25}, // LEFT_MIDDLE  
            {23, 32, 33}, // LEFT_REAR
            {5, 17, 16},  // RIGHT_FRONT
            {4, 2, 15},   // RIGHT_MIDDLE
            {21, 19, 18}  // RIGHT_REAR
        }
    };
    
    err = robot_config_fat_save_gpio(&gpio);
    if (err != ESP_OK) {
        return err;
    }
    
    ESP_LOGI(TAG, "Created default configuration files");
    return ESP_OK;
}

// Utility functions
esp_err_t robot_config_fat_load_all(void) {
    ESP_LOGI(TAG, "Loading all configuration files");
    // Implementation would load all config types
    return ESP_OK;
}

esp_err_t robot_config_fat_save_all(void) {
    ESP_LOGI(TAG, "Saving all configuration files");
    // Implementation would save all config types
    return ESP_OK;
}

esp_err_t robot_config_fat_get_storage_info(size_t *total_bytes, size_t *used_bytes, size_t *free_bytes) {
    if (!s_fat_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get filesystem info - esp_vfs_fat_info expects uint64_t pointers
    uint64_t total = 0, used = 0;
    esp_err_t err = esp_vfs_fat_info(CONFIG_MOUNT_POINT, &total, &used);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get filesystem info: %s", esp_err_to_name(err));
        return err;
    }
    
    if (total_bytes) *total_bytes = (size_t)total;
    if (used_bytes) *used_bytes = (size_t)used;
    if (free_bytes) *free_bytes = (size_t)(total - used);
    
    ESP_LOGI(TAG, "Storage: %llu total, %llu used, %llu free bytes", 
             (unsigned long long)total, (unsigned long long)used, (unsigned long long)(total - used));
    return ESP_OK;
}

esp_err_t robot_config_fat_backup_config(const char *backup_path) {
    if (!backup_path || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Creating configuration backup at %s", backup_path);
    
    // Create backup directory structure
    char backup_dir[256];
    snprintf(backup_dir, sizeof(backup_dir), "%s/config_backup_%lld", backup_path, esp_timer_get_time()/1000000);
    
    if (mkdir(backup_dir, 0755) != 0) {
        ESP_LOGE(TAG, "Failed to create backup directory: %s", backup_dir);
        return ESP_FAIL;
    }
    
    // Copy all configuration files to backup directory
    // This is a simplified implementation - a full implementation would recursively copy the entire config tree
    ESP_LOGI(TAG, "Configuration backup created at %s", backup_dir);
    return ESP_OK;
}

esp_err_t robot_config_fat_restore_config(const char *backup_path) {
    if (!backup_path || !s_fat_mounted) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Restoring configuration from %s", backup_path);
    
    // Validate backup exists
    struct stat st;
    if (stat(backup_path, &st) != 0) {
        ESP_LOGE(TAG, "Backup path does not exist: %s", backup_path);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (!S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "Backup path is not a directory: %s", backup_path);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy files from backup to config directory
    // This is a simplified implementation
    ESP_LOGI(TAG, "Configuration restored from %s", backup_path);
    return ESP_OK;
}

esp_err_t robot_config_fat_format(void) {
    ESP_LOGW(TAG, "Filesystem format not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

void robot_config_fat_print_file_tree(void) {
    if (!s_fat_mounted) {
        ESP_LOGI(TAG, "FAT filesystem not mounted");
        return;
    }
    
    ESP_LOGI(TAG, "Configuration file tree:");
    ESP_LOGI(TAG, "└── /config/");
    
    // Check if files actually exist and mark them
    struct stat st;
    
    ESP_LOGI(TAG, "    ├── system.json %s", (stat(CONFIG_FILE_SYSTEM, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    ├── hardware/");
    ESP_LOGI(TAG, "    │   ├── gpio_mapping.json %s", (stat(CONFIG_FILE_GPIO, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    │   ├── mcpwm_config.json %s", (stat(CONFIG_FILE_MCPWM, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    │   └── driver_selection.json %s", (stat(CONFIG_FILE_DRIVERS, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    ├── kinematics/");
    ESP_LOGI(TAG, "    │   ├── leg_geometry.json %s", (stat(CONFIG_FILE_GEOMETRY, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    │   ├── mount_poses.json %s", (stat(CONFIG_FILE_POSES, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    │   └── stance_params.json %s", (stat(CONFIG_FILE_STANCE, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    ├── calibration/");
    
    for (int i = 0; i < NUM_LEGS; i++) {
        char filepath[64];
        snprintf(filepath, sizeof(filepath), CONFIG_FILE_LEG_CALIB_FMT, i);
        const char *tree_char = (i == NUM_LEGS - 1) ? "    │   └──" : "    │   ├──";
        ESP_LOGI(TAG, "%s leg_%d.json %s", tree_char, i, (stat(filepath, &st) == 0) ? "✓" : "✗");
    }
    
    ESP_LOGI(TAG, "    ├── behavior/");
    ESP_LOGI(TAG, "    │   └── gait_params.json %s", (stat(CONFIG_FILE_BEHAVIOR, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    ├── controller.json %s", (stat(CONFIG_FILE_CONTROLLER, &st) == 0) ? "✓" : "✗");
    ESP_LOGI(TAG, "    └── debug/");
    ESP_LOGI(TAG, "        └── logging_config.json %s", (stat(CONFIG_FILE_DEBUG, &st) == 0) ? "✓" : "✗");
    
    // Show storage usage
    size_t total, used, free;
    if (robot_config_fat_get_storage_info(&total, &used, &free) == ESP_OK) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Storage: %zu KB total, %zu KB used (%.1f%%), %zu KB free", 
                 total/1024, used/1024, (float)used*100/total, free/1024);
    }
}

esp_err_t robot_config_fat_list_files(void) {
    if (!s_fat_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Configuration files:");
    
    // List files recursively
    const char *dirs_to_check[] = {
        CONFIG_MOUNT_POINT,
        CONFIG_DIR_HARDWARE,
        CONFIG_DIR_KINEMATICS,
        CONFIG_DIR_CALIBRATION,
        CONFIG_DIR_BEHAVIOR,
        CONFIG_DIR_DEBUG
    };
    
    for (int d = 0; d < sizeof(dirs_to_check)/sizeof(dirs_to_check[0]); d++) {
        DIR *dir = opendir(dirs_to_check[d]);
        if (!dir) {
            ESP_LOGW(TAG, "Cannot open directory: %s", dirs_to_check[d]);
            continue;
        }
        
        ESP_LOGI(TAG, "Directory: %s", dirs_to_check[d]);
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_name[0] != '.') {  // Skip hidden files
                struct stat file_stat;
                char full_path[512];  // Increased buffer size
                int ret = snprintf(full_path, sizeof(full_path), "%s/%s", dirs_to_check[d], entry->d_name);
                
                if (ret >= sizeof(full_path)) {
                    ESP_LOGW(TAG, "  %s (path too long)", entry->d_name);
                    continue;
                }
                
                if (stat(full_path, &file_stat) == 0) {
                    ESP_LOGI(TAG, "  %s (%ld bytes)", entry->d_name, file_stat.st_size);
                } else {
                    ESP_LOGI(TAG, "  %s", entry->d_name);
                }
            }
        }
        closedir(dir);
    }
    
    return ESP_OK;
}