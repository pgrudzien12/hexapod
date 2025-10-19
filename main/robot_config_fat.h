#ifndef ROBOT_CONFIG_FAT_H
#define ROBOT_CONFIG_FAT_H

#include "esp_err.h"
#include "robot_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration version for migration support
#define CONFIG_VERSION_CURRENT 1

// Mount point for configuration filesystem
#define CONFIG_MOUNT_POINT "/config"

// Configuration file paths
#define CONFIG_DIR_HARDWARE    "/config/hardware"
#define CONFIG_DIR_KINEMATICS  "/config/kinematics"
#define CONFIG_DIR_CALIBRATION "/config/calibration"
#define CONFIG_DIR_BEHAVIOR    "/config/behavior"
#define CONFIG_DIR_DEBUG       "/config/debug"

#define CONFIG_FILE_SYSTEM     "/config/system.json"
#define CONFIG_FILE_GPIO       "/config/hardware/gpio_mapping.json"
#define CONFIG_FILE_MCPWM      "/config/hardware/mcpwm_config.json"
#define CONFIG_FILE_DRIVERS    "/config/hardware/driver_selection.json"
#define CONFIG_FILE_GEOMETRY   "/config/kinematics/leg_geometry.json"
#define CONFIG_FILE_POSES      "/config/kinematics/mount_poses.json"
#define CONFIG_FILE_STANCE     "/config/kinematics/stance_params.json"
#define CONFIG_FILE_CONTROLLER "/config/controller.json"
#define CONFIG_FILE_BEHAVIOR   "/config/behavior/gait_params.json"
#define CONFIG_FILE_DEBUG      "/config/debug/logging_config.json"

// Per-leg calibration files (formatted with leg index)
#define CONFIG_FILE_LEG_CALIB_FMT "/config/calibration/leg_%d.json"

// System configuration structure (matches system.json)
typedef struct {
    uint32_t config_version;
    char robot_name[32];
    char firmware_version[16];
    uint64_t created_timestamp;
    uint64_t last_modified_timestamp;
    bool factory_reset_flag;
} system_config_t;

// Hardware configuration structures
typedef struct {
    int servo_gpio[NUM_LEGS][3];  // GPIO pins per leg/joint
} robot_gpio_config_t;

typedef struct {
    int mcpwm_group_id[NUM_LEGS]; // MCPWM group per leg
} robot_mcpwm_config_t;

typedef struct {
    int servo_driver_sel[NUM_LEGS][3]; // Driver selection per joint (0=MCPWM, 1=LEDC)
} robot_driver_config_t;

// Kinematics configuration structures
typedef struct {
    float leg_geometry[NUM_LEGS][3]; // coxa, femur, tibia lengths per leg
} robot_geometry_config_t;

typedef struct {
    float mount_poses[NUM_LEGS][4];  // x, y, z, yaw per leg
} robot_poses_config_t;

typedef struct {
    float stance_params[NUM_LEGS][2]; // outward, forward per leg
} robot_stance_config_t;

// Per-leg calibration structure (matches leg_X.json)
typedef struct {
    int leg_index;
    joint_calib_t joints[3];  // coxa, femur, tibia calibration
    char notes[128];          // User notes for this leg
    uint64_t last_calibrated_timestamp;
} robot_leg_calib_config_t;

// Controller configuration structure
typedef struct {
    uint8_t active_controller_type;   // 0=FLYSKY, 1=BT_CLASSIC, 2=WIFI_TCP
    char controller_name[32];
    // Driver-specific configs would be stored as additional JSON objects
} robot_controller_config_t;

// Behavior configuration structure
typedef struct {
    float gait_cycle_time_s;
    float swing_height_m;
    float step_length_m;
    uint8_t gait_type;        // 0=tripod, 1=wave, etc.
    char gait_name[32];
} robot_behavior_config_t;

// Debug configuration structure
typedef struct {
    uint32_t enable_flags;            // Bitmask for various debug features
    uint8_t log_level;                // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
    uint8_t monitor_leg_index;        // Which leg to monitor
    uint16_t log_interval_ms;         // Logging interval
    float delta_threshold_rad;        // Minimum delta to trigger logging
    bool console_output_enabled;
    bool file_logging_enabled;
} robot_debug_config_t;

// Reset levels for factory reset functionality
typedef enum {
    RESET_SOFT = 0,      // Behavior/debug settings only
    RESET_FULL = 1,      // All user config (preserve hardware mappings)
    RESET_FACTORY = 2    // Everything back to embedded defaults
} reset_level_t;

// ============================================================================
// Core FAT Configuration API
// ============================================================================

// Filesystem operations
esp_err_t robot_config_fat_init(void);
esp_err_t robot_config_fat_deinit(void);
esp_err_t robot_config_fat_format(void);

// Configuration loading and saving
esp_err_t robot_config_fat_load_all(void);
esp_err_t robot_config_fat_save_all(void);

// Individual file operations
esp_err_t robot_config_fat_load_system(system_config_t *system);
esp_err_t robot_config_fat_save_system(const system_config_t *system);

esp_err_t robot_config_fat_load_gpio(robot_gpio_config_t *gpio);
esp_err_t robot_config_fat_save_gpio(const robot_gpio_config_t *gpio);

esp_err_t robot_config_fat_load_mcpwm(robot_mcpwm_config_t *mcpwm);
esp_err_t robot_config_fat_save_mcpwm(const robot_mcpwm_config_t *mcpwm);

esp_err_t robot_config_fat_load_drivers(robot_driver_config_t *drivers);
esp_err_t robot_config_fat_save_drivers(const robot_driver_config_t *drivers);

esp_err_t robot_config_fat_load_geometry(robot_geometry_config_t *geometry);
esp_err_t robot_config_fat_save_geometry(const robot_geometry_config_t *geometry);

esp_err_t robot_config_fat_load_poses(robot_poses_config_t *poses);
esp_err_t robot_config_fat_save_poses(const robot_poses_config_t *poses);

esp_err_t robot_config_fat_load_stance(robot_stance_config_t *stance);
esp_err_t robot_config_fat_save_stance(const robot_stance_config_t *stance);

esp_err_t robot_config_fat_load_leg_calib(int leg_index, robot_leg_calib_config_t *leg_calib);
esp_err_t robot_config_fat_save_leg_calib(int leg_index, const robot_leg_calib_config_t *leg_calib);

esp_err_t robot_config_fat_load_controller(robot_controller_config_t *controller);
esp_err_t robot_config_fat_save_controller(const robot_controller_config_t *controller);

esp_err_t robot_config_fat_load_behavior(robot_behavior_config_t *behavior);
esp_err_t robot_config_fat_save_behavior(const robot_behavior_config_t *behavior);

esp_err_t robot_config_fat_load_debug(robot_debug_config_t *debug);
esp_err_t robot_config_fat_save_debug(const robot_debug_config_t *debug);

// Factory reset and validation
esp_err_t robot_config_fat_factory_reset(reset_level_t reset_level);
esp_err_t robot_config_fat_validate_config(void);
bool robot_config_fat_is_first_boot(void);

// File management utilities
esp_err_t robot_config_fat_create_directories(void);
esp_err_t robot_config_fat_list_files(void);
esp_err_t robot_config_fat_backup_config(const char *backup_path);
esp_err_t robot_config_fat_restore_config(const char *backup_path);

// JSON utilities
esp_err_t robot_config_fat_read_json_file(const char *filepath, char **json_string, size_t *json_size);
esp_err_t robot_config_fat_write_json_file(const char *filepath, const char *json_string);
esp_err_t robot_config_fat_ensure_file_exists(const char *filepath, const char *default_content);

// Migration support
esp_err_t robot_config_fat_migrate_config(uint32_t from_version, uint32_t to_version);

// Status and diagnostics
esp_err_t robot_config_fat_get_storage_info(size_t *total_bytes, size_t *used_bytes, size_t *free_bytes);
void robot_config_fat_print_file_tree(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_FAT_H