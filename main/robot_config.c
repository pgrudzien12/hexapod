#include "robot_config.h"
#include "robot_config_fat.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "robot_config";

// Single static instance for now. In the future, load/save to NVS.
static robot_config_t g_cfg;
static float g_base_x[NUM_LEGS];
static float g_base_y[NUM_LEGS];
static float g_base_z[NUM_LEGS];
static float g_base_yaw[NUM_LEGS];
static float g_stance_out[NUM_LEGS]; // leg-local outward (+)
static float g_stance_fwd[NUM_LEGS]; // leg-local forward (+)

// NVS integration state
static bool g_config_dirty = false;      // True if config needs to be saved
static bool g_fat_available = false;     // True if FAT filesystem is initialized and available

void robot_config_init_default(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    
    // Try to load from FAT filesystem first
    esp_err_t fat_err = robot_config_load_from_nvs();  // Note: function name kept for backward compatibility
    if (fat_err == ESP_OK) {
        ESP_LOGI(TAG, "Configuration loaded from FAT filesystem successfully");
        return;  // Successfully loaded from FAT, we're done
    } else {
        ESP_LOGW(TAG, "Failed to load from FAT filesystem (%s), using hardcoded defaults", esp_err_to_name(fat_err));
    }
    
    // If NVS loading failed, proceed with hardcoded defaults
    // Default: no GPIOs assigned yet; distribute legs across two MCPWM groups (0 for legs 0..2, 1 for legs 3..5)
    for (int i = 0; i < NUM_LEGS; ++i) {
        g_cfg.mcpwm_group_id[i] = (i < 3) ? 0 : 1;
        for (int j = 0; j < 3; ++j) g_cfg.servo_gpio[i][j] = -1;
        for (int j = 0; j < 3; ++j) g_cfg.servo_driver_sel[i][j] = 0; // default MCPWM
    }
    // Rebalance within first three legs: move left-rear leg to group 1 so group 0 only has two legs (6 channels)
    g_cfg.mcpwm_group_id[LEG_LEFT_REAR] = 1;

    // GPIO assignments by leg index enum

    g_cfg.servo_gpio[LEG_LEFT_FRONT][LEG_SERVO_COXA]  = 27;
    g_cfg.servo_gpio[LEG_LEFT_FRONT][LEG_SERVO_FEMUR] = 13;
    g_cfg.servo_gpio[LEG_LEFT_FRONT][LEG_SERVO_TIBIA] = 12;

    g_cfg.servo_gpio[LEG_LEFT_MIDDLE][LEG_SERVO_COXA]  = 14;
    g_cfg.servo_gpio[LEG_LEFT_MIDDLE][LEG_SERVO_FEMUR] = 26;
    g_cfg.servo_gpio[LEG_LEFT_MIDDLE][LEG_SERVO_TIBIA] = 25;

    g_cfg.servo_gpio[LEG_LEFT_REAR][LEG_SERVO_COXA]  = 23;
    g_cfg.servo_gpio[LEG_LEFT_REAR][LEG_SERVO_FEMUR] = 32;
    g_cfg.servo_gpio[LEG_LEFT_REAR][LEG_SERVO_TIBIA] = 33;

    g_cfg.servo_gpio[LEG_RIGHT_FRONT][LEG_SERVO_COXA]  = 5;
    g_cfg.servo_gpio[LEG_RIGHT_FRONT][LEG_SERVO_FEMUR] = 17;
    g_cfg.servo_gpio[LEG_RIGHT_FRONT][LEG_SERVO_TIBIA] = 16;

    g_cfg.servo_gpio[LEG_RIGHT_MIDDLE][LEG_SERVO_COXA]  = 4;
    g_cfg.servo_gpio[LEG_RIGHT_MIDDLE][LEG_SERVO_FEMUR] = 2;
    g_cfg.servo_gpio[LEG_RIGHT_MIDDLE][LEG_SERVO_TIBIA] = 15;

    g_cfg.servo_gpio[LEG_RIGHT_REAR][LEG_SERVO_COXA]  = 21;
    g_cfg.servo_gpio[LEG_RIGHT_REAR][LEG_SERVO_FEMUR] = 19;
    g_cfg.servo_gpio[LEG_RIGHT_REAR][LEG_SERVO_TIBIA] = 18;

    // Driver selection: choose LEDC for the three joints of RIGHT_MIDDLE (leg 4) and LEFT_REAR (leg 2) as example.
    // Adjust to match pins that are LEDC-friendly if some MCPWM pins (like 32/33/34/35) cause LEDC invalid gpio errors.
    // For now we move LEDC usage away from GPIO 32/35 which are input-only or restricted on classic ESP32.
    for (int j = 0; j < 3; ++j) {
        g_cfg.servo_driver_sel[LEG_RIGHT_MIDDLE][j] = 1; // LEDC
        g_cfg.servo_driver_sel[LEG_RIGHT_REAR][j] = 1;   // LEDC (example second leg)
    }

    // Default geometry for all 6 legs.
    // NOTE: Units must match usage across the project. Our swing_trajectory uses meters,
    // so we set lengths in meters as placeholders.
    // TODO(ESP-Storage): Replace with values loaded from storage per leg.
    const leg_config_t geom = {
        .len_coxa = 0.068f,  // 68 mm
        .len_femur = 0.088f, // 88 mm
        .len_tibia = 0.127f, // 127 mm
        .coxa_offset_rad = 0*-0.017453292519943295f,
        .femur_offset_rad = 0.5396943301595464f,
        .tibia_offset_rad = 1.0160719600939494f,
    };

    for (int i = 0; i < NUM_LEGS; ++i) {
        (void)leg_configure(&geom, &g_cfg.legs[i]);
        // TODO: Consider per-leg geometry differences (mirrors, tolerances) via stored config
    }

    // --- Joint calibration defaults ---
    // Range: [-90°, +90°], no inversion, zero offset = 0, endpoints 500..2500us, neutral ~1500us
    for (int i = 0; i < NUM_LEGS; ++i) {
        for (int j = 0; j < 3; ++j) {
            joint_calib_t *c = &g_cfg.joint_calib[i][j];
            c->zero_offset_rad = 0.0f;
            if (j == LEG_SERVO_COXA || j == LEG_SERVO_TIBIA) {
                c->invert_sign = -1; // coxa often needs inversion
            } else {
                c->invert_sign = 1;
            }
            // if (i>= LEG_RIGHT_FRONT && j == LEG_SERVO_COXA) {
            //     c->invert_sign *= -1; // right-side coxa often needs inversion
            // }
            c->min_rad = (float)-M_PI * 0.5f;
            c->max_rad = (float) M_PI * 0.5f;
            c->pwm_min_us = 500;
            c->pwm_max_us = 2500;
            c->neutral_us = 1500;
        }
    }

    // --- Mount poses (defaults) ---
    // Indexing convention (example): 0..2 left front->rear, 3..5 right front->rear.
    // Body frame: x forward (+), y left (+), z up (+).
    // User defaults:
    //  - Front/back legs offset in x by ±0.08 m (front +0.08, back -0.08)
    //  - All legs offset in y by ±0.05 m (left +0.05, right -0.05)
    //  - Mount height z ~ 0.0 m baseline (adjust if topological zero differs)
    //  - Base yaw chosen so that neutral servo points outward:
    //      left side: +90° (pi/2) from body forward to left (outward)
    //      right side: -90° (-pi/2) from body forward to right (outward)
    // NOTE: If your neutral servo mechanically points outward with zero angle, you might set these to 0/π and
    // then account for the 90° rotation in the leg’s joint zero offset at the actuator layer. We choose +/−90° here
    // to absorb the chassis-to-leg frame rotation so IK gets a consistent leg-local frame.

    const float X_OFF_FRONT = 0.08f;
    const float X_OFF_REAR  = -0.08f;
    const float Y_OFF_LEFT  = 0.05f;
    const float Y_OFF_RIGHT = -0.05f;
    const float Z_OFF       = 0.0f;
    const float YAW_LEFT    = (float)M_PI * 0.5f;   // +90 deg
    const float YAW_RIGHT   = (float)-M_PI * 0.5f;  // -90 deg
    
    const float QANGLE = (float)M_PI * 0.25f;   // +45 deg
    const float STD_STANCE_OUT = 0.15f; // meters
    const float STD_STANCE_FWD = 0.0f;  // meters

    for (int i = 0; i < NUM_LEGS; ++i) {
        g_stance_out[i] = STD_STANCE_OUT;
        g_stance_fwd[i] = STD_STANCE_FWD;
    }

    // Leg mount poses by enum
    g_base_x[LEG_LEFT_FRONT] = X_OFF_FRONT;  
    g_base_y[LEG_LEFT_FRONT] = Y_OFF_LEFT;  
    g_base_z[LEG_LEFT_FRONT] = Z_OFF;  
    g_base_yaw[LEG_LEFT_FRONT] = YAW_LEFT - QANGLE;

    g_base_x[LEG_LEFT_MIDDLE] = 0.0f;
    g_base_y[LEG_LEFT_MIDDLE] = Y_OFF_LEFT; 
    g_base_z[LEG_LEFT_MIDDLE] = Z_OFF;
    g_base_yaw[LEG_LEFT_MIDDLE] = YAW_LEFT;

    g_base_x[LEG_LEFT_REAR] = X_OFF_REAR;  
    g_base_y[LEG_LEFT_REAR] = Y_OFF_LEFT; 
    g_base_z[LEG_LEFT_REAR] = Z_OFF;  
    g_base_yaw[LEG_LEFT_REAR] = YAW_LEFT + QANGLE;

    g_base_x[LEG_RIGHT_FRONT] = X_OFF_FRONT;
    g_base_y[LEG_RIGHT_FRONT] = Y_OFF_RIGHT;
    g_base_z[LEG_RIGHT_FRONT] = Z_OFF;
    g_base_yaw[LEG_RIGHT_FRONT] = YAW_RIGHT + QANGLE;

    g_base_x[LEG_RIGHT_MIDDLE] = 0.0f;     
    g_base_y[LEG_RIGHT_MIDDLE] = Y_OFF_RIGHT; 
    g_base_z[LEG_RIGHT_MIDDLE] = Z_OFF; 
    g_base_yaw[LEG_RIGHT_MIDDLE] = YAW_RIGHT;

    g_base_x[LEG_RIGHT_REAR] = X_OFF_REAR;  
    g_base_y[LEG_RIGHT_REAR] = Y_OFF_RIGHT;  
    g_base_z[LEG_RIGHT_REAR] = Z_OFF;   
    g_base_yaw[LEG_RIGHT_REAR] = YAW_RIGHT - QANGLE;

    // --- Future hardware settings (not applied here; live in robot_control) ---
    // - Servo pins and MCPWM mapping
    // - Per-joint limits/offsets/inversions
    // - Mount poses (position + yaw)

    // --- Debug defaults ---
    g_cfg.debug_leg_enable = 1;         // enable by default for bring-up
    g_cfg.debug_leg_index = 0;          // monitor leg 0 by default
    g_cfg.debug_leg_delta_thresh = 0.0174533f; // ~1 degree
    g_cfg.debug_leg_min_interval_ms = 100;     // 100 ms between logs min
    
    // Mark configuration as dirty since we're using defaults
    g_config_dirty = true;
    ESP_LOGI(TAG, "Initialized with hardcoded defaults");
}

leg_handle_t robot_config_get_leg(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return NULL;
    return g_cfg.legs[leg_index];
}

void robot_config_get_base_pose(int leg_index, float *x, float *y, float *z, float *yaw) {
    if (x)   *x = (leg_index >= 0 && leg_index < NUM_LEGS) ? g_base_x[leg_index] : 0.0f;
    if (y)   *y = (leg_index >= 0 && leg_index < NUM_LEGS) ? g_base_y[leg_index] : 0.0f;
    if (z)   *z = (leg_index >= 0 && leg_index < NUM_LEGS) ? g_base_z[leg_index] : 0.0f;
    if (yaw) *yaw = (leg_index >= 0 && leg_index < NUM_LEGS) ? g_base_yaw[leg_index] : 0.0f;
}

const joint_calib_t* robot_config_get_joint_calib(int leg_index, leg_servo_t joint) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return NULL;
    int j = (int)joint;
    if (j < 0 || j >= 3) return NULL;
    return &g_cfg.joint_calib[leg_index][j];
}

int robot_config_get_servo_gpio(int leg_index, leg_servo_t joint) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return -1;
    int j = (int)joint;
    if (j < 0 || j >= 3) return -1;
    return g_cfg.servo_gpio[leg_index][j];
}

int robot_config_get_mcpwm_group(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0;
    return g_cfg.mcpwm_group_id[leg_index];
}

int robot_config_get_servo_driver(int leg_index, leg_servo_t joint) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0;
    int j = (int)joint; if (j < 0 || j >= 3) return 0;
    return g_cfg.servo_driver_sel[leg_index][j];
}

int robot_config_debug_enabled(void) {
    return g_cfg.debug_leg_enable;
}
int robot_config_debug_leg_index(void) {
    return g_cfg.debug_leg_index;
}
float robot_config_debug_delta_thresh(void) {
    return g_cfg.debug_leg_delta_thresh;
}

float robot_config_get_stance_out_m(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
    return g_stance_out[leg_index];
}
float robot_config_get_stance_fwd_m(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
    return g_stance_fwd[leg_index];
}

// ============================================================================
// NVS Integration Implementation
// ============================================================================

static esp_err_t convert_to_fat_gpio_config(robot_gpio_config_t *fat_gpio) {
    if (!fat_gpio) return ESP_ERR_INVALID_ARG;
    
    // Copy GPIO mappings
    memcpy(fat_gpio->servo_gpio, g_cfg.servo_gpio, sizeof(fat_gpio->servo_gpio));
    
    return ESP_OK;
}

static esp_err_t convert_from_fat_gpio_config(const robot_gpio_config_t *fat_gpio) {
    if (!fat_gpio) return ESP_ERR_INVALID_ARG;
    
    // Copy GPIO mappings
    memcpy(g_cfg.servo_gpio, fat_gpio->servo_gpio, sizeof(g_cfg.servo_gpio));
    
    return ESP_OK;
}

static esp_err_t convert_to_fat_poses_config(robot_poses_config_t *fat_poses) {
    if (!fat_poses) return ESP_ERR_INVALID_ARG;
    
    // Copy mount poses
    for (int i = 0; i < NUM_LEGS; i++) {
        fat_poses->mount_poses[i][0] = g_base_x[i];
        fat_poses->mount_poses[i][1] = g_base_y[i];
        fat_poses->mount_poses[i][2] = g_base_z[i];
        fat_poses->mount_poses[i][3] = g_base_yaw[i];
    }
    
    return ESP_OK;
}

static esp_err_t convert_from_fat_poses_config(const robot_poses_config_t *fat_poses) {
    if (!fat_poses) return ESP_ERR_INVALID_ARG;
    
    // Copy mount poses
    for (int i = 0; i < NUM_LEGS; i++) {
        g_base_x[i] = fat_poses->mount_poses[i][0];
        g_base_y[i] = fat_poses->mount_poses[i][1];
        g_base_z[i] = fat_poses->mount_poses[i][2];
        g_base_yaw[i] = fat_poses->mount_poses[i][3];
    }
    
    return ESP_OK;
}

esp_err_t robot_config_load_from_nvs(void) {
    ESP_LOGI(TAG, "Loading robot configuration from FAT filesystem");
    
    // Initialize FAT filesystem if not already done
    esp_err_t err = robot_config_fat_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FAT filesystem: %s", esp_err_to_name(err));
        g_fat_available = false;
        return err;
    }
    g_fat_available = true;
    
    // Load GPIO configuration
    robot_gpio_config_t gpio_config;
    err = robot_config_fat_load_gpio(&gpio_config);
    if (err == ESP_OK) {
        convert_from_fat_gpio_config(&gpio_config);
        ESP_LOGI(TAG, "Loaded GPIO config from FAT");
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "GPIO config not found, using defaults");
    } else {
        ESP_LOGE(TAG, "Failed to load GPIO config: %s", esp_err_to_name(err));
    }
    
    // Load poses configuration
    robot_poses_config_t poses_config;
    err = robot_config_fat_load_poses(&poses_config);
    if (err == ESP_OK) {
        convert_from_fat_poses_config(&poses_config);
        ESP_LOGI(TAG, "Loaded poses config from FAT");
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Poses config not found, using defaults");
    } else {
        ESP_LOGE(TAG, "Failed to load poses config: %s", esp_err_to_name(err));
    }
    
    // Load per-leg calibrations (when implemented)
    for (int i = 0; i < NUM_LEGS; i++) {
        robot_leg_calib_config_t leg_calib;
        err = robot_config_fat_load_leg_calib(i, &leg_calib);
        if (err == ESP_OK) {
            memcpy(g_cfg.joint_calib[i], leg_calib.joints, sizeof(leg_calib.joints));
            ESP_LOGD(TAG, "Loaded leg %d calibration from FAT", i);
        } else if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGD(TAG, "Leg %d calibration not found or not implemented, using defaults", i);
        } else {
            ESP_LOGE(TAG, "Failed to load leg %d calibration: %s", i, esp_err_to_name(err));
        }
    }
    
    g_config_dirty = false;
    ESP_LOGI(TAG, "Robot configuration loaded successfully from FAT filesystem");
    return ESP_OK;
}

esp_err_t robot_config_save_to_nvs(void) {
    if (!g_fat_available) {
        ESP_LOGE(TAG, "FAT filesystem not available for saving configuration");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Saving robot configuration to FAT filesystem");
    
    // Save GPIO configuration
    robot_gpio_config_t gpio_config;
    esp_err_t err = convert_to_fat_gpio_config(&gpio_config);
    if (err == ESP_OK) {
        err = robot_config_fat_save_gpio(&gpio_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save GPIO config: %s", esp_err_to_name(err));
            return err;
        }
    }
    
    // Save poses configuration
    robot_poses_config_t poses_config;
    err = convert_to_fat_poses_config(&poses_config);
    if (err == ESP_OK) {
        err = robot_config_fat_save_poses(&poses_config);
        if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGE(TAG, "Failed to save poses config: %s", esp_err_to_name(err));
            return err;
        }
    }
    
    // Save per-leg calibrations (when implemented)
    for (int i = 0; i < NUM_LEGS; i++) {
        robot_leg_calib_config_t leg_calib = {
            .leg_index = i,
            .last_calibrated_timestamp = esp_timer_get_time()
        };
        memcpy(leg_calib.joints, g_cfg.joint_calib[i], sizeof(leg_calib.joints));
        snprintf(leg_calib.notes, sizeof(leg_calib.notes), "Auto-generated calibration for leg %d", i);
        
        err = robot_config_fat_save_leg_calib(i, &leg_calib);
        if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGE(TAG, "Failed to save leg %d calibration: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Update system metadata
    system_config_t system;
    err = robot_config_fat_load_system(&system);
    if (err != ESP_OK) {
        // Create new system config if it doesn't exist
        memset(&system, 0, sizeof(system));
        system.config_version = CONFIG_VERSION_CURRENT;
        system.created_timestamp = esp_timer_get_time();
        strcpy(system.robot_name, "Hexapod-001");
        strcpy(system.firmware_version, "1.0.0");
    }
    system.last_modified_timestamp = esp_timer_get_time();
    
    err = robot_config_fat_save_system(&system);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save system metadata: %s", esp_err_to_name(err));
        return err;
    }
    
    g_config_dirty = false;
    ESP_LOGI(TAG, "Robot configuration saved successfully to FAT filesystem");
    return ESP_OK;
}

esp_err_t robot_config_factory_reset(void) {
    ESP_LOGI(TAG, "Performing factory reset");
    
    if (!g_fat_available) {
        ESP_LOGW(TAG, "FAT filesystem not available, only resetting in-memory config");
        robot_config_init_default();
        return ESP_OK;
    }
    
    // Perform FAT filesystem factory reset
    esp_err_t err = robot_config_fat_factory_reset(RESET_FACTORY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAT factory reset failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Reinitialize with defaults
    robot_config_init_default();
    g_config_dirty = true;  // Mark for saving
    
    ESP_LOGI(TAG, "Factory reset completed");
    return ESP_OK;
}

// Enhanced configuration setters
esp_err_t robot_config_set_joint_calib(int leg_index, leg_servo_t joint, const joint_calib_t *calib) {
    if (leg_index < 0 || leg_index >= NUM_LEGS || !calib) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int j = (int)joint;
    if (j < 0 || j >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&g_cfg.joint_calib[leg_index][j], calib, sizeof(joint_calib_t));
    g_config_dirty = true;
    
    ESP_LOGD(TAG, "Updated leg %d joint %d calibration", leg_index, j);
    return ESP_OK;
}

esp_err_t robot_config_set_servo_gpio(int leg_index, leg_servo_t joint, int gpio) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int j = (int)joint;
    if (j < 0 || j >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_cfg.servo_gpio[leg_index][j] = gpio;
    g_config_dirty = true;
    
    ESP_LOGD(TAG, "Updated leg %d joint %d GPIO to %d", leg_index, j, gpio);
    return ESP_OK;
}

esp_err_t robot_config_set_base_pose(int leg_index, float x, float y, float z, float yaw) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_base_x[leg_index] = x;
    g_base_y[leg_index] = y;
    g_base_z[leg_index] = z;
    g_base_yaw[leg_index] = yaw;
    g_config_dirty = true;
    
    ESP_LOGD(TAG, "Updated leg %d base pose", leg_index);
    return ESP_OK;
}

esp_err_t robot_config_set_stance_params(int leg_index, float stance_out_m, float stance_fwd_m) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_stance_out[leg_index] = stance_out_m;
    g_stance_fwd[leg_index] = stance_fwd_m;
    g_config_dirty = true;
    
    ESP_LOGD(TAG, "Updated leg %d stance parameters", leg_index);
    return ESP_OK;
}

// Configuration status functions
bool robot_config_is_valid(void) {
    // Basic validation - check for reasonable values
    for (int i = 0; i < NUM_LEGS; i++) {
        // Check GPIO assignments are valid
        for (int j = 0; j < 3; j++) {
            int gpio = g_cfg.servo_gpio[i][j];
            if (gpio < -1 || gpio > 39) {  // ESP32 GPIO range
                return false;
            }
        }
        
        // Check base poses are reasonable
        if (fabs(g_base_x[i]) > 1.0f || fabs(g_base_y[i]) > 1.0f || fabs(g_base_z[i]) > 1.0f) {
            return false;
        }
    }
    
    return true;
}

bool robot_config_needs_save(void) {
    return g_config_dirty;
}

esp_err_t robot_config_mark_dirty(void) {
    g_config_dirty = true;
    return ESP_OK;
}

// ============================================================================
// Calibration and Configuration Helpers Implementation
// ============================================================================

static int g_calibration_leg = -1;  // Currently calibrating leg (-1 = none)

esp_err_t robot_config_start_calibration_mode(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_calibration_leg != -1) {
        ESP_LOGW(TAG, "Already in calibration mode for leg %d, switching to leg %d", g_calibration_leg, leg_index);
    }
    
    g_calibration_leg = leg_index;
    ESP_LOGI(TAG, "Started calibration mode for leg %d", leg_index);
    return ESP_OK;
}

esp_err_t robot_config_end_calibration_mode(void) {
    if (g_calibration_leg == -1) {
        ESP_LOGW(TAG, "Not in calibration mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Ended calibration mode for leg %d", g_calibration_leg);
    g_calibration_leg = -1;
    
    // Mark config as dirty since calibration likely changed values
    g_config_dirty = true;
    return ESP_OK;
}

esp_err_t robot_config_calibrate_joint_range(int leg_index, leg_servo_t joint, int pwm_min, int pwm_max, float angle_min, float angle_max) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int j = (int)joint;
    if (j < 0 || j >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate PWM and angle ranges
    if (pwm_min >= pwm_max || pwm_min < 500 || pwm_max > 2500) {
        ESP_LOGE(TAG, "Invalid PWM range [%d, %d]", pwm_min, pwm_max);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (angle_min >= angle_max || fabs(angle_min) > M_PI || fabs(angle_max) > M_PI) {
        ESP_LOGE(TAG, "Invalid angle range [%.3f, %.3f]", angle_min, angle_max);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update calibration
    joint_calib_t *calib = &g_cfg.joint_calib[leg_index][j];
    calib->pwm_min_us = pwm_min;
    calib->pwm_max_us = pwm_max;
    calib->min_rad = angle_min;
    calib->max_rad = angle_max;
    calib->neutral_us = (pwm_min + pwm_max) / 2;  // Simple midpoint
    
    g_config_dirty = true;
    
    ESP_LOGI(TAG, "Calibrated leg %d joint %d: PWM[%d,%d] -> Angle[%.3f,%.3f]", 
             leg_index, j, pwm_min, pwm_max, angle_min, angle_max);
    return ESP_OK;
}

esp_err_t robot_config_auto_detect_neutral(int leg_index, leg_servo_t joint) {
    // This would involve moving the joint to find the mechanical neutral
    // For now, just set to midpoint of current range
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int j = (int)joint;
    if (j < 0 || j >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    joint_calib_t *calib = &g_cfg.joint_calib[leg_index][j];
    calib->neutral_us = (calib->pwm_min_us + calib->pwm_max_us) / 2;
    
    // Set zero offset to middle of angle range
    calib->zero_offset_rad = (calib->min_rad + calib->max_rad) / 2.0f;
    
    g_config_dirty = true;
    
    ESP_LOGI(TAG, "Auto-detected neutral for leg %d joint %d: PWM=%dus, offset=%.3frad", 
             leg_index, j, calib->neutral_us, calib->zero_offset_rad);
    return ESP_OK;
}

void robot_config_print_summary(void) {
    ESP_LOGI(TAG, "=== Robot Configuration Summary ===");
    ESP_LOGI(TAG, "Configuration dirty: %s", g_config_dirty ? "YES" : "NO");
    ESP_LOGI(TAG, "FAT available: %s", g_fat_available ? "YES" : "NO");
    ESP_LOGI(TAG, "Configuration valid: %s", robot_config_is_valid() ? "YES" : "NO");
    
    if (g_calibration_leg >= 0) {
        ESP_LOGI(TAG, "Calibration mode: Active (Leg %d)", g_calibration_leg);
    } else {
        ESP_LOGI(TAG, "Calibration mode: Inactive");
    }
    
    // Print GPIO assignments
    ESP_LOGI(TAG, "--- GPIO Assignments ---");
    for (int i = 0; i < NUM_LEGS; i++) {
        ESP_LOGI(TAG, "Leg %d: Coxa=%d, Femur=%d, Tibia=%d (Group %d)", i,
                 g_cfg.servo_gpio[i][0], g_cfg.servo_gpio[i][1], g_cfg.servo_gpio[i][2],
                 g_cfg.mcpwm_group_id[i]);
    }
    
    // Print mount poses
    ESP_LOGI(TAG, "--- Mount Poses ---");
    for (int i = 0; i < NUM_LEGS; i++) {
        ESP_LOGI(TAG, "Leg %d: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", i,
                 g_base_x[i], g_base_y[i], g_base_z[i], g_base_yaw[i]);
    }
}

void robot_config_print_calibration_status(void) {
    ESP_LOGI(TAG, "=== Calibration Status ===");
    
    for (int i = 0; i < NUM_LEGS; i++) {
        ESP_LOGI(TAG, "--- Leg %d ---", i);
        const char *joint_names[] = {"Coxa", "Femur", "Tibia"};
        
        for (int j = 0; j < 3; j++) {
            const joint_calib_t *calib = &g_cfg.joint_calib[i][j];
            ESP_LOGI(TAG, "  %s: PWM[%d-%d]us, Angle[%.3f-%.3f]rad, Offset=%.3frad, Invert=%d",
                     joint_names[j], calib->pwm_min_us, calib->pwm_max_us,
                     calib->min_rad, calib->max_rad, calib->zero_offset_rad, calib->invert_sign);
        }
    }
}

esp_err_t robot_config_get_config_stats(size_t *total_size, size_t *used_size, uint32_t *last_save_time) {
    if (!g_fat_available) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get actual storage usage from FAT filesystem
    size_t total_bytes = 0, used_bytes = 0, free_bytes = 0;
    esp_err_t err = robot_config_fat_get_storage_info(&total_bytes, &used_bytes, &free_bytes);
    if (err != ESP_OK) {
        return err;
    }
    
    if (total_size) {
        *total_size = total_bytes;
    }
    
    if (used_size) {
        *used_size = used_bytes;
    }
    
    if (last_save_time) {
        // Try to get last modified time from system config
        system_config_t system_config;
        err = robot_config_fat_load_system(&system_config);
        if (err == ESP_OK) {
            *last_save_time = (uint32_t)(system_config.last_modified_timestamp / 1000000ULL);  // Convert to seconds
        } else {
            *last_save_time = 0;
        }
    }
    
    return ESP_OK;
}

// Backup/restore stubs (full implementation would require more complex serialization)
esp_err_t robot_config_backup_to_buffer(uint8_t **backup_data, size_t *backup_size) {
    ESP_LOGW(TAG, "Configuration backup to buffer not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_restore_from_buffer(const uint8_t *backup_data, size_t backup_size) {
    ESP_LOGW(TAG, "Configuration restore from buffer not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_config_compare_with_defaults(bool *differences_found) {
    if (!differences_found) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // This would require creating a default config and comparing
    // For now, assume differences exist if config is dirty
    *differences_found = g_config_dirty;
    
    ESP_LOGW(TAG, "Full comparison with defaults not yet implemented");
    return ESP_OK;
}
