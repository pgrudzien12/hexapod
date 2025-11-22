#include "robot_config.h"
#include "config_manager.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Single static instance for now. In the future, load/save to NVS.
static robot_config_t g_cfg;
static float g_base_x[NUM_LEGS];
static float g_base_y[NUM_LEGS];
static float g_base_z[NUM_LEGS];
static float g_base_yaw[NUM_LEGS];
static float g_stance_out[NUM_LEGS]; // leg-local outward (+)
static float g_stance_fwd[NUM_LEGS]; // leg-local forward (+)

void robot_config_init_default(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
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
    const leg_geometry_t geom = {
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

    // --- Joint calibration ---
    // Joint calibration is now handled directly by config_manager.
    // No caching needed - robot_config_get_joint_calib() calls config_manager on-demand.

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
    
    // Static storage for calibration data
    static joint_calib_t temp_calib;
    
    // Get data directly from configuration manager
    if (config_get_joint_calib_data(leg_index, j, &temp_calib) == ESP_OK) {
        // No conversion needed - structures are now identical
        return &temp_calib;
    }
    ESP_LOGW("robot_config", "robot_config_get_joint_calib: Using fallback defaults for leg %d joint %d", leg_index, j);
    
    // Fallback to default values if config manager is not available
    temp_calib.zero_offset_rad = 0.0f;
    temp_calib.invert_sign = (j == LEG_SERVO_COXA || j == LEG_SERVO_TIBIA) ? -1 : 1;
    temp_calib.min_rad = (float)-M_PI * 0.5f;
    temp_calib.max_rad = (float) M_PI * 0.5f;
    temp_calib.pwm_min_us = 500;
    temp_calib.pwm_max_us = 2500;
    temp_calib.neutral_us = 1500;
    return &temp_calib;
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


