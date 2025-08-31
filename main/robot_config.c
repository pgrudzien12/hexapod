#include "robot_config.h"
#include <string.h>
#include <math.h>
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
    }
    // Rebalance within first three legs: move leg 2 to group 1 so group 0 only has two legs (6 channels)
    g_cfg.mcpwm_group_id[2] = 1;

    // Temporary GPIO assignment: leg 0 only
    // Joint order: [0]=COXA (yaw), [1]=FEMUR (pitch), [2]=TIBIA (knee)
    g_cfg.servo_gpio[0][LEG_SERVO_COXA]  = 13;
    g_cfg.servo_gpio[0][LEG_SERVO_FEMUR] = 12;
    g_cfg.servo_gpio[0][LEG_SERVO_TIBIA] = 14;
    
    // Keep other legs disabled for now (set to -1) until MCPWM capacity is confirmed and pins are verified
    g_cfg.servo_gpio[1][LEG_SERVO_COXA]  = -1;
    g_cfg.servo_gpio[1][LEG_SERVO_FEMUR] = -1;
    g_cfg.servo_gpio[1][LEG_SERVO_TIBIA] = -1;

    g_cfg.servo_gpio[2][LEG_SERVO_COXA]  = -1;
    g_cfg.servo_gpio[2][LEG_SERVO_FEMUR] = -1;
    g_cfg.servo_gpio[2][LEG_SERVO_TIBIA] = -1;

    // Default geometry for all 6 legs.
    // NOTE: Units must match usage across the project. Our swing_trajectory uses meters,
    // so we set lengths in meters as placeholders.
    // TODO(ESP-Storage): Replace with values loaded from storage per leg.
    const leg_config_t geom = {
        .len_coxa = 0.068f,  // 68 mm
        .len_femur = 0.088f, // 88 mm
        .len_tibia = 0.127f, // 127 mm
        .coxa_offset_rad = 4*-0.017453292519943295f,
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
            c->invert_sign = 1;
            c->min_rad = (float)-M_PI * 0.5f;
            c->max_rad = (float) M_PI * 0.5f;
            c->pwm_min_us = 500;
            c->pwm_max_us = 2500;
            c->neutral_us = 1500;
        }
    }

    // --- Mount poses (defaults) ---
    // Indexing convention (example): 0..2 left front->rear, 3..5 right front->rear.
    // Body frame: x forward (+), y left (+), z down (+).
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

    // Leg 0 (left-front)
    g_base_x[0] = X_OFF_FRONT; g_base_y[0] = Y_OFF_LEFT; g_base_z[0] = Z_OFF; g_base_yaw[0] = YAW_LEFT;
    g_stance_fwd[0] =  -0.10f; g_stance_out[0] = 0.15f; // 15 cm outward
    // Leg 1 (left-middle)
    g_base_x[1] = 0.0f;        g_base_y[1] = Y_OFF_LEFT; g_base_z[1] = Z_OFF; g_base_yaw[1] = YAW_LEFT;
    g_stance_fwd[1] = 0.0f; g_stance_out[1] = 0.15f; // 15 cm outward
    // Leg 2 (left-rear)
    g_base_x[2] = X_OFF_REAR;  g_base_y[2] = Y_OFF_LEFT; g_base_z[2] = Z_OFF; g_base_yaw[2] = YAW_LEFT;
    g_stance_fwd[2] = 0.10f; g_stance_out[2] = 0.15f; // 15 cm outward
    // Leg 3 (right-front)
    g_base_x[3] = X_OFF_FRONT; g_base_y[3] = Y_OFF_RIGHT; g_base_z[3] = Z_OFF; g_base_yaw[3] = YAW_RIGHT;
    g_stance_fwd[3] = -0.10f; g_stance_out[3] = 0.15f; // 15 cm outward
    // Leg 4 (right-middle)
    g_base_x[4] = 0.0f;        g_base_y[4] = Y_OFF_RIGHT; g_base_z[4] = Z_OFF; g_base_yaw[4] = YAW_RIGHT;
    g_stance_fwd[4] = 0.0f; g_stance_out[4] = 0.15f; // 15 cm outward
    // Leg 5 (right-rear)
    g_base_x[5] = X_OFF_REAR;  g_base_y[5] = Y_OFF_RIGHT; g_base_z[5] = Z_OFF; g_base_yaw[5] = YAW_RIGHT;
    g_stance_fwd[5] = 0.10f; g_stance_out[5] = 0.15f; // 15 cm outward

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

int robot_config_debug_enabled(void) {
    return g_cfg.debug_leg_enable;
}
int robot_config_debug_leg_index(void) {
    return g_cfg.debug_leg_index;
}
float robot_config_debug_delta_thresh(void) {
    return g_cfg.debug_leg_delta_thresh;
}
unsigned int robot_config_debug_min_interval_ms(void) {
    return g_cfg.debug_leg_min_interval_ms;
}

float robot_config_get_stance_out_m(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
    return g_stance_out[leg_index];
}
float robot_config_get_stance_fwd_m(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
    return g_stance_fwd[leg_index];
}
