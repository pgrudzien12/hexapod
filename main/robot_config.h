#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "leg.h"
#include "whole_body_control.h"

// Central robot configuration holder.
// NOTE: Keep this minimal and focused on data; we're planning to move settings to ESP storage later.
// TODO(ESP-Storage): Persist all fields in NVS (or preferred storage) and load at boot.
// Per-joint actuator calibration
typedef struct {
    float zero_offset_rad;  // mechanical zero offset added before inversion/clamp
    int8_t invert_sign;     // +1 or -1 to flip direction
    float min_rad;          // lower mechanical/electronic limit (radians)
    float max_rad;          // upper mechanical/electronic limit (radians)
    int pwm_min_us;         // PWM at min_rad (microseconds)
    int pwm_max_us;         // PWM at max_rad (microseconds)
    int neutral_us;         // PWM at neutral (optional; informative)
} joint_calib_t;

typedef struct {
    // IK geometry per leg (opaque handles with only lengths inside)
    leg_handle_t legs[NUM_LEGS];

    // Per-joint calibration per leg (coxa, femur, tibia)
    joint_calib_t joint_calib[NUM_LEGS][3];

    // Servo GPIO mapping per leg/joint (−1 means unassigned)
    int servo_gpio[NUM_LEGS][3];
    // MCPWM group per leg (for now one group id per leg; simplest is all 0)
    int mcpwm_group_id[NUM_LEGS];

    // --- Planned settings to migrate to storage (comments only for now) ---
    // - Servo GPIO pins per leg/joint (coxa/femur/tibia)
    // - MCPWM group/operator/timer mapping per joint
    // - Per-joint angle limits (min/max in radians)
    // - Per-joint calibration offsets (radians) and direction inversions
    // - Neutral positions and soft-limit margins
    // - Safety flags (enable gate, estop)
    // - Leg mounting poses (position + yaw) for accurate body->leg transform
    // - Units selection and scale (meters vs mm)
    // - Tuning parameters for gait/trajectory
} robot_config_t;

// Initialize a process-wide default config with placeholder geometry.
// Call this once at startup before using whole_body_control/robot_control.
void robot_config_init_default(void);

// Access per-leg IK handle (geometry). Returns NULL if not initialized.
leg_handle_t robot_config_get_leg(int leg_index);

// Get per-leg mount pose in body frame (meters, radians).
// x: forward (+), y: left (+), z: down (+), yaw: rotation about +Z.
void robot_config_get_base_pose(int leg_index, float *x, float *y, float *z, float *yaw);

// Get per-joint calibration (read-only pointer; lifetime of process).
const joint_calib_t* robot_config_get_joint_calib(int leg_index, leg_servo_t joint);

// GPIO mapping for a servo channel; returns -1 if unassigned.
int robot_config_get_servo_gpio(int leg_index, leg_servo_t joint);

// MCPWM group id for this leg (default 0).
int robot_config_get_mcpwm_group(int leg_index);

#endif // ROBOT_CONFIG_H
