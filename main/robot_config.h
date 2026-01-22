#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "leg.h"
#include "types/joint_types.h"
#include "robot_constants.h"



// Leg index enumeration for identification
typedef enum {
    LEG_LEFT_FRONT = 0,   // x=+0.08, y=+0.05
    LEG_LEFT_MIDDLE = 1,  // x=0.00,  y=+0.05
    LEG_LEFT_REAR = 2,    // x=-0.08, y=+0.05
    LEG_RIGHT_FRONT = 3,  // x=+0.08, y=-0.05
    LEG_RIGHT_MIDDLE = 4, // x=0.00,  y=-0.05
    LEG_RIGHT_REAR = 5    // x=-0.08, y=-0.05
} leg_index_t;
// Central robot configuration holder.
// NOTE: Keep this minimal and focused on data; we're planning to move settings to ESP storage later.
// TODO(ESP-Storage): Persist all fields in NVS (or preferred storage) and load at boot.

typedef struct {
    // IK geometry per leg (opaque handles with only lengths inside)
    leg_handle_t legs[NUM_LEGS];

    // Servo GPIO mapping per leg/joint (−1 means unassigned)
    int servo_gpio[NUM_LEGS][3];
    // MCPWM group per leg (for now one group id per leg; simplest is all 0)
    int mcpwm_group_id[NUM_LEGS];

    // Servo driver selection per joint: 0 = MCPWM, 1 = LEDC (extend later if needed)
    int servo_driver_sel[NUM_LEGS][3];

    // Debug/telemetry controls
    int debug_leg_enable;          // 0/1 to enable leg debugger
    int debug_leg_index;           // which leg to monitor
    float debug_leg_delta_thresh;  // radians; min delta to log
    unsigned int debug_leg_min_interval_ms; // min interval between logs

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
// Right-handed body frame convention:
//   X: forward (+)
//   Y: left (+)
//   Z: up (+)
// yaw: rotation about +Z (right-handed, CCW positive when looking down from above)
void robot_config_get_base_pose(int leg_index, float *x, float *y, float *z, float *yaw);

// Get per-joint calibration (read-only pointer; lifetime of process).
const joint_calib_t* robot_config_get_joint_calib(int leg_index, leg_servo_t joint);

// GPIO mapping for a servo channel; returns -1 if unassigned.
int robot_config_get_servo_gpio(int leg_index, leg_servo_t joint);

// MCPWM group id for this leg (default 0).
int robot_config_get_mcpwm_group(int leg_index);

// Driver selection getter: 0 -> MCPWM, 1 -> LEDC
int robot_config_get_servo_driver(int leg_index, leg_servo_t joint);

// Debug controls getters
int robot_config_debug_enabled(void);
int robot_config_debug_leg_index(void);
float robot_config_debug_delta_thresh(void);

// Stance getters (per leg). In leg-local axes: X_outward (+), Y_forward (+). Units: meters.
float robot_config_get_stance_out_m(int leg_index);
float robot_config_get_stance_fwd_m(int leg_index);

#endif // ROBOT_CONFIG_H
