#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "leg.h"
#include "whole_body_control.h"

// Central robot configuration holder.
// NOTE: Keep this minimal and focused on data; we're planning to move settings to ESP storage later.
// TODO(ESP-Storage): Persist all fields in NVS (or preferred storage) and load at boot.
typedef struct {
    // IK geometry per leg (opaque handles with only lengths inside)
    leg_handle_t legs[NUM_LEGS];

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

#endif // ROBOT_CONFIG_H
