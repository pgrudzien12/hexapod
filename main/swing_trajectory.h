#ifndef SWING_TRAJECTORY_H
#define SWING_TRAJECTORY_H

#include <stdint.h>
#include "gait_scheduler.h" // for leg_state_t and NUM_LEGS

typedef struct {
    // Body frame (right-handed): X forward, Y left, Z up.
    // z is absolute height above reference plane (e.g., ground). Lower foot = smaller z.
    float x;
    float y;
    float z; // Z up
} foot_position_t;

typedef struct {
    foot_position_t desired_positions[NUM_LEGS];
    float step_length;
    float clearance_height;
    // Scaling from normalized pose commands (-1..+1) to meters
    // TODO: Consider exposing these via config or a calibration struct
    //       so they can be tuned without recompiling.
    float y_range_m;   // maps y_offset: -1..+1 -> +/- y_range_m
    // Absolute body height limits (meters) for safe mapping of z_target
    // For Z up convention: z_min_m < z_max_m, both positive heights.
    // Foot target heights should stay within [z_min_m, z_max_m].
    float z_min_m;     // minimum allowed foot height (closest to ground)
    float z_max_m;     // maximum allowed foot height (lift limit)
} swing_trajectory_t;

void swing_trajectory_init(swing_trajectory_t *trajectory, float step_length, float clearance_height);
// Generate desired foot positions using scheduler state and current user command
// NOTE: This currently implements a simple cycloid swing and flat support.
// TODO: Add yaw (wz) coupling to bias per-leg x/y for turning, and add
//       terrain-dependent clearance shaping.
void swing_trajectory_generate(swing_trajectory_t *trajectory, const gait_scheduler_t *scheduler, const user_command_t *cmd);

#endif // SWING_TRAJECTORY_H
