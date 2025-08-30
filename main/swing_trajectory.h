#ifndef SWING_TRAJECTORY_H
#define SWING_TRAJECTORY_H

#include <stdint.h>
#include "gait_scheduler.h" // for leg_state_t and NUM_LEGS

typedef struct {
    float x;
    float y;
    float z;
} foot_position_t;

typedef struct {
    foot_position_t desired_positions[NUM_LEGS];
    float step_length;
    float clearance_height;
} swing_trajectory_t;

void swing_trajectory_init(swing_trajectory_t *trajectory, float step_length, float clearance_height);
// Generate desired foot positions using scheduler state and current user command
void swing_trajectory_generate(swing_trajectory_t *trajectory, const gait_scheduler_t *scheduler, const user_command_t *cmd);

#endif // SWING_TRAJECTORY_H
