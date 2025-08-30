#ifndef WHOLE_BODY_CONTROL_H
#define WHOLE_BODY_CONTROL_H

#include <stdint.h>
#include "swing_trajectory.h"

#define NUM_LEGS 6

// Joint angles for one leg
typedef struct {
    float joint_angles[3]; // Assume 3 DOF per leg
} leg_joint_cmd_t;

typedef struct {
    leg_joint_cmd_t joint_cmds[NUM_LEGS];
} whole_body_cmd_t;

void whole_body_control_compute(const swing_trajectory_t *trajectory, whole_body_cmd_t *cmds);

#endif // WHOLE_BODY_CONTROL_H
