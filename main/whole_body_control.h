#ifndef WHOLE_BODY_CONTROL_H
#define WHOLE_BODY_CONTROL_H

#include <stdint.h>
#include "swing_trajectory.h"
#include "robot_config.h"


// Joint angles for one leg
typedef struct {
    // Temporarily repurposed to carry leg-local Cartesian foot targets (x,y,z)
    // TODO: Rename this structure or add a separate field for Cartesian targets.
    float joint_angles[3]; // [0]=x_leg, [1]=y_leg, [2]=z_leg
} leg_joint_cmd_t;

typedef struct {
    leg_joint_cmd_t joint_cmds[NUM_LEGS];
} whole_body_cmd_t;

void whole_body_control_compute(const swing_trajectory_t *trajectory, whole_body_cmd_t *cmds);

#endif // WHOLE_BODY_CONTROL_H
