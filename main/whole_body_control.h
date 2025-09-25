#ifndef WHOLE_BODY_CONTROL_H
#define WHOLE_BODY_CONTROL_H

#include <stdint.h>
#include "swing_trajectory.h"
#include "robot_config.h"


// Joint angles for one leg.
// Frame conventions:
//   Body frame: X forward (+), Y left (+), Z up (+)
//   Leg-local frame: X outward from body (+), Y forward (+), Z up (+)
typedef struct {
    float joint_angles[3]; // [0]=coxa yaw (rad), [1]=femur pitch (rad), [2]=tibia (rad)
} leg_joint_cmd_t;

typedef struct {
    leg_joint_cmd_t joint_cmds[NUM_LEGS];
} whole_body_cmd_t;

void whole_body_control_compute(const swing_trajectory_t *trajectory, whole_body_cmd_t *cmds);

#endif // WHOLE_BODY_CONTROL_H
