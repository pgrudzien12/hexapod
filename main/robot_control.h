#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "whole_body_control.h"
#include "leg.h" // for leg_servo_t enum
#include "esp_err.h"

// Apply whole-body command: consumes leg-local Cartesian targets, runs IK, and drives servos.
void robot_execute(const whole_body_cmd_t *cmds);

// Low-level joint command API (host for moved functionality from leg.c).
// Implementations will clamp, apply offsets, and command servos per joint.
esp_err_t robot_set_joint_angle_rad(int leg_index, leg_servo_t joint, float radians);

#endif // ROBOT_CONTROL_H
