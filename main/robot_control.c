#include "robot_control.h"

esp_err_t robot_set_joint_angle_rad(int leg_index, leg_servo_t joint, float radians) {
    (void)leg_index; (void)joint; (void)radians;
    // TODO: Implement MCPWM servo driving and per-joint calibration here.
    return ESP_OK;
}

void robot_execute(const whole_body_cmd_t *cmds) {
    (void)cmds;
    // TODO: Convert Cartesian targets to joint angles via leg_ik_solve and call robot_set_joint_angle_rad.
}
