/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "leg.h"
#include <math.h>
#include "gait_scheduler.h"
#include "swing_trajectory.h"
#include "whole_body_control.h"
#include "robot_control.h"
#include "robot_config.h"
#include "user_command.h"
#include "controller.h"

static const char *TAG = "leg";

// Define the three servo pins for the leg here
#define LEG_SERVO1_GPIO 13
#define LEG_SERVO2_GPIO 12
#define LEG_SERVO3_GPIO 14

float deg_to_rad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

// --- Gait Framework Main Loop ---
void app_main(void)
{
    gait_scheduler_t scheduler;
    swing_trajectory_t trajectory;
    whole_body_cmd_t cmds;
    user_command_t ucmd;

    // Initialize modules with example parameters
    gait_scheduler_init(&scheduler, 0.25f); // 0.5 second cycle time
    swing_trajectory_init(&trajectory, 0.10f, 0.03f); // 10cm step, 3cm clearance
    robot_config_init_default();
    // TODO: Calibrate swing_trajectory y/z ranges for your robot; WBC expects meters
    user_command_init();

    const float dt = 0.01f; // 10ms loop
    while (1) {
        // Read user command (placeholder)
        user_command_poll(&ucmd);
        ucmd.enable = 1; // Force enable for testing
        ucmd.gait = GAIT_TRIPOD;
        ucmd.vx = 1.f; 
        ucmd.step_scale = 1.0f; // Medium step length

        // ESP_LOGI(TAG, "Cmd: gait=%d vx=%.2f step_scale=%.2f z_target=%.2f y_offset=%.2f terrain_climb=%d enable=%d",
        //          ucmd.gait, ucmd.vx, ucmd.step_scale, ucmd.z_target, ucmd.y_offset, ucmd.terrain_climb, ucmd.enable);

        // Update gait scheduler (leg phases)
        gait_scheduler_update(&scheduler, dt, &ucmd);
        // ESP_LOGI(TAG, "Scheduler: phase=%.3f, LEG1=%d", scheduler.phase, (int)scheduler.leg_states[0]);


        // // Generate swing trajectories for legs using scheduler + command
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);
        // ESP_LOGI(TAG, "Gait=%d, vx=%.2f, Trajectory: L1=(%.3f, %.3f, %.3f) L2=(%.3f, %.3f, %.3f) L3=(%.3f, %.3f, %.3f)",
        //          ucmd.gait,
        //          ucmd.vx,
        //          trajectory.desired_positions[0].x,
        //          trajectory.desired_positions[0].y,
        //          trajectory.desired_positions[0].z,
        //          trajectory.desired_positions[1].x,
        //          trajectory.desired_positions[1].y,
        //          trajectory.desired_positions[1].z,
        //          trajectory.desired_positions[2].x,
        //          trajectory.desired_positions[2].y,
        //          trajectory.desired_positions[2].z);

        // ESP_LOGI(TAG, "Gait=%d, vx=%.2f, Trajectory: L4=(%.3f, %.3f, %.3f) L5=(%.3f, %.3f, %.3f) L6=(%.3f, %.3f, %.3f)",
        //          ucmd.gait,
        //          ucmd.vx,
        //          trajectory.desired_positions[3].x,
        //          trajectory.desired_positions[3].y,
        //          trajectory.desired_positions[3].z,
        //          trajectory.desired_positions[4].x,
        //          trajectory.desired_positions[4].y,
        //          trajectory.desired_positions[4].z,
        //          trajectory.desired_positions[5].x,
        //          trajectory.desired_positions[5].y,
        //          trajectory.desired_positions[5].z);

        // // Compute joint commands from trajectories
        trajectory.desired_positions[0].x = 0.25f;
        trajectory.desired_positions[0].y = 0.15f;
        trajectory.desired_positions[0].z = 0.08f;
        whole_body_control_compute(&trajectory, &cmds);
        // ESP_LOGI(TAG, "Cmds: L1=(%.1f, %.1f, %.1f) L2=(%.1f, %.1f, %.1f) L3=(%.1f, %.1f, %.1f)",
        //      ((float*)(&cmds.joint_cmds[0]))[0],
        //      ((float*)(&cmds.joint_cmds[0]))[1],
        //      ((float*)(&cmds.joint_cmds[0]))[2],
        //      ((float*)(&cmds.joint_cmds[1]))[0],
        //      ((float*)(&cmds.joint_cmds[1]))[1],
        //      ((float*)(&cmds.joint_cmds[1]))[2],
        //      ((float*)(&cmds.joint_cmds[2]))[0],
        //      ((float*)(&cmds.joint_cmds[2]))[1],
        //      ((float*)(&cmds.joint_cmds[2]))[2]);

        // // Debug log for a single leg (angle deltas)
        // leg_debugger_update(&cmds);
        // cmds.joint_cmds[0].joint_angles[0] = 0.0f;
        // cmds.joint_cmds[0].joint_angles[1] = 0.0f;
        // cmds.joint_cmds[0].joint_angles[2] = 0.0f;
        // cmds.joint_cmds[3].joint_angles[0] = 0.0f;
        // cmds.joint_cmds[3].joint_angles[1] = 0.0f;
        // cmds.joint_cmds[3].joint_angles[2] = 0.0f;
        // // Send commands to robot
        robot_execute(&cmds);

        // Wait for next loop (replace with ESP-IDF delay)
        vTaskDelay((int)(dt * 1000) / portTICK_PERIOD_MS);
        // loop timing governed by vTaskDelay; phase advanced via dt
    }
}
