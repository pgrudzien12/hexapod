/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_timer.h"
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

// --- Gait Framework Main Loop ---
void gait_framework_main(void *arg)
{
    gait_scheduler_t scheduler;
    swing_trajectory_t trajectory;
    whole_body_cmd_t cmds;
    user_command_t ucmd;

    // Initialize modules with example parameters
    gait_scheduler_init(&scheduler, 2.f); // 1 second cycle time
    swing_trajectory_init(&trajectory, 0.06f, 0.03f); // 6cm step, 3cm clearance
    robot_config_init_default();
    // TODO: Calibrate swing_trajectory y/z ranges for your robot; WBC expects meters
    user_command_init();

    const float dt = 0.01f; // 10ms loop
    while (1) {
        float time_start = esp_timer_get_time();
        // Read user command (placeholder)
        user_command_poll(&ucmd);

        ucmd.enable = 1; // Force enable for testing
        ucmd.gait = GAIT_TRIPOD;
        ucmd.vx = 1.f; 
        ucmd.z_target = 0.0f; // Mid height
        ucmd.step_scale = 1.0f; // Medium step length

        // Update gait scheduler (leg phases)
        gait_scheduler_update(&scheduler, dt, &ucmd);

        // // Generate swing trajectories for legs using scheduler + command
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);


        // Compute joint commands from trajectories
        // trajectory.desired_positions[0].x = 0.080;
        // trajectory.desired_positions[0].y = 0.23f;
        // trajectory.desired_positions[0].z = -0.00f;
        whole_body_control_compute(&trajectory, &cmds);
        // cmds.joint_cmds[0].joint_angles[0] = 0.0f;
        // cmds.joint_cmds[0].joint_angles[1] = 0.0f;
        // cmds.joint_cmds[0].joint_angles[2] = scheduler.phase * M_PI/4.0; // test tibia motion
        // Send commands to robot
        robot_execute(&cmds);

        
        float time_end = esp_timer_get_time();

        // Calculate how long to wait to maintain dt period
        // this reports about 0.41 ms per loop if no logging
        float elapsed = (time_end - time_start) / 1000.0f;
        // wait at least 1 tick to yield to other tasks
        float wait_ms = (dt * 1000.0f) - elapsed;
        int wait_ticks = (int)(wait_ms / portTICK_PERIOD_MS);
        if (wait_ticks < 1) wait_ticks = 1; // Ensure at least 1 tick
        // ESP_LOGI(TAG, "Loop time: %.3f ms, wait: %.3f ms (%d ticks)", elapsed, wait_ms, wait_ticks);
        vTaskDelay(wait_ticks);
    }
}

void app_main(void)
{
    gait_framework_main(NULL);
}