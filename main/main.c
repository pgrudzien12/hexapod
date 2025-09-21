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
        float ms_start = esp_timer_get_time() / 1000.0f;
        // Read user command (placeholder)
        user_command_poll(&ucmd);

        ucmd.enable = 1; // Force enable for testing
        ucmd.gait = GAIT_TRIPOD;
        ucmd.vx = 1.f; 
        ucmd.step_scale = 1.0f; // Medium step length

        // Update gait scheduler (leg phases)
        gait_scheduler_update(&scheduler, dt, &ucmd);

        // // Generate swing trajectories for legs using scheduler + command
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);

        // Compute joint commands from trajectories
        trajectory.desired_positions[0].x = 0.25f;
        trajectory.desired_positions[0].y = 0.15f;
        trajectory.desired_positions[0].z = 0.08f;
        whole_body_control_compute(&trajectory, &cmds);

        // Send commands to robot
        robot_execute(&cmds);
        float ms_end = esp_timer_get_time() / 1000.0f;

        // Calculate how long to wait to maintain dt period
        // this reports about 0.41 ms per loop if no logging
        float elapsed = ms_end - ms_start;
        float wait_ms = (dt * 1000.0f) - elapsed;
        if (wait_ms > 0) {
            vTaskDelay((int)wait_ms / portTICK_PERIOD_MS);
        }
        // loop timing governed by vTaskDelay; phase advanced via dt
    }
}
