/*
 *
 * License: Apache-2.0
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
#include <string.h>

static const char *TAG = "leg";

// --- Gait Framework Main Loop ---
void gait_framework_main(void *arg)
{
    gait_scheduler_t scheduler;
    swing_trajectory_t trajectory;
    whole_body_cmd_t cmds;
    user_command_t ucmd; // current command
    user_command_t prev_cmd; // previous command for change detection
    memset(&ucmd, 0, sizeof(ucmd));
    memset(&prev_cmd, 0, sizeof(prev_cmd));

    // Initialize modules with example parameters
    gait_scheduler_init(&scheduler, 1.5f); // 1.5 second cycle time
    swing_trajectory_init(&trajectory, 0.07f, 0.04f); // 7cm step, 4cm clearance
    robot_config_init_default();
    // TODO: Calibrate swing_trajectory y/z ranges for your robot; WBC expects meters
    user_command_init();

    const float dt = 0.01f; // 10ms loop
    while (1) {
        float time_start = esp_timer_get_time();
        // Copy current to previous then poll new
        prev_cmd = ucmd;
        user_command_poll(&ucmd);
        if (!controller_user_command_equal(&prev_cmd, &ucmd, 1e-2f)) {
            ESP_LOGD(TAG, "User command: vx=%.2f, wz=%.2f, z_target=%.2f, y_offset=%.2f gait=%d enable=%d pose=%d terrain=%d step_scale=%.2f",
                    ucmd.vx, ucmd.wz, ucmd.z_target, ucmd.y_offset, ucmd.gait, ucmd.enable, ucmd.pose_mode, ucmd.terrain_climb, ucmd.step_scale);
        }

        // Update gait scheduler (leg phases)
        gait_scheduler_update(&scheduler, dt, &ucmd);
        // Generate swing trajectories for legs using scheduler + command
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);
        // Compute joint commands from trajectories
        whole_body_control_compute(&trajectory, &cmds);
        // Send commands to robot
        robot_execute(&cmds);

        float time_end = esp_timer_get_time();
        // Calculate how long to wait to maintain dt period
        float elapsed = (time_end - time_start) / 1000.0f;
        float wait_ms = (dt * 1000.0f) - elapsed;
        int wait_ticks = (int)(wait_ms / portTICK_PERIOD_MS);
        if (wait_ticks < 1) wait_ticks = 1; // Ensure at least 1 tick
        vTaskDelay(wait_ticks);
    }
}

void app_main(void)
{
    gait_framework_main(NULL);
}