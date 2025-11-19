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
#include "wifi_ap.h"
#include "controller_bt_classic.h"
#include "controller_wifi_tcp.h"
#include "kpp_system.h"
#include "kpp_debug.h"
#include "config_manager.h"
#include "rpc_commands.h"

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

    // Initialize KPP (Kinematic Pose Position) system
    kinematic_state_t kpp_state;
    motion_limits_t motion_limits;
    ESP_ERROR_CHECK(kpp_init(&kpp_state, &motion_limits));
    ESP_LOGI(TAG, "KPP system initialized with motion limiting enabled");

    // Initialize gait framework modules
    gait_scheduler_init(&scheduler, 1.5f); // 1.5 second cycle time
    swing_trajectory_init(&trajectory, 0.07f, 0.04f); // 7cm step, 4cm clearance

    const float dt = 0.01f; // 10ms loop
    while (1) {
        float time_start = esp_timer_get_time();
        // Copy current to previous then poll new
        prev_cmd = ucmd;
        user_command_poll(&ucmd);
        if (!controller_user_command_equal(&prev_cmd, &ucmd, 1e-2f)) {
            ESP_LOGI(TAG, "User command: vx=%.2f, wz=%.2f, z_target=%.2f, y_offset=%.2f gait=%d enable=%d pose=%d terrain=%d step_scale=%.2f",
                    ucmd.vx, ucmd.wz, ucmd.z_target, ucmd.y_offset, ucmd.gait, ucmd.enable, ucmd.pose_mode, ucmd.terrain_climb, ucmd.step_scale);
        }

        // Update gait scheduler (leg phases)
        gait_scheduler_update(&scheduler, dt, &ucmd);
        // Generate swing trajectories for legs using scheduler + command
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);
        // Compute joint commands from trajectories
        whole_body_control_compute(&trajectory, &cmds);
        
        // KPP: Apply motion limiting for smooth servo operation
        whole_body_cmd_t limited_cmds;
        kpp_apply_limits(&kpp_state, &motion_limits, &cmds, &limited_cmds, dt);
        
        // Send limited commands to robot
        robot_execute(&limited_cmds);
        
        // CRITICAL FIX: Update state estimation based on ORIGINAL commands, not limited ones
        // This breaks the feedback loop that was causing oscillations
        kpp_update_state(&kpp_state, &cmds, dt);  // Use original commands for state estimation
        
        // Debug monitoring (only in debug builds)
        kpp_debug_monitor_limits(&kpp_state, &cmds, &limited_cmds, dt);        
        
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
    // Initialize configuration manager first
    ESP_LOGI(TAG, "Starting hexapod application...");
    esp_err_t err = config_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize configuration manager: %s", esp_err_to_name(err));
        return;
    }
    
    // Get system configuration
    const system_config_t* sys_config = config_get_system();
    ESP_LOGI(TAG, "System config loaded: robot_id=%s, robot_name=%s", 
             sys_config->robot_id, sys_config->robot_name);
    ESP_LOGI(TAG, "Safety settings: emergency_stop=%s, voltage_min=%.2fV", 
             sys_config->emergency_stop_enabled ? "enabled" : "disabled",
             sys_config->safety_voltage_min);
    
    // Apply startup delay from configuration
    if (sys_config->startup_delay_ms > 0) {
        ESP_LOGI(TAG, "Applying startup delay: %lu ms", (unsigned long)sys_config->startup_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(sys_config->startup_delay_ms));
    }
    
    // Initialize RPC system
    rpc_init();
    
    // Bring up WiFi AP early so that network-based controller drivers or diagnostics
    // can connect even if later initialization stalls. Uses default options (MAC suffix).
    wifi_ap_init_once();
    
    // Initialize robot configuration
    robot_config_init_default();
    
    // Initialize the primary controller from system configuration
    controller_config_t ctrl_cfg = {
        .driver_type = sys_config->controller_type,
        .task_stack = 4096,
        .task_prio = 10,
    };

    // Assign driver-specific configuration if needed
    controller_wifi_tcp_cfg_t wifi_cfg;
    if (ctrl_cfg.driver_type == CONTROLLER_DRIVER_WIFI_TCP) {
        wifi_cfg = controller_wifi_tcp_default();
        ctrl_cfg.driver_cfg = &wifi_cfg;
        ctrl_cfg.driver_cfg_size = sizeof(wifi_cfg);
    }
    
    controller_init(&ctrl_cfg);

    // ALWAYS initialize WiFi TCP controller for RPC commands (separate from primary controller)
    // This provides network-based diagnostics and control regardless of primary controller type
    if (ctrl_cfg.driver_type != CONTROLLER_DRIVER_WIFI_TCP) {
        ESP_LOGI(TAG, "Initializing WiFi TCP controller as secondary RPC interface");
        controller_wifi_tcp_cfg_t wifi_rpc_cfg = controller_wifi_tcp_default();
        controller_config_t wifi_ctrl_cfg = {
            .driver_type = CONTROLLER_DRIVER_WIFI_TCP,
            .task_stack = 4096,
            .task_prio = 8,  // Lower priority than primary controller
            .driver_cfg = &wifi_rpc_cfg,
            .driver_cfg_size = sizeof(wifi_rpc_cfg),
        };
        controller_driver_init_wifi_tcp(&wifi_ctrl_cfg);
    }
    
    gait_framework_main(NULL);
}