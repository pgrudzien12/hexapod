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
#include "robot_config_fat.h"
#include "user_command.h"
#include "controller.h"
#include <string.h>
#include "wifi_ap.h"
#include "controller_bt_classic.h"

static const char *TAG = "main";

// Configuration auto-save task
void config_autosave_task(void *arg) {
    const TickType_t save_interval = pdMS_TO_TICKS(30000); // Save every 30 seconds if dirty
    
    while (1) {
        vTaskDelay(save_interval);
        
        if (robot_config_needs_save()) {
            ESP_LOGI(TAG, "Auto-saving configuration");
            esp_err_t err = robot_config_save_to_nvs();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Auto-save failed: %s", esp_err_to_name(err));
            }
        }
    }
}

// Early system initialization
static esp_err_t system_early_init(void) {
    ESP_LOGI(TAG, "Starting system early initialization");
    
    // Initialize FAT filesystem first
    esp_err_t err = robot_config_fat_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FAT filesystem: %s", esp_err_to_name(err));
        return err;
    }
    
    // Check if migration is needed
    system_config_t system;
    err = robot_config_fat_load_system(&system);
    if (err == ESP_OK) {
        if (system.config_version < CONFIG_VERSION_CURRENT) {
            ESP_LOGI(TAG, "Configuration migration needed (v%lu -> v%d)", system.config_version, CONFIG_VERSION_CURRENT);
            err = robot_config_fat_migrate_config(system.config_version, CONFIG_VERSION_CURRENT);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Configuration migration failed: %s", esp_err_to_name(err));
                ESP_LOGW(TAG, "Performing factory reset due to migration failure");
                robot_config_fat_factory_reset(RESET_FACTORY);
            }
        }
    }
    
    // Initialize robot configuration (will load from FAT or use defaults)
    robot_config_init_default();
    
    // Validate configuration
    err = robot_config_fat_validate_config();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Configuration validation failed (%s), performing factory reset", esp_err_to_name(err));
        robot_config_factory_reset();
        
        // Re-validate after factory reset
        err = robot_config_fat_validate_config();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Configuration still invalid after factory reset: %s", esp_err_to_name(err));
        }
    }
    
    // Additional application-level validation
    if (!robot_config_is_valid()) {
        ESP_LOGW(TAG, "Application-level validation failed, performing factory reset");
        robot_config_factory_reset();
    }
    
    // Save configuration if it was loaded from defaults or modified
    if (robot_config_needs_save()) {
        ESP_LOGI(TAG, "Saving initial configuration to FAT filesystem");
        err = robot_config_save_to_nvs(); // Note: function name kept for API compatibility
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save initial config: %s", esp_err_to_name(err));
        }
    }
    
    // Print configuration summary
    robot_config_print_summary();
    robot_config_fat_print_file_tree();
    
    ESP_LOGI(TAG, "System early initialization completed");
    return ESP_OK;
}

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
    // robot_config_init_default(); // Already done in app_main()
    // user_command_init();
    
    // Start configuration auto-save task
    xTaskCreate(config_autosave_task, "config_save", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started configuration auto-save task");
    
    // Initialize Bluetooth Classic controller driver
    controller_bt_classic_cfg_t bt_cfg = controller_bt_classic_default();
    controller_config_t ctrl_cfg = {
        .driver_type = CONTROLLER_DRIVER_BT_CLASSIC,
        .task_stack = 4096,
        .task_prio = 10,
        .driver_cfg = &bt_cfg,
        .driver_cfg_size = sizeof(bt_cfg)
    };
    controller_init(&ctrl_cfg);

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
        // while(true)
        //     for (int i = 0; i < 3; ++i) {
        //         // Scale joint commands by step_scale (0..1)
        //         for (int j = 0; j < 6; ++j) {
        //             ESP_LOGI(TAG, "Leg %d Joint %d", j, i);
        //             // do 20 angle sweep on the joint
        //             for (int k = 0; k < 20; ++k) {
        //                 cmds.joint_cmds[j].joint_angles[i] = k * (M_PI / 180.0f);
        //                 robot_execute(&cmds);
        //                 vTaskDelay(50 / portTICK_PERIOD_MS);
        //             }
                    
        //             vTaskDelay(200 / portTICK_PERIOD_MS);
        //         }
        //         vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     }
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
    ESP_LOGI(TAG, "Hexapod Robot Starting...");
    
    // Early system initialization (NVS, configuration)
    esp_err_t err = system_early_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Early initialization failed, continuing with defaults");
    }
    
    // Bring up WiFi AP early so that network-based controller drivers or diagnostics
    // can connect even if later initialization stalls. Uses default options (MAC suffix).
    // wifi_ap_init_once();
    
    ESP_LOGI(TAG, "Starting gait framework");
    gait_framework_main(NULL);
}