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

static const char *TAG = "example";

// Define the three servo pins for the leg here
#define LEG_SERVO1_GPIO 13
#define LEG_SERVO2_GPIO 12
#define LEG_SERVO3_GPIO 14

void app_main(void)
{
    // Configure leg on pins 13, 12, 14 with placeholder lengths (e.g., mm)
    leg_config_t cfg = {
        .gpio_coxa = LEG_SERVO1_GPIO,
        .gpio_femur = LEG_SERVO2_GPIO,
        .gpio_tibia = LEG_SERVO3_GPIO,
        .len_coxa = 63.0f,
        .len_femur = 89.0f,
        .len_tibia = 126.0f,
        .group_id = 0,
    };
    leg_handle_t leg = NULL;
    ESP_ERROR_CHECK(leg_configure(&cfg, &leg));
    ESP_ERROR_CHECK(leg_test_neutral(leg));

    float angle = -1.57f; // -90 degrees in radians
    float angle_change = 0.1f; // ~6 degrees in radians
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        angle += angle_change;
        ESP_LOGI(TAG, "Setting leg servos to angle: %.2f rad", angle);
        ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_COXA, angle));
        ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_FEMUR, angle));
        ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_TIBIA, angle));
        if (angle > 1.57f || angle < -1.57f) {
            angle_change = -angle_change;
        }
    }
}
