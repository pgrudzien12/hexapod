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

static const char *TAG = "leg";

// Define the three servo pins for the leg here
#define LEG_SERVO1_GPIO 13
#define LEG_SERVO2_GPIO 12
#define LEG_SERVO3_GPIO 14

float deg_to_rad(float degrees) {
    return degrees * (M_PI / 180.0f);
}
void move_leg(leg_handle_t leg, float x, float y, float z, int delay) {
    ESP_LOGI(TAG, "Move XYZ -> x=%.1f y=%.1f z=%.1f", x, y, z);
    ESP_ERROR_CHECK(leg_move_xyz(leg, x, y, z));
    vTaskDelay(pdMS_TO_TICKS(delay));
}

// Draw a circle in the XY plane at fixed Z
void circle_xy(leg_handle_t leg,
               float x_center,
               float y_center,
               float z_fixed,
               float radius_mm,
               int steps,
               int delay_ms)
{
    if (steps < 3) steps = 3;
    ESP_LOGI(TAG, "Circle XY: center(%.1f, %.1f, %.1f) R=%.1f steps=%d", x_center, y_center, z_fixed, radius_mm, steps);
    for (int i = 0; i < steps; ++i) {
        float a = (2.0f * (float)M_PI * (float)i) / (float)steps;
        float x = x_center + radius_mm * cosf(a);
        float y = y_center + radius_mm * sinf(a);
        move_leg(leg, x, y, z_fixed, delay_ms);
    }
}

// Draw a circle in the XZ plane at fixed Y
void circle_xz(leg_handle_t leg,
               float x_center,
               float y_fixed,
               float z_center,
               float radius_mm,
               int steps,
               int delay_ms)
{
    if (steps < 3) steps = 3;
    ESP_LOGI(TAG, "Circle XZ: center(%.1f, %.1f, %.1f) R=%.1f steps=%d", x_center, y_fixed, z_center, radius_mm, steps);
    for (int i = 0; i < steps; ++i) {
        float a = (2.0f * (float)M_PI * (float)i) / (float)steps;
        float x = x_center + radius_mm * cosf(a);
        float z = z_center + radius_mm * sinf(a); // +Z is down
        move_leg(leg, x, y_fixed, z, delay_ms);
    }
}

// Draw a circle in the YZ plane at fixed X
void circle_yz(leg_handle_t leg,
               float x_fixed,
               float y_center,
               float z_center,
               float radius_mm,
               int steps,
               int delay_ms)
{
    if (steps < 3) steps = 3;
    ESP_LOGI(TAG, "Circle YZ: center(%.1f, %.1f, %.1f) R=%.1f steps=%d", x_fixed, y_center, z_center, radius_mm, steps);
    for (int i = 0; i < steps; ++i) {
        float a = (2.0f * (float)M_PI * (float)i) / (float)steps;
        float y = y_center + radius_mm * cosf(a);
        float z = z_center + radius_mm * sinf(a); // +Z is down
        move_leg(leg, x_fixed, y, z, delay_ms);
    }
}
void app_main(void)
{
// Configure leg on pins 13, 12, 14 with placeholder lengths (e.g., mm)
    leg_config_t cfg = {
        .gpio_coxa = LEG_SERVO1_GPIO,
        .gpio_femur = LEG_SERVO2_GPIO,
        .gpio_tibia = LEG_SERVO3_GPIO,
        .len_coxa = 68.0f,
        .len_femur = 88.0f,
        .len_tibia = 127.0f,
        .group_id = 0,
        // Allow tibia to move negative down to -90 deg, with max at +16 deg (both in radians)
        .min_rad_tibia = deg_to_rad(-90.0f),
        .max_rad_tibia = deg_to_rad(16.0f),
    // Servo calibration offsets (radians)
    .coxa_offset_rad = 4*-0.017453292519943295f,
    .femur_offset_rad = 0.5396943301595464f,
    .tibia_offset_rad = 1.0160719600939494f,
    };


    leg_handle_t leg = NULL;
    ESP_ERROR_CHECK(leg_configure(&cfg, &leg));

    ESP_LOGI(TAG, "Leg configured on GPIOs %d (coxa), %d (femur), %d (tibia)", LEG_SERVO1_GPIO, LEG_SERVO2_GPIO, LEG_SERVO3_GPIO);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_ERROR_CHECK(leg_test_neutral(leg));
    vTaskDelay(pdMS_TO_TICKS(1500));
    // X-axis only sweep using IK: keep Y=0, Z fixed; move X back and forth
    const float X_FIXED = 200.0f;
    const float Y_FIXED = 0.0f;
    const float Z_FIXED = 0.0f;
    // ESP_LOGI(TAG, "Move XYZ -> x=%.1f y=%.1f z=%.1f", 450.0f, Y_FIXED, Z_FIXED);
    // ESP_ERROR_CHECK(leg_move_xyz(leg, 450.0f, Y_FIXED, Z_FIXED));
    // vTaskDelay(pdMS_TO_TICKS(1500));
    // ESP_LOGI(TAG, "Set to zero angles");
    // ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_COXA, deg_to_rad(0.0f)));
    // ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_FEMUR, deg_to_rad(-0.0f)));
    // ESP_ERROR_CHECK(leg_set_angle_rad(leg, LEG_SERVO_TIBIA, deg_to_rad(-0.0f)));

    move_leg(leg, 200.0f, Y_FIXED, Z_FIXED, 1);

    // ESP_LOGI(TAG, "Test X axis");
    // move_leg(leg, 200.0f, Y_FIXED, Z_FIXED, 1);
    // move_leg(leg, 161.0f, Y_FIXED, Z_FIXED, 1);
    // move_leg(leg, 250.0f, Y_FIXED, Z_FIXED, 1);

    // ESP_LOGI(TAG, "Test Y axis");
    // for (float y = 0.0f; y <= 30.0f; y += 10.0f) {
    //     move_leg(leg, X_FIXED, y, Z_FIXED, 1);
    // }

    // ESP_LOGI(TAG, "Test Z axis");
    // for (float z = -30.0f; z <= 0.0f; z += 10.0f) {
    //     move_leg(leg, X_FIXED, Y_FIXED, z, 1);
    // }

    // move_leg(leg, 200.0f, 0.0f, 50.0f, 1);

    // swipe in x axis
    int delay = 25;
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // for (float z = -30.0f; z <= 30.0f; z += 10.0f) {
        //     for (float x = 150.0f; x <= 250.0f; x += 2.0f) {
        //         ESP_LOGI(TAG, "Move XYZ -> x=%.1f y=%.1f z=%.1f", x, Y_FIXED, z);
        //         ESP_ERROR_CHECK(leg_move_xyz(leg, x, Y_FIXED, z));
        //         vTaskDelay(pdMS_TO_TICKS(delay));
        //     }
        //     for (float x = 250.0f; x >= 150.0f; x -= 2.0f) {
        //         ESP_LOGI(TAG, "Move XYZ -> x=%.1f y=%.1f z=%.1f", x, Y_FIXED, z);
        //         ESP_ERROR_CHECK(leg_move_xyz(leg, x, Y_FIXED, z));
        //         vTaskDelay(pdMS_TO_TICKS(delay));
        //     }
        //     vTaskDelay(pdMS_TO_TICKS(500));
        // }
        
        circle_xy(leg, 200.0f, 0.0f, 50.0f, 30.0f, 80, 25);
        vTaskDelay(pdMS_TO_TICKS(1000));
        circle_xz(leg, 200.0f, 0.0f, 50.0f, 30.0f, 80, 25);
        vTaskDelay(pdMS_TO_TICKS(1000));
        circle_yz(leg, 230.0f, -30.0f, 50.0f, 30.0f, 80, 25);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
