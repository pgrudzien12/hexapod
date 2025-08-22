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

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             2        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Define the three servo pins for the leg here
#define LEG_SERVO1_GPIO 13
#define LEG_SERVO2_GPIO 12
#define LEG_SERVO3_GPIO 14

void app_main(void)
{
    // Configure leg on pins 13, 12, 14 and set neutral
    ESP_ERROR_CHECK(leg_configure(LEG_SERVO1_GPIO, LEG_SERVO2_GPIO, LEG_SERVO3_GPIO));
    ESP_ERROR_CHECK(leg_test_neutral());

    // Keep idle task alive; nothing else to do yet
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
