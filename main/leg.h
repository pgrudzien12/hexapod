/*
 * Hexapod leg control (3-DOF) - minimal API
 * Configures three MCPWM-driven hobby servos and provides a test to set them to neutral.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Servos indices for a single 3-DOF leg
typedef enum {
	LEG_SERVO_COXA = 0,
	LEG_SERVO_FEMUR = 1,
	LEG_SERVO_TIBIA = 2,
} leg_servo_t;

// Initialize the leg servos on the given GPIO pins.
// Returns ESP_OK on success.
esp_err_t leg_configure(int servo1_gpio, int servo2_gpio, int servo3_gpio);

// Drive all three servos to their neutral position (1.5ms pulse, ~0 deg).
esp_err_t leg_test_neutral(void);

// Set angle for a specific joint in radians (clamped to supported range).
// Returns ESP_ERR_INVALID_STATE if leg is not configured yet.
esp_err_t leg_set_angle_rad(leg_servo_t joint, float radians);

#ifdef __cplusplus
}
#endif
