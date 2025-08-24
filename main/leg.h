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

// Opaque leg descriptor
typedef struct leg_s* leg_handle_t;

// Configuration for a leg: GPIOs and segment lengths (units up to you, e.g., mm)
typedef struct {
	int gpio_coxa;
	int gpio_femur;
	int gpio_tibia;
	float len_coxa;
	float len_femur;
	float len_tibia;
	int group_id; // MCPWM group to use (default 0)
	// Optional joint angle limits in radians (servo space). If both min and max are 0,
	// defaults to [-pi/2, +pi/2] for that joint.
	float min_rad_coxa;
	float max_rad_coxa;
	float min_rad_femur;
	float max_rad_femur;
	float min_rad_tibia;
	float max_rad_tibia;
	// Optional per-joint servo offsets in radians (added to computed geometry before clamping)
	// Use these to calibrate neutral/zero positions. Defaults to 0 if not set.
	float femur_offset_rad; // shoulder offset
	float tibia_offset_rad; // knee offset
} leg_config_t;

// Create/configure a leg and return a descriptor via out_leg
esp_err_t leg_configure(const leg_config_t* cfg, leg_handle_t* out_leg);

// Drive all three servos to their neutral position (1.5ms pulse, ~0 deg).
esp_err_t leg_test_neutral(leg_handle_t leg);

// Set angle for a specific joint in radians (clamped to supported range).
// Returns ESP_ERR_INVALID_STATE if leg is not configured yet.
esp_err_t leg_set_angle_rad(leg_handle_t leg, leg_servo_t joint, float radians);

// Inverse kinematics entry: set a foot target in leg-local coordinates.
// Coordinate frame:
//  - XY plane is the ground plane
//  - Z axis points downward (increases toward ground)
//  - X points outward from the robot body (to the side)
//  - Y points forward
// Units for x/y/z should match the leg lengths provided in leg_config_t.
// This computes joint angles and commands servos. Returns ESP_OK on success.
esp_err_t leg_move_xyz(leg_handle_t leg, float x, float y, float z);

#ifdef __cplusplus
}
#endif
