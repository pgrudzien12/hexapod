/*
 * Hexapod leg inverse kinematics (3-DOF) - pure IK API
 * Computes joint angles from a Cartesian foot target in the leg-local frame.
 * Hardware/servo driving is intentionally decoupled and lives in robot_control.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Example usage
// leg_geometry_t cfg = {
//     .len_coxa = 0.068f,
//     .len_femur = 0.088f,
//     .len_tibia = 0.1270f,
//     .coxa_offset_rad = 4*-0.017453292519943295f,
//     .femur_offset_rad = 0.5396943301595464f,
//     .tibia_offset_rad = 1.0160719600939494f,
// };


// leg_handle_t leg = NULL;
// ESP_ERROR_CHECK(leg_configure(&cfg, &leg));

// ESP_LOGI(TAG, "Leg configured on GPIOs %d (coxa), %d (femur), %d (tibia)", LEG_SERVO1_GPIO, LEG_SERVO2_GPIO, LEG_SERVO3_GPIO);
// vTaskDelay(pdMS_TO_TICKS(1500));

// ESP_ERROR_CHECK(leg_test_neutral(leg));
// Servos indices for a single 3-DOF leg (shared enum, also used by robot control)
typedef enum {
	LEG_SERVO_COXA = 0,
	LEG_SERVO_FEMUR = 1,
	LEG_SERVO_TIBIA = 2,
} leg_servo_t;

// Leg geometry and calibration structure (exposed for direct access)
typedef struct {
	// Geometry - link lengths in consistent units (meters)
	float len_coxa;   // distance from hip yaw joint to femur joint along the X axis
	float len_femur;  // thigh length
	float len_tibia;  // shank length

	// Servo calibration offsets (radians)
	float coxa_offset_rad;
	float femur_offset_rad;
	float tibia_offset_rad;
} leg_geometry_t;

// Leg handle - now explicit structure pointer instead of opaque
typedef leg_geometry_t* leg_handle_t;

// Create/configure a leg and return a descriptor via out_leg
esp_err_t leg_configure(const leg_geometry_t* cfg, leg_handle_t* out_leg);

// IK output angles in radians
typedef struct {
	float coxa;   // hip yaw
	float femur;  // hip pitch
	float tibia;  // knee/ankle
} leg_angles_t;

// Inverse kinematics: compute joint angles for a foot target in leg-local coordinates.
// Coordinate frame:
//  - XY plane is the ground plane
//  - Z axis points downward (increases toward ground)
//  - X points outward from the robot body (to the side)
//  - Y points forward
// Units for x/y/z should match the leg lengths provided in leg_geometry_t.
// Returns ESP_OK on success and writes angles to out_angles (no clamping/offsets applied).
esp_err_t leg_ik_solve(leg_handle_t leg, float x, float y, float z, leg_angles_t *out_angles);

// Deprecated: kept for source-compatibility with old demos. This no longer drives servos.
static inline esp_err_t leg_move_xyz(leg_handle_t leg, float x, float y, float z) {
	leg_angles_t tmp;
	return leg_ik_solve(leg, x, y, z, &tmp);
}

#ifdef __cplusplus
}
#endif
