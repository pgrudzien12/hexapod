/*
 * Kinematic Pose Position (KPP) System
 * 
 * Provides state estimation and motion limiting for hexapod robot.
 * Features:
 * - Joint state tracking (position, velocity, acceleration)
 * - Forward kinematics (joint angles -> leg positions)
 * - Motion limiting with acceleration and jerk control
 * - S-curve motion profiles for smooth servo operation
 * 
 * License: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "whole_body_control.h"
#include "robot_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Motion mode for different operating conditions
typedef enum {
    MOTION_MODE_NORMAL,      // S-curve jerk-limited, focus on acceleration/jerk control
    // TODO: Add MOTION_MODE_FAST and MOTION_MODE_EMERGENCY later
} motion_mode_t;

// Complete kinematic state of the robot
typedef struct {
    // Joint state (6 legs × 3 joints each)
    float joint_angles[NUM_LEGS][3];      // Current angles [leg][joint] (rad)
    float joint_velocities[NUM_LEGS][3];  // Angular velocities (rad/s)
    float joint_accelerations[NUM_LEGS][3]; // Angular accelerations (rad/s²)
    
    // Leg endpoint positions in body frame
    float leg_positions[NUM_LEGS][3];     // [leg][x,y,z] (m)
    float leg_velocities[NUM_LEGS][3];    // [leg][vx,vy,vz] (m/s)
    
    // Body state (estimated from leg positions)
    float body_position[3];        // x, y, z in world frame
    float body_orientation[3];     // roll, pitch, yaw (rad)
    float body_velocity[3];        // linear velocity (m/s)
    float body_angular_vel[3];     // angular velocity (rad/s)
    
    // Timing and initialization
    float last_update_time;        // For dt calculation (us)
    bool initialized;              // First update flag
} kinematic_state_t;

// Motion limits for smooth servo operation
typedef struct {
    // Motion limits per joint type [coxa, femur, tibia]
    float max_velocity[3];         // rad/s
    float max_acceleration[3];     // rad/s²
    float max_jerk[3];            // rad/s³
    
    motion_mode_t current_mode;
    // TODO: Add mode_transition_time for future multi-mode support
} motion_limits_t;

/**
 * @brief Initialize KPP system with default parameters
 * 
 * @param state Kinematic state structure to initialize
 * @param limits Motion limits structure to initialize
 * @return esp_err_t ESP_OK on success
 */
esp_err_t kpp_init(kinematic_state_t* state, motion_limits_t* limits);

/**
 * @brief Update kinematic state from current servo commands
 * 
 * Estimates current robot state based on commanded joint positions.
 * Calculates velocities and accelerations using numerical differentiation.
 * 
 * @param state Kinematic state to update
 * @param current_cmd Current joint commands being executed
 * @param dt Time step since last update (seconds)
 */
void kpp_update_state(kinematic_state_t* state, const whole_body_cmd_t* current_cmd, float dt);

/**
 * @brief Apply motion limits to desired joint commands
 * 
 * Applies acceleration and jerk limiting to smooth motion and reduce servo stress.
 * Uses S-curve motion profiles for natural movement.
 * 
 * @param state Current kinematic state
 * @param limits Motion limits to apply
 * @param desired_cmd Desired joint commands (input)
 * @param limited_cmd Motion-limited commands (output)
 * @param dt Time step for integration (seconds)
 */
void kpp_apply_limits(const kinematic_state_t* state, const motion_limits_t* limits, 
                          const whole_body_cmd_t* desired_cmd, whole_body_cmd_t* limited_cmd, float dt);

/**
 * @brief Compute leg positions from joint angles (forward kinematics)
 * 
 * @param joint_angles Joint angles [leg][joint] in radians
 * @param leg_positions Output leg positions [leg][x,y,z] in body frame
 * @return esp_err_t ESP_OK on success
 */
esp_err_t kpp_forward_kinematics(const float joint_angles[NUM_LEGS][3], float leg_positions[NUM_LEGS][3]);

/**
 * @brief Set motion mode (for future multi-mode support)
 * 
 * @param limits Motion limits structure to modify
 * @param mode New motion mode
 * @return esp_err_t ESP_OK on success
 */
esp_err_t kpp_set_motion_mode(motion_limits_t* limits, motion_mode_t mode);

/**
 * @brief Get current motion limits for debugging
 * 
 * @param limits Motion limits structure
 * @param joint Joint type (0=coxa, 1=femur, 2=tibia)
 * @param max_vel Output maximum velocity (rad/s)
 * @param max_accel Output maximum acceleration (rad/s²)
 * @param max_jerk Output maximum jerk (rad/s³)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t kpp_get_limits(const motion_limits_t* limits, int joint, 
                        float* max_vel, float* max_accel, float* max_jerk);

#ifdef __cplusplus
}
#endif