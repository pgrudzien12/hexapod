/*
 * KPP System Core Implementation
 * 
 * Implements initialization, state estimation, and motion limiting.
 * 
 * License: Apache-2.0
 */

#include "kpp_system.h"
#include "kpp_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "kpp_system";

// Forward declarations for internal functions
static esp_err_t kpp_limit_joint_motion(const kinematic_state_t* state, const motion_limits_t* limits,
                                       int leg, int joint, float desired_angle, float* limited_angle, float dt);
static float kpp_apply_exponential_filter(float new_value, float old_value, float alpha);
static float kpp_constrain_f(float value, float min_val, float max_val);
static void kpp_validate_velocity_estimates(kinematic_state_t* state);

// KPP state update helper functions
static void kpp_update_joint_state(kinematic_state_t* state, const whole_body_cmd_t* current_cmd, float dt);
static void kpp_update_leg_velocities(kinematic_state_t* state, float dt);
static void kpp_update_body_pose(kinematic_state_t* state);
static void kpp_update_body_velocities(kinematic_state_t* state, float dt);
static void kpp_log_state(const kinematic_state_t* state);

esp_err_t kpp_init(kinematic_state_t* state, motion_limits_t* limits)
{
    if (!state || !limits) {
        ESP_LOGE(TAG, "Null pointer in kpp_init");
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize kinematic state
    memset(state, 0, sizeof(kinematic_state_t));
    state->initialized = false;
    state->last_update_time = 0.0f;

    // Initialize motion limits with configuration values
    limits->max_velocity[0] = KPP_MAX_VELOCITY_COXA;
    limits->max_velocity[1] = KPP_MAX_VELOCITY_FEMUR;
    limits->max_velocity[2] = KPP_MAX_VELOCITY_TIBIA;
    
    limits->max_acceleration[0] = KPP_MAX_ACCELERATION_COXA;
    limits->max_acceleration[1] = KPP_MAX_ACCELERATION_FEMUR;
    limits->max_acceleration[2] = KPP_MAX_ACCELERATION_TIBIA;
    
    limits->max_jerk[0] = KPP_MAX_JERK_COXA;
    limits->max_jerk[1] = KPP_MAX_JERK_FEMUR;
    limits->max_jerk[2] = KPP_MAX_JERK_TIBIA;
    
    limits->current_mode = MOTION_MODE_NORMAL;

    ESP_LOGI(TAG, "KPP system initialized");
    ESP_LOGI(TAG, "Motion limits - vel: %.1f rad/s, accel: %.1f rad/s², jerk: %.1f rad/s³", 
             limits->max_velocity[0], limits->max_acceleration[0], limits->max_jerk[0]);

    return ESP_OK;
}

void kpp_update_state(kinematic_state_t* state, const whole_body_cmd_t* current_cmd, float dt)
{
    assert(state != NULL);
    assert(current_cmd != NULL);
    assert(dt > 0);

    // Validate and clamp time step
    if (dt < KPP_MIN_DT || dt > KPP_MAX_DT) {
        ESP_LOGW(TAG, "Invalid dt: %.6f, clamping to valid range", dt);
        dt = kpp_constrain_f(dt, KPP_MIN_DT, KPP_MAX_DT);
    }

    // Update joint angles, velocities, and accelerations
    kpp_update_joint_state(state, current_cmd, dt);
    
    // Update leg positions (forward kinematics) and leg velocities
    kpp_update_leg_velocities(state, dt);
    
    // Estimate body pose from leg positions
    kpp_update_body_pose(state);
    
    // Calculate body velocities from pose changes
    kpp_update_body_velocities(state, dt);
    
    // Validate and constrain all velocity estimates
    kpp_validate_velocity_estimates(state);
    
    // Log state information for debugging
    kpp_log_state(state);
}

void kpp_apply_limits(const kinematic_state_t* state, const motion_limits_t* limits, 
                          const whole_body_cmd_t* desired_cmd, whole_body_cmd_t* limited_cmd, float dt)
{
    assert(state != NULL);
    assert(limits != NULL);
    assert(desired_cmd != NULL);
    assert(limited_cmd != NULL);

    if (!state->initialized) {
        ESP_LOGW(TAG, "KPP state not initialized, passing through commands");
        *limited_cmd = *desired_cmd;
        return;
    }

    // Validate time step
    if (dt < KPP_MIN_DT || dt > KPP_MAX_DT) {
        ESP_LOGW(TAG, "Invalid dt in apply_limits: %.6f", dt);
        dt = kpp_constrain_f(dt, KPP_MIN_DT, KPP_MAX_DT);
    }

    // Apply motion limiting to each joint
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            float desired_angle = desired_cmd->joint_cmds[leg].joint_angles[joint];
            float limited_angle;
            
            esp_err_t ret = kpp_limit_joint_motion(state, limits, leg, joint, desired_angle, &limited_angle, dt);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Motion limiting failed for leg %d joint %d", leg, joint);
                limited_angle = desired_angle; // Fallback to desired angle
            }
            
            limited_cmd->joint_cmds[leg].joint_angles[joint] = limited_angle;
        }
    }

#if KPP_ENABLE_LIMIT_LOGGING
    static int limit_log_counter = 0;
    if (++limit_log_counter >= KPP_LOG_INTERVAL * 2) {
        // Log example of limiting effect
        float desired = desired_cmd->joint_cmds[0].joint_angles[0];
        float limited = limited_cmd->joint_cmds[0].joint_angles[0];
        float diff = fabsf(desired - limited);
        if (diff > 0.01f) { // Only log if significant limiting occurred
            ESP_LOGD(TAG, "Motion limiting: leg0_coxa desired=%.3f limited=%.3f diff=%.3f", 
                     desired, limited, diff);
        }
        limit_log_counter = 0;
    }
#endif
}

esp_err_t kpp_set_motion_mode(motion_limits_t* limits, motion_mode_t mode)
{
    if (!limits) {
        ESP_LOGE(TAG, "Null pointer in kpp_set_motion_mode");
        return ESP_ERR_INVALID_ARG;
    }

    if (mode != MOTION_MODE_NORMAL) {
        ESP_LOGW(TAG, "Only MOTION_MODE_NORMAL currently supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    limits->current_mode = mode;
    ESP_LOGI(TAG, "Motion mode set to: %d", mode);
    
    return ESP_OK;
}

esp_err_t kpp_get_limits(const motion_limits_t* limits, int joint, 
                        float* max_vel, float* max_accel, float* max_jerk)
{
    if (!limits || !max_vel || !max_accel || !max_jerk) {
        ESP_LOGE(TAG, "Null pointer in kpp_get_limits");
        return ESP_ERR_INVALID_ARG;
    }

    if (joint < 0 || joint >= 3) {
        ESP_LOGE(TAG, "Invalid joint index: %d", joint);
        return ESP_ERR_INVALID_ARG;
    }

    *max_vel = limits->max_velocity[joint];
    *max_accel = limits->max_acceleration[joint];
    *max_jerk = limits->max_jerk[joint];

    return ESP_OK;
}

// =============================================================================
// KPP State Update Helper Functions
// =============================================================================

static void kpp_update_joint_state(kinematic_state_t* state, const whole_body_cmd_t* current_cmd, float dt)
{
    // Store previous values for differentiation
    float prev_angles[NUM_LEGS][3];
    float prev_velocities[NUM_LEGS][3];
    
    if (state->initialized) {
        memcpy(prev_angles, state->joint_angles, sizeof(prev_angles));
        memcpy(prev_velocities, state->joint_velocities, sizeof(prev_velocities));
    }

    // Update joint angles from commands
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            state->joint_angles[leg][joint] = current_cmd->joint_cmds[leg].joint_angles[joint];
        }
    }

    if (!state->initialized) {
        // First update - zero velocities and accelerations
        memset(state->joint_velocities, 0, sizeof(state->joint_velocities));
        memset(state->joint_accelerations, 0, sizeof(state->joint_accelerations));
        state->initialized = true;
        
        ESP_LOGI(TAG, "KPP state initialization complete");
    } else {
        // Calculate velocities and accelerations using numerical differentiation
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                // Velocity estimation with filtering
                float raw_velocity = (state->joint_angles[leg][joint] - prev_angles[leg][joint]) / dt;
                state->joint_velocities[leg][joint] = kpp_apply_exponential_filter(
                    raw_velocity, state->joint_velocities[leg][joint], KPP_VELOCITY_FILTER_ALPHA);

                // Acceleration estimation with filtering
                float raw_acceleration = (state->joint_velocities[leg][joint] - prev_velocities[leg][joint]) / dt;
                state->joint_accelerations[leg][joint] = kpp_apply_exponential_filter(
                    raw_acceleration, state->joint_accelerations[leg][joint], KPP_ACCEL_FILTER_ALPHA);
            }
        }
    }
}

static void kpp_update_leg_velocities(kinematic_state_t* state, float dt)
{
    // Store previous leg positions for velocity calculation
    static float prev_leg_positions[NUM_LEGS][3];
    static bool prev_positions_valid = false;
    
    // Store current positions as previous for next iteration (before updating)
    if (state->initialized && prev_positions_valid) {
        // Calculate leg velocities from position differences
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int axis = 0; axis < 3; axis++) {
                // Raw velocity from numerical differentiation
                float raw_leg_velocity = (state->leg_positions[leg][axis] - prev_leg_positions[leg][axis]) / dt;
                
                // Apply low-pass filtering to reduce noise
                state->leg_velocities[leg][axis] = kpp_apply_exponential_filter(
                    raw_leg_velocity, state->leg_velocities[leg][axis], KPP_LEG_VELOCITY_FILTER_ALPHA);
            }
        }
    } else {
        // Initialize leg velocities to zero
        memset(state->leg_velocities, 0, sizeof(state->leg_velocities));
    }

    // Store current leg positions before updating them
    if (state->initialized) {
        memcpy(prev_leg_positions, state->leg_positions, sizeof(prev_leg_positions));
        prev_positions_valid = true;
    }

    // Update leg positions using forward kinematics
    esp_err_t ret = kpp_forward_kinematics(state->joint_angles, state->leg_positions);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Forward kinematics failed: %s", esp_err_to_name(ret));
    }
}

static void kpp_update_body_pose(kinematic_state_t* state)
{
    if (!state->initialized) {
        // Initialize body pose to zero
        memset(state->body_position, 0, sizeof(state->body_position));
        memset(state->body_orientation, 0, sizeof(state->body_orientation));
        return;
    }

    // Calculate body center of mass from leg positions
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        sum_x += state->leg_positions[leg][0];
        sum_y += state->leg_positions[leg][1]; 
        sum_z += state->leg_positions[leg][2];
    }
    
    // Average position (crude body position estimate)
    state->body_position[0] = sum_x / NUM_LEGS;
    state->body_position[1] = sum_y / NUM_LEGS;
    state->body_position[2] = sum_z / NUM_LEGS;
    
    // Simple body orientation estimation from leg height differences
    // This is a basic implementation - could be improved with better geometric analysis
    float front_height = (state->leg_positions[0][2] + state->leg_positions[1][2]) / 2.0f;  // Front legs
    float back_height = (state->leg_positions[4][2] + state->leg_positions[5][2]) / 2.0f;   // Back legs
    float left_height = (state->leg_positions[0][2] + state->leg_positions[2][2] + state->leg_positions[4][2]) / 3.0f;  // Left legs
    float right_height = (state->leg_positions[1][2] + state->leg_positions[3][2] + state->leg_positions[5][2]) / 3.0f; // Right legs
    
    // Estimate pitch (front-back tilt) and roll (left-right tilt)
    float pitch_estimate = atan2f(back_height - front_height, KPP_FRONT_TO_BACK_DISTANCE);
    float roll_estimate = atan2f(right_height - left_height, KPP_LEFT_TO_RIGHT_DISTANCE);
    
    // Apply filtering to orientation estimates
    state->body_orientation[0] = kpp_apply_exponential_filter(roll_estimate, state->body_orientation[0], KPP_BODY_ROLL_FILTER_ALPHA);   // Roll
    state->body_orientation[1] = kpp_apply_exponential_filter(pitch_estimate, state->body_orientation[1], KPP_BODY_PITCH_FILTER_ALPHA);  // Pitch
    // Yaw (state->body_orientation[2]) would require more sophisticated estimation or IMU data
}

static void kpp_update_body_velocities(kinematic_state_t* state, float dt)
{
    // Static variables to maintain state between calls
    static float prev_body_position[3] = {0};
    static float prev_body_orientation[3] = {0};
    static bool prev_data_valid = false;
    
    if (!state->initialized) {
        // Initialize body velocities to zero
        memset(state->body_velocity, 0, sizeof(state->body_velocity));
        memset(state->body_angular_vel, 0, sizeof(state->body_angular_vel));
        return;
    }
    
    if (prev_data_valid) {
        // Calculate body linear velocity from position change
        for (int axis = 0; axis < 3; axis++) {
            float raw_body_velocity = (state->body_position[axis] - prev_body_position[axis]) / dt;
            state->body_velocity[axis] = kpp_apply_exponential_filter(
                raw_body_velocity, state->body_velocity[axis], KPP_BODY_VELOCITY_FILTER_ALPHA);
        }
        
        // Calculate body angular velocity from orientation change
        for (int axis = 0; axis < 3; axis++) {
            float raw_angular_velocity = (state->body_orientation[axis] - prev_body_orientation[axis]) / dt;
            state->body_angular_vel[axis] = kpp_apply_exponential_filter(
                raw_angular_velocity, state->body_angular_vel[axis], KPP_BODY_VELOCITY_FILTER_ALPHA);
        }
    } else {
        // Initialize body velocities to zero
        memset(state->body_velocity, 0, sizeof(state->body_velocity));
        memset(state->body_angular_vel, 0, sizeof(state->body_angular_vel));
        prev_data_valid = true;
    }
    
    // Store current values for next iteration
    memcpy(prev_body_position, state->body_position, sizeof(prev_body_position));
    memcpy(prev_body_orientation, state->body_orientation, sizeof(prev_body_orientation));
}

static void kpp_log_state(const kinematic_state_t* state)
{
#if KPP_ENABLE_STATE_LOGGING
    static int log_counter = 0;
    if (++log_counter >= KPP_LOG_INTERVAL) {
        ESP_LOGD(TAG, "Joint State: leg0 angles=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] accel=[%.1f,%.1f,%.1f]",
                 state->joint_angles[0][0], state->joint_angles[0][1], state->joint_angles[0][2],
                 state->joint_velocities[0][0], state->joint_velocities[0][1], state->joint_velocities[0][2],
                 state->joint_accelerations[0][0], state->joint_accelerations[0][1], state->joint_accelerations[0][2]);
        
        ESP_LOGD(TAG, "Leg State: leg0 pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f]",
                 state->leg_positions[0][0], state->leg_positions[0][1], state->leg_positions[0][2],
                 state->leg_velocities[0][0], state->leg_velocities[0][1], state->leg_velocities[0][2]);
        
        ESP_LOGD(TAG, "Body State: pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f] orient=[%.2f,%.2f,%.2f] angvel=[%.2f,%.2f,%.2f]",
                 state->body_position[0], state->body_position[1], state->body_position[2],
                 state->body_velocity[0], state->body_velocity[1], state->body_velocity[2],
                 state->body_orientation[0]*180.0f/3.14159f, state->body_orientation[1]*180.0f/3.14159f, state->body_orientation[2]*180.0f/3.14159f,
                 state->body_angular_vel[0], state->body_angular_vel[1], state->body_angular_vel[2]);
        
        log_counter = 0;
    }
#endif
}

// =============================================================================
// Internal Helper Functions
// =============================================================================

static esp_err_t kpp_limit_joint_motion(const kinematic_state_t* state, const motion_limits_t* limits,
                                       int leg, int joint, float desired_angle, float* limited_angle, float dt)
{
    if (!state || !limits || !limited_angle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (leg < 0 || leg >= NUM_LEGS || joint < 0 || joint >= 3) {
        return ESP_ERR_INVALID_ARG;
    }

    float current_angle = state->joint_angles[leg][joint];
    float current_velocity = state->joint_velocities[leg][joint];
    float current_acceleration = state->joint_accelerations[leg][joint];

    // Calculate desired motion parameters
    float desired_velocity = (desired_angle - current_angle) / dt;
    float desired_acceleration = (desired_velocity - current_velocity) / dt;
    float desired_jerk = (desired_acceleration - current_acceleration) / dt;

    // Apply jerk limiting first (S-curve shaping)
    float limited_jerk = kpp_constrain_f(desired_jerk, -limits->max_jerk[joint], limits->max_jerk[joint]);
    float limited_acceleration = current_acceleration + limited_jerk * dt;

    // Apply acceleration limiting
    limited_acceleration = kpp_constrain_f(limited_acceleration, 
                                         -limits->max_acceleration[joint], limits->max_acceleration[joint]);

    // Apply velocity limiting
    float limited_velocity = current_velocity + limited_acceleration * dt;
    limited_velocity = kpp_constrain_f(limited_velocity, -limits->max_velocity[joint], limits->max_velocity[joint]);

    // Integrate to get final limited angle
    *limited_angle = current_angle + limited_velocity * dt;

    return ESP_OK;
}

static float kpp_apply_exponential_filter(float new_value, float old_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * old_value;
}

static float kpp_constrain_f(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static void kpp_validate_velocity_estimates(kinematic_state_t* state)
{
    // Validate and constrain leg velocities
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int axis = 0; axis < 3; axis++) {
            float vel = state->leg_velocities[leg][axis];
            if (fabsf(vel) > KPP_MAX_LEG_VELOCITY) {
                ESP_LOGW(TAG, "Clamping leg %d axis %d velocity from %.3f to %.3f", 
                         leg, axis, vel, copysignf(KPP_MAX_LEG_VELOCITY, vel));
                state->leg_velocities[leg][axis] = kpp_constrain_f(vel, -KPP_MAX_LEG_VELOCITY, KPP_MAX_LEG_VELOCITY);
            }
        }
    }
    
    // Validate and constrain body velocities
    for (int axis = 0; axis < 3; axis++) {
        float vel = state->body_velocity[axis];
        if (fabsf(vel) > KPP_MAX_BODY_VELOCITY) {
            ESP_LOGW(TAG, "Clamping body velocity axis %d from %.3f to %.3f", 
                     axis, vel, copysignf(KPP_MAX_BODY_VELOCITY, vel));
            state->body_velocity[axis] = kpp_constrain_f(vel, -KPP_MAX_BODY_VELOCITY, KPP_MAX_BODY_VELOCITY);
        }
        
        float ang_vel = state->body_angular_vel[axis];
        if (fabsf(ang_vel) > KPP_MAX_ANGULAR_VELOCITY) {
            ESP_LOGW(TAG, "Clamping body angular velocity axis %d from %.3f to %.3f", 
                     axis, ang_vel, copysignf(KPP_MAX_ANGULAR_VELOCITY, ang_vel));
            state->body_angular_vel[axis] = kpp_constrain_f(ang_vel, -KPP_MAX_ANGULAR_VELOCITY, KPP_MAX_ANGULAR_VELOCITY);
        }
    }
}