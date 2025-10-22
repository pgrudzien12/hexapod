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

    // Validate time step
    if (dt < KPP_MIN_DT || dt > KPP_MAX_DT) {
        ESP_LOGW(TAG, "Invalid dt: %.6f, clamping to valid range", dt);
        dt = kpp_constrain_f(dt, KPP_MIN_DT, KPP_MAX_DT);
    }

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

    // Update leg positions using forward kinematics
    esp_err_t ret = kpp_forward_kinematics(state->joint_angles, state->leg_positions);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Forward kinematics failed: %s", esp_err_to_name(ret));
    }

    // TODO: Calculate leg velocities from position differences
    // TODO: Estimate body pose from leg positions
    // TODO: Calculate body velocities

#if KPP_ENABLE_STATE_LOGGING
    static int log_counter = 0;
    if (++log_counter >= KPP_LOG_INTERVAL) {
        ESP_LOGD(TAG, "State: leg0 angles=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] accel=[%.1f,%.1f,%.1f]",
                 state->joint_angles[0][0], state->joint_angles[0][1], state->joint_angles[0][2],
                 state->joint_velocities[0][0], state->joint_velocities[0][1], state->joint_velocities[0][2],
                 state->joint_accelerations[0][0], state->joint_accelerations[0][1], state->joint_accelerations[0][2]);
        log_counter = 0;
    }
#endif
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