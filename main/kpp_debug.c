/*
 * KPP Debug Utilities Implementation
 * 
 * Debug-only monitoring and logging for motion limiting performance.
 * 
 * License: Apache-2.0
 */

#include "kpp_debug.h"
#include "kpp_config.h"
#include "robot_config.h"  // For NUM_LEGS
#include "esp_log.h"
#include <math.h>

#if KPP_ENABLE_DEBUG_MONITORING
static const char *TAG = "kpp_debug";
#endif

void kpp_debug_monitor_limits(const kinematic_state_t* state,
                              const whole_body_cmd_t* original_cmd,
                              const whole_body_cmd_t* limited_cmd,
                              float dt)
{
#if KPP_ENABLE_DEBUG_MONITORING
    if (!state || !original_cmd || !limited_cmd) return;

    // Static variables for tracking extremes and logging interval
    static int log_counter = 0;
    static float max_accel_seen = 0.0f;
    static float max_jerk_seen = 0.0f;
    static float min_accel_seen = 0.0f;
    static float min_jerk_seen = 0.0f;
    static float vel_prev[NUM_LEGS][3] = {0};

    // Track acceleration and jerk extremes across all joints
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            float accel = state->joint_accelerations[leg][joint];
            float vel_curr = state->joint_velocities[leg][joint];
            float jerk = (vel_curr - vel_prev[leg][joint]) / dt;
            vel_prev[leg][joint] = vel_curr;
            
            // Track extremes
            if (accel > max_accel_seen) max_accel_seen = accel;
            if (accel < min_accel_seen) min_accel_seen = accel;
            if (jerk > max_jerk_seen) max_jerk_seen = jerk;
            if (jerk < min_jerk_seen) min_jerk_seen = jerk;
        }
    }

    // Log periodically (every 1 second)
    if (++log_counter >= KPP_DEBUG_LOG_INTERVAL) {
        float gait_cmd = original_cmd->joint_cmds[0].joint_angles[0];
        float limited_cmd_val = limited_cmd->joint_cmds[0].joint_angles[0];
        float body_roll = state->body_orientation[0] * 180.0f / 3.14159f;
        float body_pitch = state->body_orientation[1] * 180.0f / 3.14159f;
        
        ESP_LOGD(TAG, "MOTION: gait=%.3f limited=%.3f diff=%.3f body_roll=%.1f° pitch=%.1f°", 
                 gait_cmd, limited_cmd_val, fabsf(gait_cmd - limited_cmd_val), body_roll, body_pitch);
                 
        // Log acceleration and jerk statistics for limit tuning
        ESP_LOGD(TAG, "LIMITS: max_accel=%.1f max_jerk=%.0f min_accel=%.1f min_jerk=%.0f", 
                 max_accel_seen, max_jerk_seen, min_accel_seen, min_jerk_seen);
        
        // Reset statistics for next period
        max_accel_seen = 0.0f;
        max_jerk_seen = 0.0f;
        min_accel_seen = 0.0f;
        min_jerk_seen = 0.0f;
        log_counter = 0;
    }
#else
    // Debug monitoring disabled - suppress unused parameter warnings
    (void)state; (void)original_cmd; (void)limited_cmd; (void)dt;
#endif
}