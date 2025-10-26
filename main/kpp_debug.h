/*
 * KPP Debug Utilities
 * 
 * Debug-only functions for monitoring motion limiting performance
 * and tuning acceleration/jerk limits. Disabled in production builds.
 * 
 * License: Apache-2.0
 */

#pragma once

#include "kpp_system.h"
#include "whole_body_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Monitor and log motion limiting statistics
 * 
 * Tracks acceleration and jerk extremes across all joints to help
 * tune motion limits. Only active in debug builds.
 * 
 * @param state Current kinematic state
 * @param original_cmd Original gait commands
 * @param limited_cmd Motion-limited commands
 * @param dt Time step
 */
void kpp_debug_monitor_limits(const kinematic_state_t* state,
                              const whole_body_cmd_t* original_cmd,
                              const whole_body_cmd_t* limited_cmd,
                              float dt);

#ifdef __cplusplus
}
#endif