/*
 * KPP System Configuration
 * 
 * Compile-time constants for motion limits and system parameters.
 * TODO: Later add NVS storage for runtime tuning.
 * 
 * License: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Motion Limits - Normal Mode
// Focus on acceleration and jerk limits, allow higher velocity
// =============================================================================

// Velocity limits (rad/s) - based on observed maximum ~5.4 rad/s
#define KPP_MAX_VELOCITY_COXA      5.0f    // Adjusted to 5.0f per user request
#define KPP_MAX_VELOCITY_FEMUR     5.0f    // Adjusted to 5.0f per user request  
#define KPP_MAX_VELOCITY_TIBIA     5.0f    // Adjusted to 5.0f per user request

// Acceleration limits (rad/s²) - based on observed maximum ~477 rad/s²
#define KPP_MAX_ACCELERATION_COXA  600.0f  // Increased from 200.0f based on logs
#define KPP_MAX_ACCELERATION_FEMUR 600.0f  // Increased from 200.0f based on logs
#define KPP_MAX_ACCELERATION_TIBIA 600.0f  // Increased from 200.0f based on logs

// Jerk limits (rad/s³) - based on observed maximum ~2384 rad/s³
#define KPP_MAX_JERK_COXA          3500.0f  // Reduced from 10000.0f based on logs
#define KPP_MAX_JERK_FEMUR         3500.0f  // Reduced from 10000.0f based on logs
#define KPP_MAX_JERK_TIBIA         3500.0f  // Reduced from 10000.0f based on logs

// =============================================================================
// State Estimation Parameters
// =============================================================================

// Velocity estimation filter (exponential smoothing)
#define KPP_VELOCITY_FILTER_ALPHA  0.3f    // 0.0 = no filtering, 1.0 = no update

// Acceleration estimation filter
#define KPP_ACCEL_FILTER_ALPHA     0.2f    // More filtering for acceleration

// Leg velocity estimation filter (for position-derived velocities)
#define KPP_LEG_VELOCITY_FILTER_ALPHA  0.25f  // Slightly more filtering for leg velocities

// Body velocity estimation filter
#define KPP_BODY_VELOCITY_FILTER_ALPHA 0.2f   // More filtering for body velocity estimates

// Body pose estimation parameters
#define KPP_BODY_PITCH_FILTER_ALPHA    0.1f   // Heavy filtering for orientation estimates
#define KPP_BODY_ROLL_FILTER_ALPHA     0.1f   // Heavy filtering for orientation estimates

// Geometric parameters for body pose estimation (meters)
#define KPP_FRONT_TO_BACK_DISTANCE     0.15f  // Distance from front legs to back legs
#define KPP_LEFT_TO_RIGHT_DISTANCE     0.12f  // Distance from left legs to right legs

// Velocity validation bounds
#define KPP_MAX_LEG_VELOCITY           3.0f   // m/s - maximum reasonable leg tip velocity (increased for dynamic motion)
#define KPP_MAX_BODY_VELOCITY          1.0f   // m/s - maximum reasonable body velocity
#define KPP_MAX_ANGULAR_VELOCITY       5.0f   // rad/s - maximum reasonable angular velocity

// Minimum time step for numerical differentiation (seconds)
#define KPP_MIN_DT                 0.001f  // 1ms minimum

// Maximum time step for safety (seconds)
#define KPP_MAX_DT                 0.050f  // 50ms maximum

// =============================================================================
// Forward Kinematics Parameters
// =============================================================================

// Body frame origin offset from leg coordinate systems
#define KPP_BODY_OFFSET_X          0.0f    // Body center offset (m)
#define KPP_BODY_OFFSET_Y          0.0f
#define KPP_BODY_OFFSET_Z          0.0f

// =============================================================================
// Debugging and Logging
// =============================================================================

// Enable detailed state logging (ESP_LOGD)
#define KPP_ENABLE_STATE_LOGGING   1

// Log interval for periodic state dumps (control cycles)
#define KPP_LOG_INTERVAL           100     // Log every 100 cycles (1 second at 10ms)

// Enable motion limiting debug info
#define KPP_ENABLE_LIMIT_LOGGING   1

// Enable debug monitoring of acceleration/jerk limits (development only)
#ifndef NDEBUG
#define KPP_ENABLE_DEBUG_MONITORING 1
#define KPP_DEBUG_LOG_INTERVAL     100     // Debug logs every 100 cycles
#else
#define KPP_ENABLE_DEBUG_MONITORING 0
#endif

// =============================================================================
// Future Configuration TODOs
// =============================================================================

// TODO: Add fast mode limits for quick reactions
// TODO: Add emergency mode limits for fall recovery  
// TODO: Add NVS storage keys for runtime parameter tuning
// TODO: Add coordinate frame transformation parameters
// TODO: Add sensor fusion parameters (when IMU/force sensors added)

// =============================================================================
// Production Notes
// =============================================================================

/*
 * KPP System Optimization Summary:
 * 
 * The KPP system has been optimized through extensive real-world testing:
 * 
 * CRITICAL ARCHITECTURAL FIX:
 * - State estimation uses ORIGINAL gait commands, not limited commands
 * - This prevents feedback loop oscillations that were causing instability
 * - Motion limiting still protects servos without interfering with gait timing
 * 
 * OPTIMAL LIMITS (based on data collection):
 * - Velocity: 5.0 rad/s (max observed ~5.4 rad/s in extreme maneuvers)
 * - Acceleration: 600 rad/s² (max observed ~477 rad/s² in normal operation)
 * - Jerk: 3500 rad/s³ (max observed ~2384 rad/s³ in rapid direction changes)
 * 
 * PERFORMANCE CHARACTERISTICS:
 * - Normal operation: Zero interference (diff=0.000)
 * - Extreme maneuvers: Minimal limiting when needed (diff<0.01 typical)
 * - Debug builds: Include monitoring and logging
 * - Production builds: Zero debug overhead
 */

#ifdef __cplusplus
}
#endif