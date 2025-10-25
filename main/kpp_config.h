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

// Velocity limits (rad/s) - near hardware limit for responsiveness
#define KPP_MAX_VELOCITY_COXA      6.0f    // Hip yaw joint
#define KPP_MAX_VELOCITY_FEMUR     6.0f    // Hip pitch joint  
#define KPP_MAX_VELOCITY_TIBIA     6.0f    // Knee joint

// Acceleration limits (rad/s²) - smooth acceleration for servo health
#define KPP_MAX_ACCELERATION_COXA  50.0f
#define KPP_MAX_ACCELERATION_FEMUR 50.0f
#define KPP_MAX_ACCELERATION_TIBIA 50.0f

// Jerk limits (rad/s³) - S-curve smoothing for natural motion
#define KPP_MAX_JERK_COXA          2000.0f
#define KPP_MAX_JERK_FEMUR         2000.0f
#define KPP_MAX_JERK_TIBIA         2000.0f

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
#define KPP_MAX_LEG_VELOCITY           2.0f   // m/s - maximum reasonable leg tip velocity  
#define KPP_MAX_BODY_VELOCITY          1.0f   // m/s - maximum reasonable body velocity
#define KPP_MAX_ANGULAR_VELOCITY       5.0f   // rad/s - maximum reasonable angular velocity

// Body pose estimation parameters
#define KPP_BODY_PITCH_FILTER_ALPHA    0.1f   // Heavy filtering for orientation estimates
#define KPP_BODY_ROLL_FILTER_ALPHA     0.1f   // Heavy filtering for orientation estimates

// Geometric parameters for body pose estimation (meters)
#define KPP_FRONT_TO_BACK_DISTANCE     0.15f  // Distance from front legs to back legs
#define KPP_LEFT_TO_RIGHT_DISTANCE     0.12f  // Distance from left legs to right legs

// Velocity validation bounds
#define KPP_MAX_LEG_VELOCITY           2.0f   // m/s - maximum reasonable leg tip velocity  
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

// =============================================================================
// Future Configuration TODOs
// =============================================================================

// TODO: Add fast mode limits for quick reactions
// TODO: Add emergency mode limits for fall recovery  
// TODO: Add NVS storage keys for runtime parameter tuning
// TODO: Add coordinate frame transformation parameters
// TODO: Add sensor fusion parameters (when IMU/force sensors added)

#ifdef __cplusplus
}
#endif