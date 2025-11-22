/*
 * Joint Type Definitions
 * 
 * Shared types for joint calibration used across multiple modules.
 * This header provides a clean separation of concerns without circular dependencies.
 * 
 * License: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Per-joint actuator calibration data
typedef struct {
    float zero_offset_rad;  // mechanical zero offset added before inversion/clamp
    int8_t invert_sign;     // +1 or -1 to flip direction  
    float min_rad;          // lower mechanical/electronic limit (radians)
    float max_rad;          // upper mechanical/electronic limit (radians)
    int32_t pwm_min_us;     // PWM at min_rad (microseconds)
    int32_t pwm_max_us;     // PWM at max_rad (microseconds)
    int32_t neutral_us;     // PWM at neutral (optional; informative)
} joint_calib_t;

#ifdef __cplusplus
}
#endif