#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "types/joint_types.h"  // existing joint_calib_t type

// Robot physical structure constants
#define NUM_LEGS 6
#define NUM_JOINTS_PER_LEG 3

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_CAL_SCHEMA_VERSION 1

// Complete joint calibration configuration (moved from monolith)
typedef struct {
    joint_calib_t joints[NUM_LEGS][NUM_JOINTS_PER_LEG];
    uint16_t config_version; // per-namespace versioning (future migrations)
} joint_calib_config_t;

#ifdef __cplusplus
}
#endif
