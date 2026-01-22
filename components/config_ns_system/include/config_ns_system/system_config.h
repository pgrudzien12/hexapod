#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "controller.h" // for controller_driver_type_e

#ifdef __cplusplus
extern "C" {
#endif

// System Configuration Structure (moved from legacy config_manager.h)
typedef struct {
    bool emergency_stop_enabled;
    uint32_t auto_disarm_timeout;
    float safety_voltage_min;
    float temperature_limit_max;
    uint32_t motion_timeout_ms;
    uint32_t startup_delay_ms;
    uint32_t max_control_frequency;
    char robot_id[32];
    char robot_name[64];
    uint16_t config_version;
    controller_driver_type_e controller_type;
} system_config_t;

// Factory default loader (declaration; implementation stays in legacy until migrated fully)
void system_config_load_defaults(system_config_t *cfg);

#ifdef __cplusplus
}
#endif
