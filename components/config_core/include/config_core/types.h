#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Core-local parameter info type to avoid conflicts with legacy typedefs.
// Namespaces can translate into this lightweight metadata when needed.
typedef struct {
    const char *name;
    uint16_t type; // mirror of CONFIG_TYPE_* values
    uint16_t size;
} config_param_info_core_t;

// Generic value carrier for dynamic get/set APIs.
typedef union {
    bool     b;
    int32_t  i32;
    uint32_t u32;
    uint16_t u16;
    float    f;
    // For strings, caller will supply external buffer; union variant kept small intentionally.
} config_value_union_t;

// Namespace descriptor forward declaration for manager APIs.
struct config_namespace_descriptor_t; // defined in namespace.h

