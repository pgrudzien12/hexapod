#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "user_command.h" // for user_command_t comparison helper

#define CONTROLLER_MAX_CHANNELS 32

// Supported controller input drivers. Implemented: FLYSKY_IBUS, WIFI_TCP.
typedef enum {
    CONTROLLER_DRIVER_FLYSKY_IBUS = 0, // current default (UART iBUS)
    CONTROLLER_DRIVER_UART_GENERIC,    // placeholder for simple raw UART protocol
    CONTROLLER_DRIVER_BT_CLASSIC,      // placeholder
    CONTROLLER_DRIVER_BT_LE,           // placeholder
    CONTROLLER_DRIVER_WIFI_TCP,        // placeholder (planned next)
} controller_driver_type_e;

typedef struct {
    controller_driver_type_e driver_type; // selected input driver
    int task_stack;    // common task stack size (driver may override internally)
    int task_prio;     // common task priority
    // Opaque pointer to driver-specific configuration struct (lifetime must outlive controller unless deep copied in future extension)
    const void *driver_cfg;   // e.g. pointer to controller_flysky_ibus_cfg_t
    size_t driver_cfg_size;   // size of structure pointed to by driver_cfg (for validation / optional deep copy)
} controller_config_t;

// Initialize selected controller driver task that updates internal channel cache.
// Passing NULL uses default configuration (FLYSKY_IBUS driver with built-in UART pins).
void controller_init(const controller_config_t *cfg);

// Copy latest raw channel values (signed -32768..32767). Returns true on success.
bool controller_get_channels(int16_t out[CONTROLLER_MAX_CHANNELS]);

// Helpers to map channels to logical controls
typedef enum {
    GAIT_MODE_WAVE = 0,
    GAIT_MODE_RIPPLE = 1,
    GAIT_MODE_TRIPOD = 2,
} gait_mode_e;

typedef struct {
    // normalized sticks: -1..+1
    float right_horiz; // CH1
    float left_vert;   // CH2 (forward speed)
    float right_vert;  // CH3
    float left_horiz;  // CH4
    // switches
    bool swa_arm;      // CH5
    bool swb_pose;     // CH7 (true=pose mode)
    gait_mode_e swc_gait; // CH8
    bool swd_terrain;  // CH9 (true=climb)
    float sra_knob;    // CH10 0..1
} controller_state_t;

// Convert raw signed channels into normalized controller_state
// Stick channels assumed approximately linear in -32768..32767.
void controller_decode(const int16_t ch[CONTROLLER_MAX_CHANNELS], controller_state_t *out);

// Default epsilon for float field comparison in user_command comparison helper
#ifndef CONTROLLER_CMD_FLOAT_EPSILON
#define CONTROLLER_CMD_FLOAT_EPSILON 1e-2f
#endif

// Compare two user_command_t structures field-by-field.
// Floating point members are considered equal if their absolute difference
// is <= (tol > 0 ? tol : CONTROLLER_CMD_FLOAT_EPSILON).
// Returns true if both commands are equivalent under these rules.
// If both pointers are NULL returns true; if only one is NULL returns false.
bool controller_user_command_equal(const user_command_t *a, const user_command_t *b, float tol);

#endif // CONTROLLER_H