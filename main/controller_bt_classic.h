#ifndef CONTROLLER_BT_CLASSIC_H
#define CONTROLLER_BT_CLASSIC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for controller_config_t
struct controller_config_s;

// Bluetooth Classic SPP controller driver configuration
typedef struct {
    const char *device_name;     // Bluetooth device name (default: "HEXAPOD_BT")
    const char *spp_server_name; // SPP server name (default: "HEXAPOD_SPP")
    bool enable_ssp;             // Enable Secure Simple Pairing (default: false for no-input devices)
    uint32_t pin_code;           // Fixed PIN code for legacy pairing (default: 1234)
    uint16_t connection_timeout_ms; // Connection timeout (default: 1000ms)
} controller_bt_classic_cfg_t;

// Default configuration initializer
static inline controller_bt_classic_cfg_t controller_bt_classic_default(void) {
    controller_bt_classic_cfg_t cfg = {
        .device_name = "HEXAPOD_BT",
        .spp_server_name = "HEXAPOD_SPP", 
        .enable_ssp = false,  // Disable SSP since device has no input buttons
        .pin_code = 1234,     // Fixed PIN for legacy pairing
        .connection_timeout_ms = 1000
    };
    return cfg;
}

// Driver initialization function (called by controller core)
void controller_driver_init_bt_classic(const struct controller_config_s *core);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_BT_CLASSIC_H