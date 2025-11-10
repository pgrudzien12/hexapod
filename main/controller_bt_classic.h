#ifndef CONTROLLER_BT_CLASSIC_H
#define CONTROLLER_BT_CLASSIC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for controller_config_t
struct controller_config_s;

// Bluetooth Classic SPP controller driver configuration
typedef struct {
    const char *device_name_prefix;  // Device name prefix (default: "HEXAPOD")
    const char *spp_server_name;     // SPP server name (default: "HEXAPOD_SPP")
    bool enable_ssp;                 // Enable Secure Simple Pairing (default: false for no-input devices)
    uint32_t pin_code;               // Fixed PIN code for legacy pairing (default: 1234)
    uint16_t connection_timeout_ms;  // Connection timeout (default: 1000ms)
} controller_bt_classic_cfg_t;

// Default configuration initializer
static inline controller_bt_classic_cfg_t controller_bt_classic_default(void) {
    controller_bt_classic_cfg_t cfg = {
        .device_name_prefix = "HEXAPOD",
        .spp_server_name = "HEXAPOD_SPP", 
        .enable_ssp = true, 
        .pin_code = 1234,
        .connection_timeout_ms = 1000
    };
    return cfg;
}

// Driver initialization function (called by controller core)
void controller_driver_init_bt_classic(const struct controller_config_s *core);

// Get the active Bluetooth device name (NULL if not started yet)
const char *controller_bt_get_device_name(void);

// Send raw textual data over active SPP connection (for RPC responses)
// Returns ESP_OK on success, ESP_FAIL if not connected.
esp_err_t controller_bt_classic_send_raw(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_BT_CLASSIC_H