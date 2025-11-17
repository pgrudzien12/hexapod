#ifndef CONTROLLER_WIFI_TCP_H
#define CONTROLLER_WIFI_TCP_H

#include <stdint.h>
#include "esp_err.h"
#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration for WiFi TCP controller
typedef struct {
    uint16_t listen_port;
    uint16_t connection_timeout_ms;
} controller_wifi_tcp_cfg_t;

static inline controller_wifi_tcp_cfg_t controller_wifi_tcp_default(void) {
    return (controller_wifi_tcp_cfg_t){
        .listen_port = 5555,
        .connection_timeout_ms = 60000,
    };
}

// Initialize WiFi TCP controller driver
void controller_driver_init_wifi_tcp(const controller_config_t *core);

// Send raw textual data over active TCP connection (for RPC responses)
esp_err_t controller_wifi_tcp_send_raw(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_WIFI_TCP_H
