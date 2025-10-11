#ifndef CONTROLLER_FLYSKY_IBUS_H
#define CONTROLLER_FLYSKY_IBUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration for FlySky iBUS driver
// Keep small & POD so it can be copied or placed in NVS-friendly storage.
typedef struct {
    int uart_port;   // UART_NUM_x value
    int tx_gpio;     // TX pin (or -1 for no change)
    int rx_gpio;     // RX pin
    int rts_gpio;    // or -1
    int cts_gpio;    // or -1
    int baud_rate;   // e.g., 115200
} controller_flysky_ibus_cfg_t;

// Provide a convenience inline default initializer
static inline controller_flysky_ibus_cfg_t controller_flysky_ibus_default(void) {
    controller_flysky_ibus_cfg_t c = {
        .uart_port = 1,   // UART_NUM_1
        .tx_gpio = -1,
        .rx_gpio = 22,
        .rts_gpio = -1,
        .cts_gpio = -1,
        .baud_rate = 115200,
    };
    return c;
}

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_FLYSKY_IBUS_H
