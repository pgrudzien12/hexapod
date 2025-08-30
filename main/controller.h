#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#define CONTROLLER_MAX_CHANNELS 14

typedef struct {
    int uart_port;     // e.g., UART_NUM_1
    int tx_gpio;       // TX pin
    int rx_gpio;       // RX pin
    int rts_gpio;      // or UART_PIN_NO_CHANGE
    int cts_gpio;      // or UART_PIN_NO_CHANGE
    int baud_rate;     // e.g., 115200
    int task_stack;    // e.g., 4096
    int task_prio;     // e.g., 10
} controller_config_t;

// Initialize UART task that continuously reads iBUS and updates channel cache
void controller_init(const controller_config_t *cfg);

// Copy latest channel values; returns true if fresh data was available
bool controller_get_channels(uint16_t out[CONTROLLER_MAX_CHANNELS]);

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

// Convert raw channel u16 into normalized controller_state
void controller_decode(const uint16_t ch[CONTROLLER_MAX_CHANNELS], controller_state_t *out);

#endif // CONTROLLER_H