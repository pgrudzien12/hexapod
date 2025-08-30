// FlySky iBUS receiver reader as a reusable module
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "controller.h"

// Runtime configuration (override via controller_init)
static controller_config_t g_cfg = {
    .uart_port = UART_NUM_1,
    .tx_gpio = GPIO_NUM_17,
    .rx_gpio = GPIO_NUM_16,
    .rts_gpio = UART_PIN_NO_CHANGE,
    .cts_gpio = UART_PIN_NO_CHANGE,
    .baud_rate = 115200,
    .task_stack = 4096,
    .task_prio = 10,
};

// no logging in this module to keep it lightweight

#define BUF_SIZE (64)

// | Control                 | Function                  | Notes                         |
// | ----------------------- | ------------------------- | ----------------------------- |
// | Right Vert (non-center) | Body height $z\_target$   | Slew-limited; capture on arm  |
// | Right Horiz             | Lateral shift $y\_target$ | Small deadband                |
// | Left Vert               | Forward speed $v\_x$      | Scaled by SRA × SWC           |
// | Left Horiz              | Turn rate $\omega\_z$     | Expo recommended              |
// | SRA (knob)              | Step frequency scale      | 0–100% inside SWC caps        |
// | SWA (2)                 | Arm/Disarm                | Safety, soft-start            |
// | SWB (2)                 | Walk ↔ Pose mode          | Pose: left stick → roll/pitch |
// | SWC (3)                 | Gait                      | Wave / Ripple / Tripod        |
// | SWD (2)                 | Terrain profile           | Climb / Normal               |


static SemaphoreHandle_t g_ch_mutex;
static uint16_t g_channels[CONTROLLER_MAX_CHANNELS];
static volatile uint32_t g_last_update_ms;

static void controller_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = g_cfg.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(g_cfg.uart_port, 1024, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(g_cfg.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_cfg.uart_port, g_cfg.tx_gpio, g_cfg.rx_gpio, g_cfg.rts_gpio, g_cfg.cts_gpio));

    if (!g_ch_mutex) {
        g_ch_mutex = xSemaphoreCreateMutex();
    }
    uint8_t data[BUF_SIZE];

    while (1) { 
        // Use a small fixed timeout in ticks to avoid macro issues in static analysis
        const TickType_t timeout_ticks = 20; // ~20ms if tick=1ms
        int len = uart_read_bytes(g_cfg.uart_port, data, sizeof(data), timeout_ticks);
        if (len < 0) {
            // Failsafe defaults when transmitter not connected
            uint16_t local[CONTROLLER_MAX_CHANNELS];
            for (int i = 0; i < CONTROLLER_MAX_CHANNELS; ++i) {
                local[i] = 1000; // switches and unused default low
            }
            // Sticks neutral 1500 except CH3 Right Vert = 1000
            local[0] = 1500; // CH1 Right Horiz
            local[1] = 1500; // CH2 Left Vert
            // CH3 stays 1000
            local[3] = 1500; // CH4 Left Horiz
            local[5] = 1500; // CH6 Right Vert (unused) neutral
            if (xSemaphoreTake(g_ch_mutex, 5) == pdTRUE) {
                memcpy(g_channels, local, sizeof(local));
                g_last_update_ms = (uint32_t)(xTaskGetTickCount());
                xSemaphoreGive(g_ch_mutex);
            }
        } else if (len >= 32) {
            // look for IBUS frame
            if (data[0] == 0x20 && data[1] == 0x40) {
                uint16_t local[CONTROLLER_MAX_CHANNELS];
                for (int i = 0; i < CONTROLLER_MAX_CHANNELS; i++) {
                    local[i] = (uint16_t)(data[2 + i*2] | (data[3 + i*2] << 8));
                }
                if (xSemaphoreTake(g_ch_mutex, 5) == pdTRUE) {
                    memcpy(g_channels, local, sizeof(local));
                    g_last_update_ms = (uint32_t)(xTaskGetTickCount());
                    xSemaphoreGive(g_ch_mutex);


                }
                // ESP_LOGD(TAG, "CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u CH6=%u CH7=%u CH8=%u CH9=%u CH10=%u CH11=%u CH12=%u CH13=%u CH14=%u",
                //          local[0], local[1], local[2], local[3], local[4], local[5], local[6], local[7], local[8], local[9], local[10], local[11], local[12], local[13]);
            }
        }
    }
    // CH1 - RIGHT stick, sideways - 
    // CH2 - LEFT stick, updown - Z
    // CH3 - RIGHT stick, updown - Z
    // CH4 - LEFT stick, sideways - X
    // CH5 - SWA
    // CH6 - RIGHT stick, updown - dont use
    // CH7 - SWB
    // CH8 - SWC
    // CH9 - SWD
    // CH10 - VRA
}

void controller_init(const controller_config_t *cfg)
{
    if (cfg) {
        g_cfg = *cfg;
    }
    xTaskCreate(controller_task, "controller_task", g_cfg.task_stack, NULL, g_cfg.task_prio, NULL);
}

bool controller_get_channels(uint16_t out[CONTROLLER_MAX_CHANNELS])
{
    if (!out) {
        return false;
    }
    if (!g_ch_mutex) {
        return false;
    }
    if (xSemaphoreTake(g_ch_mutex, 5) != pdTRUE) {
        return false;
    }
    memcpy(out, g_channels, sizeof(uint16_t) * CONTROLLER_MAX_CHANNELS);
    xSemaphoreGive(g_ch_mutex);
    return true;
}

static float map_ibus_norm(uint16_t v)
{
    // clamp helper to avoid any indentation warnings
    uint16_t vv;
    if (v < 1000) {
        vv = 1000;
    } else if (v > 2000) {
        vv = 2000;
    } else {
        vv = v;
    }
    float f = ((float)vv - 1500.0f) / 500.0f; // -1..+1
    return f;
}

void controller_decode(const uint16_t ch[CONTROLLER_MAX_CHANNELS], controller_state_t *out)
{
    if (!ch || !out) {
        return;
    }
    out->right_horiz = map_ibus_norm(ch[0]); // CH1: Right Horiz -> lateral shift
    out->left_vert   = map_ibus_norm(ch[1]); // CH2: Left Vert -> forward speed
    out->right_vert  = map_ibus_norm(ch[2]); // CH3: Right Vert -> z_target
    out->left_horiz  = map_ibus_norm(ch[3]); // CH4: Left Horiz -> yaw rate
    out->swa_arm  = ch[4] > 1500;            // CH5: SWA
    out->swb_pose = ch[6] > 1500;            // CH7: SWB
    // CH8 three-position to gait
    if (ch[7] < 1300) {
        out->swc_gait = GAIT_MODE_WAVE;
    } else if (ch[7] < 1700) {
        out->swc_gait = GAIT_MODE_RIPPLE;
    } else {
        out->swc_gait = GAIT_MODE_TRIPOD;
    }
    out->swd_terrain = ch[8] > 1500;         // CH9: SWD
    // CH10: VRA 0..1
    uint16_t vra = ch[9];
    vra = (vra < 1000) ? 1000 : vra;
    vra = (vra > 2000) ? 2000 : vra;
    out->sra_knob = ((float)vra - 1000.0f) / 1000.0f;
}
