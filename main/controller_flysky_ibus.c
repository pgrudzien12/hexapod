// FlySky iBUS driver implementation using UART; pushes channel updates
// into controller abstraction via controller_internal helpers.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "controller_internal.h"
#include "controller_flysky_ibus.h"

#define IBUS_BUF_SIZE 64
static const char *TAG = "ctrl_flysky_ibus";

static void flysky_task(void *arg)
{
    const controller_config_t *cfg_base = controller_internal_get_config();
    // Attempt to fetch driver specific config, else fall back to internal defaults
    controller_flysky_ibus_cfg_t local_cfg;
    const controller_flysky_ibus_cfg_t *cfg_drv = NULL;
    size_t sz = 0;
    const void *p = controller_internal_get_driver_cfg(&sz);
    if (p && sz == sizeof(controller_flysky_ibus_cfg_t)) {
        cfg_drv = (const controller_flysky_ibus_cfg_t *)p;
    } else {
        local_cfg = controller_flysky_ibus_default();
        cfg_drv = &local_cfg;
    }
    // Configure UART driver
    uart_config_t uart_config = {
        .baud_rate = cfg_drv->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(cfg_drv->uart_port, 1024, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(cfg_drv->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(cfg_drv->uart_port, cfg_drv->tx_gpio, cfg_drv->rx_gpio, cfg_drv->rts_gpio, cfg_drv->cts_gpio));

    uint8_t data[IBUS_BUF_SIZE];
    TickType_t last_frame_tick = xTaskGetTickCount();
    controller_internal_set_connected(false);

    while (1) {
        const TickType_t timeout_ticks = pdMS_TO_TICKS(20);
    int len = uart_read_bytes(cfg_drv->uart_port, data, sizeof(data), timeout_ticks);
        TickType_t now_tick = xTaskGetTickCount();
        if (len >= 32) {
            if (data[0] == 0x20 && data[1] == 0x40) {
                uint16_t local[CONTROLLER_MAX_CHANNELS];
                for (int i = 0; i < CONTROLLER_MAX_CHANNELS; ++i) {
                    local[i] = (uint16_t)(data[2 + i*2] | (data[3 + i*2] << 8));
                }
                controller_internal_update_channels(local);
                last_frame_tick = now_tick;
                if (!controller_internal_is_connected()) {
                    controller_internal_set_connected(true);
                    ESP_LOGI(TAG, "iBUS connected (UART%d RX=%d TX=%d)", (int)cfg_drv->uart_port, (int)cfg_drv->rx_gpio, (int)cfg_drv->tx_gpio);
                }
            }
        }
        // Timeout handling: if no frames >1s enter failsafe
        TickType_t dt_ticks = now_tick - last_frame_tick;
        if (controller_internal_is_connected() && dt_ticks > pdMS_TO_TICKS(1000)) {
            controller_internal_set_connected(false);
            ESP_LOGW(TAG, "iBUS disconnected: no frames for %u ms, entering failsafe", (unsigned)(dt_ticks * portTICK_PERIOD_MS));
            controller_internal_set_failsafe();
        }
    }
}

void controller_driver_init_flysky_ibus(const controller_config_t *cfg)
{
    (void)cfg; // already stored globally
    xTaskCreate(flysky_task, "flysky_ibus", cfg->task_stack, NULL, cfg->task_prio, NULL);
}
