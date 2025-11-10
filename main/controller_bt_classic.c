// Bluetooth Classic SPP controller driver: receives binary channel frames over Bluetooth SPP
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_mac.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "controller_internal.h"
#include "controller_bt_classic.h"
#include "rpc_commands.h"

static const char *TAG = "ctrl_bt";

// Frame constants (same as WiFi TCP protocol)
#define BT_CTRL_SYNC0 0xAA
#define BT_CTRL_SYNC1 0x55
#define BT_CTRL_PROTO_VERSION 1
#define BT_CTRL_BASE_HEADER 8
#define BT_CTRL_NUM_CHANNELS 32
#define BT_CTRL_CHANNEL_BYTES (BT_CTRL_NUM_CHANNELS * 2)
#define BT_CTRL_FRAME_SIZE (BT_CTRL_BASE_HEADER + BT_CTRL_CHANNEL_BYTES + 2)

// Global state
static uint32_t g_spp_handle = 0;
static bool g_connected = false;
static TickType_t g_last_frame = 0;
static controller_bt_classic_cfg_t g_config;
static char g_device_name[32]; // Generated device name buffer

// Generate unique device name based on MAC address
static void build_device_name(char *out, size_t out_sz, const char *prefix) {
    if (!out || out_sz == 0) return;
    
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        // Fallback to default if MAC read fails
        const char *pfx = prefix ? prefix : "HEXAPOD";
        snprintf(out, out_sz, "%s_DEFAULT", pfx);
        return;
    }
    
    const char *pfx = prefix ? prefix : "HEXAPOD";
    snprintf(out, out_sz, "%s_%02X%02X%02X", pfx, mac[3], mac[4], mac[5]);
}

// CRC16-CCITT (same as WiFi TCP driver)
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
        }
    }
    return crc;
}

static void process_frame_data(const uint8_t *data, size_t len) {
    if (len < BT_CTRL_FRAME_SIZE) {
        ESP_LOGW(TAG, "frame too short: %zu bytes", len);
        return;
    }

    // Parse header
    if (data[0] != BT_CTRL_SYNC0 || data[1] != BT_CTRL_SYNC1) {
        ESP_LOGW(TAG, "bad sync %02X %02X", data[0], data[1]);
        return;
    }
    
    uint8_t ver = data[2];
    if (ver != BT_CTRL_PROTO_VERSION) {
        ESP_LOGW(TAG, "unsupported version %u", (unsigned)ver);
        return;
    }
    
    uint8_t flags = data[3];
    uint16_t seq = (uint16_t)(data[4] | (data[5] << 8));
    uint16_t payload_len = (uint16_t)(data[6] | (data[7] << 8));
    
    if (payload_len != BT_CTRL_CHANNEL_BYTES) {
        ESP_LOGW(TAG, "unexpected payload_len=%u", (unsigned)payload_len);
        return;
    }

    // Verify CRC
    uint16_t rx_crc = (uint16_t)(data[BT_CTRL_BASE_HEADER + BT_CTRL_CHANNEL_BYTES] | 
                                 (data[BT_CTRL_BASE_HEADER + BT_CTRL_CHANNEL_BYTES + 1] << 8));
    uint16_t calc_crc = crc16_ccitt(data, BT_CTRL_BASE_HEADER + BT_CTRL_CHANNEL_BYTES);
    
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch rx=%04X calc=%04X", rx_crc, calc_crc);
        return;
    }

    // Extract channels
    int16_t channels[BT_CTRL_NUM_CHANNELS];
    const uint8_t *payload = data + BT_CTRL_BASE_HEADER;
    for (int i = 0; i < BT_CTRL_NUM_CHANNELS; ++i) {
        channels[i] = (int16_t)(payload[2*i] | (payload[2*i+1] << 8));
    }
    
    controller_internal_update_channels(channels);
    g_last_frame = xTaskGetTickCount();
    
    // Suppress unused variable warnings
    (void)flags;
    (void)seq;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "SPP initialized");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, g_config.spp_server_name);
        } else {
            ESP_LOGE(TAG, "SPP init failed: %d", param->init.status);
        }
        break;
        
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "SPP server started handle:%"PRIu32" scn:%d", 
                    param->start.handle, param->start.scn);
            esp_bt_gap_set_device_name(g_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(TAG, "SPP start failed: %d", param->start.status);
        }
        break;
        
    case ESP_SPP_SRV_OPEN_EVT: {
        ESP_LOGI(TAG, "SPP client connected handle:%"PRIu32" bda:[%s]", 
                param->srv_open.handle, 
                esp_bt_dev_get_address() ? "..." : "unknown");
        g_spp_handle = param->srv_open.handle;
        g_connected = true;
        controller_internal_set_connected(true);
        g_last_frame = xTaskGetTickCount();
        break;
    }
    
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP connection closed handle:%"PRIu32, param->close.handle);
        g_connected = false;
        g_spp_handle = 0;
        controller_internal_set_connected(false);
        controller_internal_set_failsafe();
        break;
        
    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.handle == g_spp_handle) {
            const uint8_t *d = param->data_ind.data;
            size_t l = param->data_ind.len;
            // Detect binary control frame vs ASCII RPC text
            if (l >= 2 && d[0] == BT_CTRL_SYNC0 && d[1] == BT_CTRL_SYNC1) {
                process_frame_data(d, l);
            } else {
                // Treat as RPC textual data
                rpc_feed_bytes(d, l);
            }
        }
        break;
        
    default:
        ESP_LOGD(TAG, "SPP event: %d", event);
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BT authentication success: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "BT authentication failed: %d", param->auth_cmpl.stat);
        }
        break;
        
    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "BT PIN requested, sending fixed PIN: %"PRIu32, g_config.pin_code);
        esp_bt_pin_code_t pin_code;
        snprintf((char*)pin_code, sizeof(pin_code), "%04"PRIu32, g_config.pin_code);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
        
    default:
        ESP_LOGD(TAG, "BT GAP event: %d", event);
        break;
    }
}

static void bt_watchdog_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(g_config.connection_timeout_ms));
        
        if (g_connected) {
            TickType_t now = xTaskGetTickCount();
            if ((now - g_last_frame) > pdMS_TO_TICKS(g_config.connection_timeout_ms)) {
                ESP_LOGW(TAG, "connection timeout, setting failsafe");
                controller_internal_set_failsafe();
            }
        }
    }
}

void controller_driver_init_bt_classic(const struct controller_config_s *core) {
    esp_err_t ret;
    
    // Get driver config or use defaults
    size_t cfg_sz = 0;
    const void *p = controller_internal_get_driver_cfg(&cfg_sz);
    g_config = controller_bt_classic_default();
    if (p && cfg_sz == sizeof(controller_bt_classic_cfg_t)) {
        g_config = *(const controller_bt_classic_cfg_t*)p;
    }
    
    // Generate unique device name
    build_device_name(g_device_name, sizeof(g_device_name), g_config.device_name_prefix);
    
    ESP_LOGI(TAG, "Initializing BT Classic controller (device: %s)", g_device_name);
    
    // Initialize NVS (required for BT)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Release BLE memory to save space
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize Bluedroid
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = g_config.enable_ssp;
    
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register callbacks
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    
    // Initialize SPP
    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));
    
    // Set up pairing parameters (legacy pairing with fixed PIN)
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    snprintf((char*)pin_code, sizeof(pin_code), "%04"PRIu32, g_config.pin_code);
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
    
    // Start watchdog task
    xTaskCreate(bt_watchdog_task, "bt_watchdog", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "BT Classic controller ready (PIN: %04"PRIu32")", g_config.pin_code);
}

const char *controller_bt_get_device_name(void) {
    return g_device_name[0] ? g_device_name : NULL;
}

esp_err_t controller_bt_classic_send_raw(const char *data, size_t len) {
    if (!g_connected || g_spp_handle == 0 || !data || len == 0) {
        return ESP_FAIL;
    }
    esp_err_t ret = esp_spp_write(g_spp_handle, len, (uint8_t*)data);
    return ret == ESP_OK ? ESP_OK : ret;
}
