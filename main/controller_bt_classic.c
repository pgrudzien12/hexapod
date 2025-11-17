// Bluetooth Classic SPP controller driver: receives RPC commands over Bluetooth SPP
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
#include "rpc_transport.h"

static const char *TAG = "ctrl_bt";

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
            // Send all data to RPC transport queue
            rpc_transport_rx_send(RPC_TRANSPORT_BLUETOOTH, d, l);
            g_last_frame = xTaskGetTickCount();
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
    
    // Register RPC transport sender
    rpc_transport_register_sender(RPC_TRANSPORT_BLUETOOTH, controller_bt_classic_send_raw);
    
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
