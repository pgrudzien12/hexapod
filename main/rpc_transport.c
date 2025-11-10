/*
 * RPC Transport Abstraction Implementation
 */

#include "rpc_transport.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "rpc_transport";

// FreeRTOS queues for RPC communication
static QueueHandle_t g_rx_queue = NULL;  // Controllers -> RPC
static QueueHandle_t g_tx_queue = NULL;  // RPC -> Controllers

// Transport send function registry
static rpc_transport_send_fn_t g_transport_senders[RPC_TRANSPORT_COUNT] = {0};

esp_err_t rpc_transport_init(void) {
    if (g_rx_queue || g_tx_queue) {
        return ESP_OK; // Already initialized
    }
    
    g_rx_queue = xQueueCreate(10, sizeof(rpc_rx_message_t));
    if (!g_rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_ERR_NO_MEM;
    }
    
    g_tx_queue = xQueueCreate(10, sizeof(rpc_tx_message_t));
    if (!g_tx_queue) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        vQueueDelete(g_rx_queue);
        g_rx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Start transport task
    xTaskCreate(rpc_transport_task, "rpc_transport", 3072, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Transport layer initialized");
    return ESP_OK;
}

esp_err_t rpc_transport_rx_send(rpc_transport_type_t transport, const uint8_t* data, size_t len) {
    if (!g_rx_queue || !data || len == 0 || len > sizeof(((rpc_rx_message_t*)0)->data)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    rpc_rx_message_t msg = {
        .transport = transport,
        .len = len
    };
    memcpy(msg.data, data, len);
    
    BaseType_t result = xQueueSend(g_rx_queue, &msg, 0); // Non-blocking
    return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t rpc_transport_rx_receive(rpc_rx_message_t* msg, uint32_t timeout_ms) {
    if (!g_rx_queue || !msg) {
        return ESP_ERR_INVALID_ARG;
    }
    
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    BaseType_t result = xQueueReceive(g_rx_queue, msg, ticks);
    return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t rpc_transport_tx_send(rpc_transport_type_t transport, const char* data, size_t len) {
    if (!g_tx_queue || !data || len == 0 || len > sizeof(((rpc_tx_message_t*)0)->data)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    rpc_tx_message_t msg = {
        .transport = transport,
        .len = len
    };
    memcpy(msg.data, data, len);
    
    BaseType_t result = xQueueSend(g_tx_queue, &msg, pdMS_TO_TICKS(100)); // 100ms timeout
    return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t rpc_transport_register_sender(rpc_transport_type_t transport, rpc_transport_send_fn_t send_fn) {
    if (transport >= RPC_TRANSPORT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_transport_senders[transport] = send_fn;
    ESP_LOGI(TAG, "Registered sender for transport %d", transport);
    return ESP_OK;
}

void rpc_transport_task(void* param) {
    ESP_LOGI(TAG, "Transport task started");
    
    while (1) {
        rpc_tx_message_t msg;
        BaseType_t result = xQueueReceive(g_tx_queue, &msg, portMAX_DELAY);
        
        if (result == pdTRUE) {
            // Send via registered transport
            if (msg.transport < RPC_TRANSPORT_COUNT && g_transport_senders[msg.transport]) {
                esp_err_t err = g_transport_senders[msg.transport](msg.data, msg.len);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Transport %d send failed: %s", msg.transport, esp_err_to_name(err));
                }
            } else {
                ESP_LOGW(TAG, "No sender registered for transport %d", msg.transport);
            }
        }
    }
}