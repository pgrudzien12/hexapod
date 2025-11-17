/*
 * RPC Transport Abstraction Layer
 * 
 * Provides a queue-based interface between controller drivers and RPC system.
 * Controllers write incoming data to rx_queue, RPC writes responses to tx_queue.
 * Transport layer handles the actual sending/receiving over the physical medium.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Transport types
typedef enum {
    RPC_TRANSPORT_BLUETOOTH = 0,
    RPC_TRANSPORT_WIFI_TCP,
    RPC_TRANSPORT_SERIAL,
    RPC_TRANSPORT_INTERNAL, // For in-process commands
    RPC_TRANSPORT_COUNT
} rpc_transport_type_t;

// Message types for the queues
typedef struct {
    rpc_transport_type_t transport;
    uint8_t data[256];
    size_t len;
} rpc_rx_message_t;

typedef struct {
    rpc_transport_type_t transport;
    char data[256];
    size_t len;
} rpc_tx_message_t;

// Transport abstraction layer initialization
esp_err_t rpc_transport_init(void);

// Queue access for controllers (producers)
esp_err_t rpc_transport_rx_send(rpc_transport_type_t transport, const uint8_t* data, size_t len);

// Queue access for RPC system (consumer/producer)  
esp_err_t rpc_transport_rx_receive(rpc_rx_message_t* msg, uint32_t timeout_ms);
esp_err_t rpc_transport_tx_send(rpc_transport_type_t transport, const char* data, size_t len);

// Transport registration for actual sending (called by transport drivers)
typedef esp_err_t (*rpc_transport_send_fn_t)(const char* data, size_t len);
esp_err_t rpc_transport_register_sender(rpc_transport_type_t transport, rpc_transport_send_fn_t send_fn);

// Transport task (processes tx queue and calls registered senders)
void rpc_transport_task(void* param);

#ifdef __cplusplus
}
#endif