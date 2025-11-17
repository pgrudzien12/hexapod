#ifndef CONTROLLER_WIFI_TCP_H
#define CONTROLLER_WIFI_TCP_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi TCP RPC Server
// Provides a simple TCP server that accepts line-terminated text commands
// and forwards them to the RPC system for processing.
//
// Protocol: Simple text lines terminated by \n or \r\n
// Example: "get system.robot_name\n"
//          "set motion.max_velocity 2.5\n"

// Initialize WiFi TCP RPC server
// task_stack: Stack size for server tasks (recommended: 4096-8192 bytes)
// task_prio: Priority for server tasks (recommended: 5-10)
void wifi_tcp_rpc_init(uint32_t task_stack, int task_prio);

// Send raw textual data over active TCP connection (for RPC responses)
// Returns ESP_OK on success, ESP_FAIL if no active client or send error
esp_err_t controller_wifi_tcp_send_raw(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_WIFI_TCP_H
