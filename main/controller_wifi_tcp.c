// WiFi TCP RPC server: listens on a TCP port for line-terminated RPC commands.
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"

#include "controller_wifi_tcp.h"
#include "wifi_ap.h"
#include "rpc_transport.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"

static const char *TAG = "wifi_rpc";

// Global state for RPC over WiFi TCP
static int g_active_client_sock = -1;

static void wifi_tcp_client_task(void *arg) {
    int client_sock = (int)(intptr_t)arg;
    g_active_client_sock = client_sock;
    ESP_LOGI(TAG, "client connected fd=%d", client_sock);

    TickType_t last_activity = xTaskGetTickCount();
    char rpc_buffer[512];
    int rpc_buffer_pos = 0;

    while (1) {
        uint8_t byte;
        int ret = recv(client_sock, &byte, 1, 0); // Blocking read
        if (ret <= 0) {
            // Connection closed or error
            ESP_LOGI(TAG, "Client disconnected or read error");
            break;
        }
        
        // Check for activity timeout (60 seconds)
        if ((xTaskGetTickCount() - last_activity) > pdMS_TO_TICKS(60000)) {
            ESP_LOGW(TAG, "Client timeout, closing connection");
            break;
        }
        
        // Accumulate bytes until line terminator
        if (rpc_buffer_pos < (sizeof(rpc_buffer) - 1)) {
            rpc_buffer[rpc_buffer_pos++] = byte;
        } else {
            // Buffer overflow, reset and continue
            ESP_LOGW(TAG, "RPC buffer overflow, resetting");
            rpc_buffer_pos = 0;
            continue;
        }
        
        // Check for line terminator
        if (byte == '\n' || byte == '\r') {
            // Complete RPC line received
            rpc_buffer[rpc_buffer_pos] = '\0';
            
            // Remove trailing \r\n characters
            while (rpc_buffer_pos > 0 && 
                   (rpc_buffer[rpc_buffer_pos-1] == '\n' || rpc_buffer[rpc_buffer_pos-1] == '\r')) {
                rpc_buffer[--rpc_buffer_pos] = '\0';
            }
            
            // Process non-empty commands
            if (rpc_buffer_pos > 0) {
                ESP_LOGI(TAG, "RPC command: %s", rpc_buffer);
                rpc_transport_rx_send(RPC_TRANSPORT_WIFI_TCP, (uint8_t*)rpc_buffer, rpc_buffer_pos);
                last_activity = xTaskGetTickCount();
            }
            
            // Reset buffer for next command
            rpc_buffer_pos = 0;
        }
    }

    close(client_sock);
    if (g_active_client_sock == client_sock) {
        g_active_client_sock = -1;
    }
    ESP_LOGI(TAG, "client closed");
    vTaskDelete(NULL);
}

static void wifi_tcp_server_task(void *arg) {
    // Ensure AP is up (simple AP-only mode for early development)
    wifi_ap_init_once();
    const char *ssid = wifi_ap_get_ssid();
    if (ssid) {
        ESP_LOGI(TAG, "WiFi AP active SSID=%s", ssid);
    }
    
    // Wait for WiFi AP to be fully ready and TCP/IP stack initialized
    ESP_LOGI(TAG, "Waiting for network stack to be ready...");
    
    // Wait for network interface to be available and have an IP
    esp_netif_t* netif = NULL;
    esp_netif_ip_info_t ip_info;
    int retry_count = 0;
    const int max_retries = 50; // 5 seconds total
    
    while (retry_count < max_retries) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Try to get the WiFi AP network interface
        netif = esp_netif_get_default_netif();

        if (netif != NULL) {
            // Check if it has an IP address configured
            esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
            if (ret == ESP_OK && ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "WiFi AP network interface ready with IP: " IPSTR, 
                        IP2STR(&ip_info.ip));
                break;
            }
        }
        retry_count++;
        
        if (retry_count % 10 == 0) {
            ESP_LOGI(TAG, "Still waiting for WiFi AP network interface... (%d/%d)", retry_count, max_retries);
        }
    }
    
    if (netif == NULL || ip_info.ip.addr == 0) {
        ESP_LOGE(TAG, "Failed to get WiFi AP network interface or IP after %d retries", retry_count);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Network interface ready, waiting additional time for TCP/IP stack...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Additional wait for TCP/IP stack
    
    // Default configuration for RPC server
    const uint16_t listen_port = 8080;
    const int max_clients = 4;
    const int connect_backoff_ms = 1000;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket create failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listen_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, max_clients) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "WiFi RPC server listening on " IPSTR ":%u", 
             IP2STR(&ip_info.ip), (unsigned)listen_port);
    
    // Register RPC transport sender
    rpc_transport_register_sender(RPC_TRANSPORT_WIFI_TCP, controller_wifi_tcp_send_raw);

    while (1) {
        struct sockaddr_in caddr; socklen_t clen = sizeof(caddr);
        int client = accept(listen_fd, (struct sockaddr*)&caddr, &clen);
        if (client < 0) {
            ESP_LOGW(TAG, "accept error errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(connect_backoff_ms));
            continue;
        }
        // Spawn client handler task
        xTaskCreate(wifi_tcp_client_task, "ctrl_wifi_cli", 4096, (void*)(intptr_t)client, 9, NULL);
    }
}

void wifi_tcp_rpc_init(uint32_t task_stack, int task_prio) {
    xTaskCreate(wifi_tcp_server_task, "wifi_rpc_srv", task_stack, NULL, task_prio, NULL);
}

esp_err_t controller_wifi_tcp_send_raw(const char *data, size_t len) {
    if (g_active_client_sock < 0 || !data || len == 0) {
        return ESP_FAIL;
    }
    
    ssize_t sent = send(g_active_client_sock, data, len, 0);
    if (sent < 0 || (size_t)sent != len) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
