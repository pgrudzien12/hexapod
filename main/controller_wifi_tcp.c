// WiFi TCP controller driver: receives RPC commands over a TCP socket.
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

#include "controller_internal.h"
#include "controller_wifi_tcp.h"
#include "wifi_ap.h"
#include "rpc_transport.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"

static const char *TAG = "ctrl_wifi_tcp";

// Global state for RPC over WiFi TCP
static int g_active_client_sock = -1;
static TickType_t g_last_frame = 0;

static void wifi_tcp_client_task(void *arg) {
    int client_sock = (int)(intptr_t)arg;
    
    // Disconnect previous client if any
    if (g_active_client_sock >= 0) {
        ESP_LOGI(TAG, "Disconnecting previous client fd=%d", g_active_client_sock);
        close(g_active_client_sock);
    }
    g_active_client_sock = client_sock;
    ESP_LOGI(TAG, "Client connected fd=%d", client_sock);
    controller_internal_set_connected(true);
    g_last_frame = xTaskGetTickCount();

    char rpc_buffer[512];
    int rpc_buffer_pos = 0;

    while (1) {
        uint8_t byte;
        int ret = recv(client_sock, &byte, 1, 0);
        if (ret <= 0) {
            ESP_LOGI(TAG, "Client disconnected or read error");
            break;
        }
        
        g_last_frame = xTaskGetTickCount();
        
        if (rpc_buffer_pos < (sizeof(rpc_buffer) - 1)) {
            rpc_buffer[rpc_buffer_pos++] = byte;
        } else {
            ESP_LOGW(TAG, "RPC buffer overflow, resetting");
            rpc_buffer_pos = 0;
            continue;
        }
        
        if (byte == '\n' || byte == '\r') {
            rpc_buffer[rpc_buffer_pos] = '\0';
            
            while (rpc_buffer_pos > 0 && 
                   (rpc_buffer[rpc_buffer_pos-1] == '\n' || rpc_buffer[rpc_buffer_pos-1] == '\r')) {
                rpc_buffer[--rpc_buffer_pos] = '\0';
            }
            
            if (rpc_buffer_pos > 0) {
                rpc_transport_rx_send(RPC_TRANSPORT_WIFI_TCP, (uint8_t*)rpc_buffer, rpc_buffer_pos);
            }
            
            rpc_buffer_pos = 0;
        }
    }

    close(client_sock);
    if (g_active_client_sock == client_sock) {
        g_active_client_sock = -1;
        controller_internal_set_connected(false);
        controller_internal_set_failsafe();
    }
    ESP_LOGI(TAG, "Client closed");
    vTaskDelete(NULL);
}

static void wifi_watchdog_task(void *arg) {
    const controller_wifi_tcp_cfg_t* cfg = (const controller_wifi_tcp_cfg_t*)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(cfg->connection_timeout_ms));
        
        if (g_active_client_sock >= 0) {
            TickType_t now = xTaskGetTickCount();
            if ((now - g_last_frame) > pdMS_TO_TICKS(cfg->connection_timeout_ms)) {
                ESP_LOGW(TAG, "Connection timeout, closing socket");
                close(g_active_client_sock);
                g_active_client_sock = -1;
                controller_internal_set_connected(false);
                controller_internal_set_failsafe();
            }
        }
    }
}

static void wifi_tcp_server_task(void *arg) {
    const controller_wifi_tcp_cfg_t* cfg = (const controller_wifi_tcp_cfg_t*)arg;

    wifi_ap_init_once();
    const char *ssid = wifi_ap_get_ssid();
    if (ssid) {
        ESP_LOGI(TAG, "WiFi AP active SSID=%s", ssid);
    }
    
    esp_netif_t* netif = NULL;
    esp_netif_ip_info_t ip_info;
    int retry_count = 0;
    const int max_retries = 50;
    
    while (retry_count < max_retries) {
        vTaskDelay(pdMS_TO_TICKS(100));
        netif = esp_netif_get_default_netif();
        if (netif != NULL) {
            esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
            if (ret == ESP_OK && ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "WiFi AP network interface ready with IP: " IPSTR, IP2STR(&ip_info.ip));
                break;
            }
        }
        retry_count++;
    }
    
    if (netif == NULL || ip_info.ip.addr == 0) {
        ESP_LOGE(TAG, "Failed to get WiFi AP network interface or IP");
        vTaskDelete(NULL);
        return;
    }
    
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
    addr.sin_port = htons(cfg->listen_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "WiFi TCP controller listening on " IPSTR ":%u", IP2STR(&ip_info.ip), (unsigned)cfg->listen_port);
    
    rpc_transport_register_sender(RPC_TRANSPORT_WIFI_TCP, controller_wifi_tcp_send_raw);

    while (1) {
        struct sockaddr_in caddr; socklen_t clen = sizeof(caddr);
        int client = accept(listen_fd, (struct sockaddr*)&caddr, &clen);
        if (client < 0) {
            ESP_LOGW(TAG, "accept error errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        xTaskCreate(wifi_tcp_client_task, "ctrl_wifi_cli", 4096, (void*)(intptr_t)client, 9, NULL);
    }
}

void controller_driver_init_wifi_tcp(const controller_config_t *core) {
    size_t cfg_sz = 0;
    const void *p = controller_internal_get_driver_cfg(&cfg_sz);
    static controller_wifi_tcp_cfg_t local_cfg;
    if (p && cfg_sz == sizeof(controller_wifi_tcp_cfg_t)) {
        local_cfg = *(const controller_wifi_tcp_cfg_t*)p;
    } else {
        local_cfg = controller_wifi_tcp_default();
    }
    
    xTaskCreate(wifi_tcp_server_task, "ctrl_wifi_srv", core->task_stack, &local_cfg, core->task_prio, NULL);
    xTaskCreate(wifi_watchdog_task, "wifi_watchdog", 2048, &local_cfg, 5, NULL);
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
