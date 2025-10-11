// WiFi TCP controller driver: listens on a TCP port and ingests binary channel frames.
#include <string.h>
#include <errno.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/errno.h"
#include "lwip/sockets.h"

#include "controller_internal.h"
#include "controller_wifi_tcp.h"
#include "wifi_ap.h"

static const char *TAG = "ctrl_wifi";

// CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xor-out)
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

static bool read_full(int sock, uint8_t *buf, size_t n, int timeout_ms) {
    size_t off = 0;
    while (off < n) {
        int to = timeout_ms;
        struct timeval tv = { .tv_sec = to / 1000, .tv_usec = (to % 1000) * 1000 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int r = recv(sock, (char*)buf + off, (int)(n - off), 0);
        if (r <= 0) return false;
        off += (size_t)r;
    }
    return true;
}

static void wifi_tcp_client_task(void *arg) {
    int client_sock = (int)(intptr_t)arg;
    controller_internal_set_connected(true);
    ESP_LOGI(TAG, "client connected fd=%d", client_sock);

    TickType_t last_frame = xTaskGetTickCount();
    int16_t channels[WIFI_CTRL_NUM_CHANNELS];
    memset(channels, 0, sizeof(channels));

    while (1) {
        uint8_t header[WIFI_CTRL_BASE_HEADER];
        if (!read_full(client_sock, header, WIFI_CTRL_BASE_HEADER, 200)) {
            // timeout / disconnect
            if ((xTaskGetTickCount() - last_frame) > pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "client timeout, closing");
                break;
            }
            continue; // allow timeout logic
        }
        if (header[0] != WIFI_CTRL_SYNC0 || header[1] != WIFI_CTRL_SYNC1) {
            ESP_LOGW(TAG, "bad sync %02X %02X", header[0], header[1]);
            break;
        }
        uint8_t ver = header[2];
        if (ver != WIFI_CTRL_PROTO_VERSION) {
            ESP_LOGW(TAG, "unsupported version %u", (unsigned)ver);
            break;
        }
        uint8_t flags = header[3];
        uint16_t seq = (uint16_t)(header[4] | (header[5] << 8));
        uint16_t payload_len = (uint16_t)(header[6] | (header[7] << 8));
        if (payload_len != WIFI_CTRL_CHANNEL_BYTES) {
            ESP_LOGW(TAG, "unexpected payload_len=%u", (unsigned)payload_len);
            // attempt to drain and continue? For now break.
            break;
        }
        uint8_t payload[WIFI_CTRL_CHANNEL_BYTES];
        if (!read_full(client_sock, payload, WIFI_CTRL_CHANNEL_BYTES, 50)) {
            ESP_LOGW(TAG, "payload read error");
            break;
        }
        uint8_t crc_buf[2];
        if (!read_full(client_sock, crc_buf, 2, 50)) {
            ESP_LOGW(TAG, "crc read error");
            break;
        }
        uint16_t rx_crc = (uint16_t)(crc_buf[0] | (crc_buf[1] << 8));
        uint16_t calc_crc = crc16_ccitt(header, WIFI_CTRL_BASE_HEADER);
        calc_crc = crc16_ccitt(payload, WIFI_CTRL_CHANNEL_BYTES) ^ calc_crc; // incorrect combination deliberately replaced below
        // Recompute correctly over contiguous buffer (simpler):
        uint8_t full_tmp[WIFI_CTRL_BASE_HEADER + WIFI_CTRL_CHANNEL_BYTES];
        memcpy(full_tmp, header, WIFI_CTRL_BASE_HEADER);
        memcpy(full_tmp + WIFI_CTRL_BASE_HEADER, payload, WIFI_CTRL_CHANNEL_BYTES);
        calc_crc = crc16_ccitt(full_tmp, sizeof(full_tmp));
        if (rx_crc != calc_crc) {
            ESP_LOGW(TAG, "CRC mismatch rx=%04X calc=%04X", rx_crc, calc_crc);
            // drop frame, continue
            continue;
        }
        // Convert payload to int16_t
        for (int i = 0; i < WIFI_CTRL_NUM_CHANNELS; ++i) {
            int16_t v = (int16_t)(payload[2*i] | (payload[2*i+1] << 8));
            channels[i] = v;
        }
        controller_internal_update_channels(channels);
        last_frame = xTaskGetTickCount();
    }

    close(client_sock);
    controller_internal_set_connected(false);
    controller_internal_set_failsafe();
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
    size_t cfg_sz=0; const void *p = controller_internal_get_driver_cfg(&cfg_sz);
    controller_wifi_tcp_cfg_t local = controller_wifi_tcp_default();
    const controller_wifi_tcp_cfg_t *cfg = &local;
    if (p && cfg_sz == sizeof(controller_wifi_tcp_cfg_t)) cfg = (const controller_wifi_tcp_cfg_t*)p;

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
    if (listen(listen_fd, cfg->max_clients) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "WiFi controller listening on port %u", (unsigned)cfg->listen_port);

    while (1) {
        struct sockaddr_in caddr; socklen_t clen = sizeof(caddr);
        int client = accept(listen_fd, (struct sockaddr*)&caddr, &clen);
        if (client < 0) {
            ESP_LOGW(TAG, "accept error errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(cfg->connect_backoff_ms));
            continue;
        }
        // Spawn client handler task
        xTaskCreate(wifi_tcp_client_task, "ctrl_wifi_cli", 4096, (void*)(intptr_t)client, 9, NULL);
    }
}

void controller_driver_init_wifi_tcp(const controller_config_t *core) {
    xTaskCreate(wifi_tcp_server_task, "ctrl_wifi_srv", core->task_stack, NULL, core->task_prio, NULL);
}
