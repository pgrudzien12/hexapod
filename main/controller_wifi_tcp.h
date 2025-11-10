#ifndef CONTROLLER_WIFI_TCP_H
#define CONTROLLER_WIFI_TCP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Binary protocol (version 1) layout for channel frames over TCP:
// Offset | Size | Field
//   0    |  2   | Sync 0xAA 0x55
//   2    |  1   | Version (currently 0x01)
//   3    |  1   | Flags (bit0: channels present, bit1: reserved, others future)
//   4    |  2   | Sequence (uint16_t, little-endian, rolls over)
//   6    |  2   | Payload length in bytes (N = 64 for 32 int16 channels)
//   8    |  N   | Channel payload: 32 * int16_t little-endian
//  8+N   |  2   | CRC16-CCITT (poly 0x1021, init 0xFFFF) over bytes [0 .. 7+N]
// Total for N=64: 8 + 64 + 2 = 74 bytes.
// Future extensions may append metadata TLVs after channels; flags will indicate.

#define WIFI_CTRL_SYNC0 0xAA
#define WIFI_CTRL_SYNC1 0x55
#define WIFI_CTRL_PROTO_VERSION 0x01
#define WIFI_CTRL_NUM_CHANNELS 32
#define WIFI_CTRL_CHANNEL_BYTES (WIFI_CTRL_NUM_CHANNELS * 2)
#define WIFI_CTRL_BASE_HEADER 8
#define WIFI_CTRL_FRAME_SIZE (WIFI_CTRL_BASE_HEADER + WIFI_CTRL_CHANNEL_BYTES + 2)

// Configuration for WiFi TCP driver
// Assumes WiFi station/AP already initialized externally before starting driver.
typedef struct {
    uint16_t listen_port;      // TCP port to listen on
    uint32_t client_timeout_ms; // inactivity timeout before failsafe (e.g., 1000)
    uint32_t connect_backoff_ms; // delay before attempting to accept again after error
    uint8_t max_clients;       // normally 1; >1 means last writer wins
    bool require_crc;          // if true, frames with bad CRC dropped
} controller_wifi_tcp_cfg_t;

static inline controller_wifi_tcp_cfg_t controller_wifi_tcp_default(void) {
    controller_wifi_tcp_cfg_t c = {
        .listen_port = 5555,
        .client_timeout_ms = 1000,
        .connect_backoff_ms = 500,
        .max_clients = 1,
        .require_crc = true,
    };
    return c;
}

// Send raw textual data over active TCP connection (for RPC responses)
// Returns ESP_OK on success, ESP_FAIL if no active client or send error
esp_err_t controller_wifi_tcp_send_raw(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_WIFI_TCP_H
