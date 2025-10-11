#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Simple AP-only WiFi bring-up.
// Default password: HEXAPOD_ESP32 (WPA2). For production replace with per-device randomized credentials.

typedef enum {
	WIFI_AP_SSID_FIXED = 0,      // Use provided fixed_ssid
	WIFI_AP_SSID_MAC_SUFFIX,     // fixed_prefix + last 3 bytes of MAC (e.g., HEXAPOD_ABC123)
	WIFI_AP_SSID_RANDOM_SUFFIX   // fixed_prefix + 4 random hex chars
} wifi_ap_ssid_mode_e;

typedef struct {
	wifi_ap_ssid_mode_e mode;
	const char *fixed_prefix;   // used for MAC or RANDOM suffix modes
	const char *fixed_ssid;     // used when mode == FIXED
	const char *password;       // NULL -> open network
	uint8_t channel;            // 1..13 (or 0 for default 1)
	uint8_t max_clients;        // default 4
} wifi_ap_options_t;

// Initialize AP once with options (if called multiple times, original config retained)
bool wifi_ap_init_with_options(const wifi_ap_options_t *opt);

// Convenience: legacy fixed init (calls wifi_ap_init_with_options with defaults)
bool wifi_ap_init_once(void);

// Retrieve the actual SSID in use (NULL if not started yet)
const char *wifi_ap_get_ssid(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_AP_H
