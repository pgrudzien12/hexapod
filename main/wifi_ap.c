 #include "wifi_ap.h"
 #include "esp_wifi.h"
 #include "esp_event.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 #include "esp_system.h"
 #include "esp_random.h"
 #include <string.h>
 #include <stdio.h>
 #include <stdlib.h>

static const char *TAG = "wifi_ap";
static bool g_started = false;
static char g_active_ssid[33]; // 32 char max + null

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "HEXAPOD_AP"
#endif
#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "HEXAPOD_ESP32"
#endif

static void build_ssid(char *out, size_t out_sz, const wifi_ap_options_t *opt) {
    if (!out || out_sz == 0) return;
    out[0] = '\0';
    if (!opt) {
        snprintf(out, out_sz, "HEXAPOD_AP");
        return;
    }
    switch (opt->mode) {
        case WIFI_AP_SSID_FIXED:
            if (opt->fixed_ssid) snprintf(out, out_sz, "%s", opt->fixed_ssid);
            else snprintf(out, out_sz, "HEXAPOD_AP");
            break;
        case WIFI_AP_SSID_MAC_SUFFIX: {
            uint8_t mac[6];
            esp_wifi_get_mac(WIFI_IF_AP, mac); // ignore error, mac defaults to zeros
            const char *pfx = opt->fixed_prefix ? opt->fixed_prefix : "HEXAPOD";
            snprintf(out, out_sz, "%s_%02X%02X%02X", pfx, mac[3], mac[4], mac[5]);
            break; }
        case WIFI_AP_SSID_RANDOM_SUFFIX: {
            const char *pfx = opt->fixed_prefix ? opt->fixed_prefix : "HEXAPOD";
            uint32_t r = esp_random();
            snprintf(out, out_sz, "%s_%04X", pfx, (unsigned)(r & 0xFFFF));
            break; }
        default:
            snprintf(out, out_sz, "HEXAPOD_AP");
            break;
    }
}

bool wifi_ap_init_with_options(const wifi_ap_options_t *opt) {
    if (g_started) return true;
    esp_err_t err;
    // Init NVS (needed by WiFi driver). Ignore errors if already inited.
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_cfg = {0};
    build_ssid(g_active_ssid, sizeof(g_active_ssid), opt);
    strncpy((char*)ap_cfg.ap.ssid, g_active_ssid, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid[sizeof(ap_cfg.ap.ssid) - 1] = '\0'; // Ensure null-termination
    ap_cfg.ap.ssid_len = 0; // null-terminated
    const char *pass = (opt && opt->password) ? opt->password : WIFI_AP_PASS;
    snprintf((char*)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", pass);
    ap_cfg.ap.channel = (opt && opt->channel) ? opt->channel : 1;
    ap_cfg.ap.max_connection = (opt && opt->max_clients) ? opt->max_clients : 4;
    ap_cfg.ap.authmode = (pass && strlen(pass) > 0) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP started SSID='%s' PASS='%s'", g_active_ssid, pass ? pass : "<open>");
    g_started = true;
    return true;
}

bool wifi_ap_init_once(void) {
    wifi_ap_options_t def = {
        .mode = WIFI_AP_SSID_MAC_SUFFIX,
        .fixed_prefix = "HEXAPOD",
        .fixed_ssid = "HEXAPOD_AP",
        .password = WIFI_AP_PASS,
        .channel = 1,
        .max_clients = 4,
    };
    return wifi_ap_init_with_options(&def);
}

const char *wifi_ap_get_ssid(void) {
    return g_started ? g_active_ssid : NULL;
}
