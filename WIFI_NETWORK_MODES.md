# WiFi Network Modes Strategy

This file captures the planned evolution of network connectivity for the hexapod.

## Current (Phase 1) – AP Only (Option A)
- Mode: `WIFI_MODE_AP`
- SSID: Configurable / generated (see "SSID Generation Modes" below). Default build behavior: MAC-suffix mode (e.g. `HEXAPOD_AP_3AF2B7`).
- Password: Default `HEXAPOD_ESP32` (WPA2). Can be overridden at compile time or future config portal.
- Purpose: Fast bring-up to test WiFi TCP controller & future config portal without dependency on external infrastructure.
- Pros:
  * Always reachable for controller frames + debugging
  * Minimal configuration complexity
  * Works in the field without other networks
- Cons:
  * Clients must switch networks (no simultaneous internet through this AP)
  * Default password not unique (not secure for production)
  * No automatic time sync via NTP unless separately handled (can add later)

### SSID Generation Modes (Implemented)
The AP helper (`wifi_ap.c`) supports multiple deterministic or semi‑random SSID strategies via `wifi_ap_options_t`:

| Mode | Description | Example |
| ---- | ----------- | ------- |
| `WIFI_AP_SSID_MODE_FIXED` | Uses the provided prefix string verbatim. | `HEXAPOD_AP` |
| `WIFI_AP_SSID_MODE_MAC_SUFFIX` | Appends last 3 bytes of the station MAC as 6 hex chars. | `HEXAPOD_AP_3AF2B7` |
| `WIFI_AP_SSID_MODE_RANDOM_SUFFIX` | Appends N random hex chars (1–8) derived from `esp_random()`. | `HEXAPOD_AP_A94C21` |

Options structure fields:
```
typedef struct {
    const char *ssid_prefix;   // Base prefix (required)
    const char *password;      // WPA2 password (>=8 chars)
    wifi_ap_ssid_mode_t ssid_mode; // One of the modes above
    int random_hex_len;        // 1..8 when RANDOM_SUFFIX
    int channel;               // WiFi channel (1 default)
    int max_connections;       // Max clients (default 4)
} wifi_ap_options_t;
```

Rationale:
* MAC suffix gives stable uniqueness across reboots and devices.
* Random suffix lets you spin up multiple temporary test units simultaneously without ambiguity.
* Fixed mode eases scripted connection in controlled single‑device environments.

Future enhancements (Phase 2+) may derive the password automatically from the MAC or a provisioning token to avoid a shared default.

## Future (Phase 2) – Dual Mode AP+STA (Option C)
- Mode: `WIFI_MODE_APSTA`
- Behavior:
  1. Attempt to connect as STA using credentials stored in NVS (if present).
  2. Maintain AP concurrently as fallback / direct maintenance channel.
  3. Optionally disable AP after stable STA connection (policy configurable) OR keep it up for guaranteed direct control.
- Credential Provisioning Paths:
  * Captive portal over AP (HTTP form) writes SSID/PASS into NVS
  * BLE provisioning using ESP-IDF provisioning manager (alternate)
- Security Enhancements:
  * Derive AP password from device unique ID (e.g., last 3 MAC bytes) on first boot
  * Add application-layer token required for control protocol frames beyond WiFi association (handshake frame before streaming channels)
  * Enable Flash Encryption + Secure Boot for at-rest credential protection
- OTA Strategy:
  * Primary: HTTPS pull (esp_https_ota) when STA connected
  * Secondary: Local upload via AP (authenticated web form) if STA unreachable

## Phase 3 – Extended Features
- Dynamic driver switching via configuration portal (RC / WiFi / BT)
- Token-based authentication for WiFi TCP control frames
- Telemetry uplink (status frames / heartbeat) over TCP or WebSocket
- Adjustable AP TX power and channel selection to minimize interference
- Optional mDNS advertisement (`_hexapod._tcp`) for discovery on STA network

## Non-Goals (for now)
- WPA3 transition (can revisit after stabilization)
- Mesh networking
- Captive portal based firmware update (lower priority until core locomotion stable)

## Migration Checklist to AP+STA
- [ ] Implement NVS credential store module
- [ ] Simple HTTP portal for credential entry
- [ ] Add handshake/auth layer to TCP control channel (optional gating)
- [ ] Policy for AP disable (config flag)
- [ ] OTA integration using esp_https_ota

---
This document will evolve; keep it updated as connectivity features progress.
