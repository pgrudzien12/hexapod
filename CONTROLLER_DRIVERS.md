# Controller Driver Implementation Guide

This document provides implementation guidelines for adding new controller input drivers (WiFi TCP, Bluetooth Classic / BLE, generic UART, etc.) to the hexapod locomotion framework.

## 1. Architecture Overview

Core separation:
- `controller.c` – Driver-agnostic core: owns shared channel buffer, mutex, connection flag, and dispatches initialization based on `controller_driver_type_e`.
- Driver source file – Implements hardware/transport specifics; runs its own FreeRTOS task; converts incoming raw frames into the canonical channel array (14 × uint16_t, 1000..2000 typical RC range) via `controller_internal_update_channels()`.
- Higher layers (`user_command.c`, gait/IK) remain transport-agnostic, using `controller_get_channels()` and `controller_decode()`.

## 2. Channel Semantics

`CONTROLLER_MAX_CHANNELS` = 32. Each channel is a signed 16‑bit value in the full range -32768..32767. Drivers should:
1. Produce values already in this symmetric range (e.g. joystick axes, virtual inputs), OR
2. Map their native transport units into this range using a linear transform.

FlySky iBUS (1000..2000) is internally rescaled to this signed range for the first 14 channels; remaining channels are zeroed.
Maintain consistency so decoding logic (deadband, normalization) remains transport‑agnostic.

## 3. Driver Task Lifecycle

Typical pattern inside driver file (e.g., `controller_wifi_tcp.c`):
1. Acquire configuration via `controller_internal_get_driver_cfg(&size)` and cast after verifying size matches your config struct.
2. Initialize transport (UART, socket, BT stack, etc.).
3. Enter loop:
   - Block on I/O with a finite timeout (e.g., 20–50 ms) to allow periodic timeout checks.
   - On receiving a complete frame, parse/validate, fill `uint16_t local[CONTROLLER_MAX_CHANNELS]`, then call `controller_internal_update_channels(local)`.
   - Track last successful frame tick; if elapsed >1s (or driver-specific timeout) and previously connected, log a warning and call `controller_internal_set_failsafe()` + `controller_internal_set_connected(false)`.
   - When first valid frame arrives, set `controller_internal_set_connected(true)` and log a concise info message.
4. Never busy-wait; always yield via blocking I/O or `vTaskDelay()`.

## 4. Internal Helper Functions

Accessible through `controller_internal.h`:
- `controller_internal_update_channels(const uint16_t *src)` – Copies channel set atomically under mutex.
- `controller_internal_set_failsafe()` – Writes predefined safe neutral channels.
- `controller_internal_set_connected(bool)` / `controller_internal_is_connected()` – Connection state tracking.
- `controller_internal_get_config()` – Core config (task parameters & driver type).
- `controller_internal_get_driver_cfg(size_t *sz)` – Opaque pointer to driver-specific config structure.

All helpers are thread-safe where needed; do not retain raw pointers to channel memory—always push updates via helper.

## 5. Driver-Specific Configuration

Each driver defines a POD struct, e.g.:
```c
typedef struct {
    int uart_port;
    int rx_gpio;
    int tx_gpio;
    int baud_rate;
} controller_flysky_ibus_cfg_t;
```
Provide a `*_default()` inline initializer. Users set up a static instance and pass it via `controller_config_t.driver_cfg` with `driver_cfg_size = sizeof(struct)`. The core does a shallow store; lifetime remains with the caller (static/global recommended). Future enhancement could deep copy + persist to NVS.

### Validation
At driver start, check `size == sizeof(your_cfg_struct)`. If mismatch, log warning and fallback to internal defaults.

## 6. Failsafe Strategy

Failsafe fills channels with deterministic neutral values (center sticks, low throttle / disarm). Avoid injecting stale or partial data. On reconnect, simply overwrite with fresh frames.

Timeout threshold: 1000 ms is default; adapt (or make configurable) if your transport expects slower update rates. Keep the threshold generous enough to avoid false negatives but fast enough to trigger safe state quickly.

## 7. Packet / Frame Design Guidelines (Non-UART Examples)

When designing a new protocol (e.g., WiFi TCP):

Standard WiFi TCP driver frame (Version 1):
```
Offset Size Field
0      2    Sync 0xAA 0x55
2      1    Version (0x01)
3      1    Flags (bit0: channels present; others reserved)
4      2    Sequence (uint16 LE)
6      2    Payload length (expected 64 for 32 * int16)
8      64   Channels (32 * int16 LE, -32768..32767)
72     2    CRC16-CCITT over bytes 0..71 (poly 0x1021, init 0xFFFF)
Total: 74 bytes
```
Frames with unsupported version or wrong length are discarded. CRC mismatch frames are dropped (if configured `require_crc=true`).

Validation steps:
1. Search stream for sync pattern.
2. Verify remaining bytes available; if using CRC, compute and compare.
3. Channels outside signed int16 range (should not happen) are saturated; higher-level semantic validation can clamp further.

For BLE: Consider using a fixed-length characteristic (e.g. 32 bytes) for atomic notifications.

## 8. Threading & Concurrency

Only the driver task writes channels. Higher layers perform reads via `controller_get_channels()` which acquires the mutex briefly. Keep driver updates <1 kHz to minimize contention (typical RC is 50–200 Hz).

Avoid long blocking operations while holding the mutex—helpers already handle locking internally; do not add external locking unless batching multiple operations (rare).

## 9. Logging Conventions

Use concise tags:
- Driver tag: `ctrl_wifi`, `ctrl_bt`, `ctrl_flysky_ibus`.
- Log levels: `ESP_LOGI` on initial connect, `ESP_LOGW` on disconnect, `ESP_LOGD` for optional per-frame diagnostics (guard behind Kconfig flag to avoid log spam).

## 10. Error Handling

Gracefully recover from transient I/O errors:
- If read() returns <0, log at debug level and continue.
- If framing error (bad header / CRC), increment a local counter; optionally every N errors log a warning.
- Do not crash or assert on malformed input—fallback to timeout-based failsafe if frames stop arriving.

## 11. Extensibility Hooks (Future)

Potential features to keep in mind while designing drivers:
- Runtime driver switching: Implement a deinit hook to stop the current task and free driver resources before starting a new driver.
- NVS persistence: A small header (driver_type, version, size) followed by config blob.
- Mixed sources / arbitration: In future, multiple drivers could feed a supervisor that chooses active source—design packet parsing to be reentrant.

## 12. Minimal Template Skeleton

```c
// controller_myproto.c
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "controller_internal.h"
#include "controller_myproto.h" // your config struct

static const char *TAG = "ctrl_myproto";

static void myproto_task(void *arg) {
    const controller_config_t *core = controller_internal_get_config();
    size_t sz=0; const void *p = controller_internal_get_driver_cfg(&sz);
    const controller_myproto_cfg_t *cfg = NULL; controller_myproto_cfg_t local;
    if (p && sz==sizeof(*cfg)) cfg = (const controller_myproto_cfg_t*)p; else { local = controller_myproto_default(); cfg = &local; }

    controller_internal_set_connected(false);
    // init transport using cfg...

    TickType_t last_frame = xTaskGetTickCount();
    while (1) {
        // read / parse frame
        bool got = false; uint16_t ch[CONTROLLER_MAX_CHANNELS];
        if (got) {
            controller_internal_update_channels(ch);
            last_frame = xTaskGetTickCount();
            if (!controller_internal_is_connected()) { controller_internal_set_connected(true); ESP_LOGI(TAG, "connected"); }
        }
        // timeout
        if (controller_internal_is_connected() && (xTaskGetTickCount()-last_frame) > pdMS_TO_TICKS(cfg->timeout_ms)) {
            controller_internal_set_connected(false);
            controller_internal_set_failsafe();
            ESP_LOGW(TAG, "disconnected: timeout");
        }
    }
}

void controller_driver_init_myproto(const controller_config_t *core) {
    xTaskCreate(myproto_task, "ctrl_myproto", core->task_stack, NULL, core->task_prio, NULL);
}
```

## 13. Testing Checklist

- [ ] Valid frame updates channels (inspect via temporary debug log after `controller_get_channels`).
- [ ] Disconnect triggers failsafe within expected timeout.
- [ ] Reconnect recovers without reboot.
- [ ] Out-of-range channel values are clamped or rejected.
- [ ] Task stack usage within budget (use `uxTaskGetStackHighWaterMark`).
- [ ] No watchdog resets under sustained 200 Hz input.

## 14. Performance Considerations

- Keep parsing O(N) with small constants (N=frame length). Avoid dynamic allocation in the hot path.
- If transport supports burst frames, process sequentially before yielding, but cap total loop time (<5 ms) to keep latency low.

## 15. Style & Naming

Prefix all internal static symbols with a short driver tag to avoid collisions. Use consistent task name pattern: `ctrl_<driver>` or `<driver>_ibus` for clarity.

---
Questions or proposing a new driver? Open a PR or issue describing the protocol, expected update rate, and any security/auth requirements (especially for WiFi/BLE sources).
