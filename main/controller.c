// Controller abstraction layer: maintains shared channel state; individual
// driver implementations (e.g., FlySky iBUS, WiFi TCP, BT) push updates via
// internal helper functions. This file holds only driver-agnostic logic.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "controller.h"

static const char *TAG_CTRL = "controller";

// Global runtime configuration (opaque driver_cfg pointer)
static controller_config_t g_cfg = {
    .driver_type = CONTROLLER_DRIVER_FLYSKY_IBUS,
    .task_stack = 4096,
    .task_prio = 10,
    .driver_cfg = NULL,
    .driver_cfg_size = 0,
};

// | Control                 | Function                  | Notes                         |
// | ----------------------- | ------------------------- | ----------------------------- |
// | Right Vert (non-center) | Body height $z\_target$   | Slew-limited; capture on arm  |
// | Right Horiz             | Lateral shift $y\_target$ | Small deadband                |
// | Left Vert               | Forward speed $v\_x$      | Scaled by SRA × SWC           |
// | Left Horiz              | Turn rate $\omega\_z$     | Expo recommended              |
// | SRA (knob)              | Step frequency scale      | 0–100% inside SWC caps        |
// | SWA (2)                 | Arm/Disarm                | Safety, soft-start            |
// | SWB (2)                 | Walk ↔ Pose mode          | Pose: left stick → roll/pitch |
// | SWC (3)                 | Gait                      | Wave / Ripple / Tripod        |
// | SWD (2)                 | Terrain profile           | Climb / Normal               |


static SemaphoreHandle_t g_ch_mutex;
static int16_t g_channels[CONTROLLER_MAX_CHANNELS];
static volatile uint32_t g_last_update_ms;
static volatile bool g_connected;             // generic connection state

// Fill default failsafe channels
static void controller_fill_failsafe(int16_t out[CONTROLLER_MAX_CHANNELS])
{
    // Neutral (center) = 0 for all axes; switches off (0); gait selection default later in decode.
    for (int i = 0; i < CONTROLLER_MAX_CHANNELS; ++i) out[i] = 0;
}

// ========== Internal helper API (for driver .c files) ======================
// To avoid exposing these externally yet, we use forward decls; a later
// controller_internal.h could formalize them when more drivers arrive.
bool controller_internal_lock(uint32_t timeout_ms)
{
    if (!g_ch_mutex) return false;
    return xSemaphoreTake(g_ch_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void controller_internal_unlock(void)
{
    if (g_ch_mutex) xSemaphoreGive(g_ch_mutex);
}

void controller_internal_set_connected(bool connected)
{
    g_connected = connected;
}

bool controller_internal_is_connected(void)
{
    return g_connected;
}

void controller_internal_update_channels(const int16_t *src)
{
    if (!src) return;
    if (!g_ch_mutex) return;
    if (xSemaphoreTake(g_ch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(g_channels, src, sizeof(int16_t)*CONTROLLER_MAX_CHANNELS);
        g_last_update_ms = (uint32_t)xTaskGetTickCount();
        xSemaphoreGive(g_ch_mutex);
    }
}

void controller_internal_set_failsafe(void)
{
    int16_t tmp[CONTROLLER_MAX_CHANNELS];
    controller_fill_failsafe(tmp);
    controller_internal_update_channels(tmp);
}

const controller_config_t *controller_internal_get_config(void) { return &g_cfg; }
const void *controller_internal_get_driver_cfg(size_t *out_size)
{
    if (out_size) *out_size = g_cfg.driver_cfg_size;
    return g_cfg.driver_cfg;
}

// Forward declaration of implemented driver init functions
void controller_driver_init_flysky_ibus(const controller_config_t *cfg);
void controller_driver_init_bt_classic(const controller_config_t *cfg);
void controller_driver_init_wifi_tcp(const controller_config_t *cfg);

void controller_init(const controller_config_t *cfg)
{
    if (cfg) {
        g_cfg.driver_type = cfg->driver_type;
        g_cfg.task_stack = cfg->task_stack;
        g_cfg.task_prio = cfg->task_prio;
        g_cfg.driver_cfg = cfg->driver_cfg;
        g_cfg.driver_cfg_size = cfg->driver_cfg_size;
    }
    if (!g_ch_mutex) {
        g_ch_mutex = xSemaphoreCreateMutex();
    }
    controller_fill_failsafe(g_channels);
    switch (g_cfg.driver_type) {
        case CONTROLLER_DRIVER_WIFI_TCP:
            controller_driver_init_wifi_tcp(&g_cfg);
            break;
        case CONTROLLER_DRIVER_BT_CLASSIC:
            controller_driver_init_bt_classic(&g_cfg);
            break;
        case CONTROLLER_DRIVER_FLYSKY_IBUS:
        default:
            controller_driver_init_flysky_ibus(&g_cfg);
            break;
    }
}

bool controller_get_channels(int16_t out[CONTROLLER_MAX_CHANNELS])
{
    if (!out) return false;
    if (!g_ch_mutex) return false;
    if (xSemaphoreTake(g_ch_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    memcpy(out, g_channels, sizeof(int16_t) * CONTROLLER_MAX_CHANNELS);
    xSemaphoreGive(g_ch_mutex);
    return true;
}

static float map_signed_norm(int16_t v)
{
    // Map int16 full scale to -1..+1 (symmetric). INT16_MIN maps to -1, INT16_MAX to +1.
    if (v <= -32768) return -1.0f;
    if (v >= 32767) return 1.0f;
    return (float)v / 32767.0f;
}

// Linear deadband with range preservation
static float apply_deadband(float v, float d)
{
    float av = (v < 0.0f) ? -v : v;
    if (av <= d) {
        return 0.0f;
    }
    float s = (v < 0.0f) ? -1.0f : 1.0f;
    float o = (av - d) / (1.0f - d);
    return s * o;
}

void controller_decode(const int16_t ch[CONTROLLER_MAX_CHANNELS], controller_state_t *out)
{
    if (!ch || !out) {
        return;
    }
    // Apply small deadband to sticks to reject noise near center
    // TODO: Make DEAD_BAND configurable via Kconfig or runtime configuration
    const float DEAD_BAND = 0.04f; // ~4%

    out->right_horiz = apply_deadband(map_signed_norm(ch[0]), DEAD_BAND); // CH1: Right Horiz -> lateral shift
    out->left_vert   = apply_deadband(map_signed_norm(ch[1]), DEAD_BAND); // CH2: Left Vert -> forward speed
    out->right_vert  = apply_deadband(map_signed_norm(ch[2]), DEAD_BAND); // CH3: Right Vert -> z_target
    out->left_horiz  = apply_deadband(map_signed_norm(ch[3]), DEAD_BAND); // CH4: Left Horiz -> yaw rate

    // Switch style channels now treated as signed int16 where positive => true.
    out->swa_arm  = ch[4] > 0;            // CH5: SWA
    out->swb_pose = ch[6] > 0;            // CH7: SWB

    // Gait selection from CH8: map ranges of signed value to modes.
    // Use thirds: [-32768,-10923] TRIPOD, (-10923,10923] RIPPLE, (10923,32767] WAVE.
    int16_t gsel = ch[7];
    if (gsel <= -10923) {
        out->swc_gait = GAIT_MODE_TRIPOD;
    } else if (gsel <= 10923) {
        out->swc_gait = GAIT_MODE_RIPPLE;
    } else {
        out->swc_gait = GAIT_MODE_WAVE;
    }
    out->swd_terrain = ch[8] > 0;         // CH9: SWD

    // Knob (CH10): map -32768..32767 to 0..1
    int16_t raw = ch[9];
    out->sra_knob = ((float)raw + 32768.0f) / 65535.0f; // map full signed to 0..1
}

static inline bool float_eq_eps(float a, float b, float eps)
{
    float d = a - b;
    if (d < 0.0f) d = -d;
    return d <= eps;
}

bool controller_user_command_equal(const user_command_t *a, const user_command_t *b, float tol)
{
    if (a == b) {
        return true; // covers both NULL and same pointer
    }
    if (!a || !b) {
        return false;
    }
    float eps = (tol > 0.0f) ? tol : CONTROLLER_CMD_FLOAT_EPSILON;
    if (!float_eq_eps(a->vx, b->vx, eps)) return false;
    if (!float_eq_eps(a->wz, b->wz, eps)) return false;
    if (!float_eq_eps(a->z_target, b->z_target, eps)) return false;
    if (!float_eq_eps(a->y_offset, b->y_offset, eps)) return false;
    if (!float_eq_eps(a->step_scale, b->step_scale, eps)) return false;
    if (a->gait != b->gait) return false;
    if (a->enable != b->enable) return false;
    if (a->pose_mode != b->pose_mode) return false;
    if (a->terrain_climb != b->terrain_climb) return false;
    return true;
}
