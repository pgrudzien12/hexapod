#include "leg_debugger.h"
#include "robot_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG_LDBG = "leg_dbg";

static struct {
    int enabled;
    int leg_index;
    float delta_thresh;
    unsigned int min_interval_ms;
    float last_angles[3];
    unsigned int last_log_ms;
    int have_last;
} s_dbg;

static inline float fabsf_local(float x){ return x < 0 ? -x : x; }

void leg_debugger_init(int enabled, int leg_index, float delta_thresh_rad, unsigned int min_interval_ms)
{
    memset(&s_dbg, 0, sizeof(s_dbg));
    s_dbg.enabled = enabled;
    s_dbg.leg_index = leg_index;
    s_dbg.delta_thresh = (delta_thresh_rad > 0.0f) ? delta_thresh_rad : 0.0f;
    s_dbg.min_interval_ms = min_interval_ms;
}

void leg_debugger_update(const whole_body_cmd_t *cmds)
{
    if (!s_dbg.enabled || !cmds) return;
    const int i = s_dbg.leg_index;
    if (i < 0 || i >= NUM_LEGS) return;
    const float coxa  = cmds->joint_cmds[i].joint_angles[0];
    const float femur = cmds->joint_cmds[i].joint_angles[1];
    const float tibia = cmds->joint_cmds[i].joint_angles[2];

    unsigned int now_ms = (unsigned int)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (!s_dbg.have_last) {
        s_dbg.last_angles[0] = coxa;
        s_dbg.last_angles[1] = femur;
        s_dbg.last_angles[2] = tibia;
        s_dbg.last_log_ms = now_ms;
        s_dbg.have_last = 1;
        ESP_LOGI(TAG_LDBG, "leg %d initial angles: c=%.3f f=%.3f t=%.3f", i, coxa, femur, tibia);
        return;
    }

    float dc = coxa - s_dbg.last_angles[0];
    float df = femur - s_dbg.last_angles[1];
    float dt = tibia - s_dbg.last_angles[2];
    float thr = s_dbg.delta_thresh;
    unsigned int dms = now_ms - s_dbg.last_log_ms;
    if ((fabsf_local(dc) >= thr) || (fabsf_local(df) >= thr) || (fabsf_local(dt) >= thr)) {
        if (dms >= s_dbg.min_interval_ms) {
            ESP_LOGI(TAG_LDBG, "leg %d changed: c=%.3f(%+.3f) f=%.3f(%+.3f) t=%.3f(%+.3f)",
                     i, coxa, dc, femur, df, tibia, dt);
            s_dbg.last_angles[0] = coxa;
            s_dbg.last_angles[1] = femur;
            s_dbg.last_angles[2] = tibia;
            s_dbg.last_log_ms = now_ms;
        }
    }
}
