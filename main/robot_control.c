#include "robot_control.h"
#include "esp_log.h"
#include "esp_check.h"
#include "robot_config.h"
#include "driver/mcpwm_prelude.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG_RC = "robot_control";

// Simple MCPWM context per servo channel (leg,joint)
typedef struct {
    bool inited;
    int group_id;
    int gpio;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t gen;
} pwm_chan_t;

static pwm_chan_t g_pwm[NUM_LEGS][3];

// Constants for 50Hz hobby servo
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1 MHz
#define SERVO_TIMEBASE_PERIOD        20000    // 20 ms

// Simple clamp helper
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int map_angle_to_pwm_us(float radians, const joint_calib_t *cal) {
    // Apply offset and inversion, then clamp
    float q = (cal ? (float)cal->invert_sign : 1.0f) * (radians + (cal ? cal->zero_offset_rad : 0.0f));
    float qmin = cal ? cal->min_rad : (-M_PI * 0.5f);
    float qmax = cal ? cal->max_rad : ( M_PI * 0.5f);
    float r = clampf(q, qmin, qmax);
    // Map to PWM
    int pwm_min = cal ? cal->pwm_min_us : 500;
    int pwm_max = cal ? cal->pwm_max_us : 2500;
    float t = (r - qmin) / (qmax - qmin);
    int us = (int)(pwm_min + t * (float)(pwm_max - pwm_min));
    return us;
}

// For now, we only log the intended PWM. Actual MCPWM output will be added later.
esp_err_t robot_set_joint_angle_rad(int leg_index, leg_servo_t joint, float radians) {
    const joint_calib_t *cal = robot_config_get_joint_calib(leg_index, joint);
    int gpio = robot_config_get_servo_gpio(leg_index, joint);
    int group = robot_config_get_mcpwm_group(leg_index);
    int pwm_us = map_angle_to_pwm_us(radians, cal);

    // Lazy initialize channel
    pwm_chan_t *ch = &g_pwm[leg_index][(int)joint];
    if (!ch->inited) {
        if (gpio < 0) {
            ESP_LOGW(TAG_RC, "leg %d joint %d has no GPIO assigned; skipping output", leg_index, (int)joint);
            return ESP_OK;
        }
        ch->group_id = group;
        ch->gpio = gpio;
        // Timer
        mcpwm_timer_config_t tcfg = {
            .group_id = group,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
            .period_ticks = SERVO_TIMEBASE_PERIOD,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tcfg, &ch->timer), TAG_RC, "timer");
        // Operator
        mcpwm_operator_config_t ocfg = { .group_id = group };
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&ocfg, &ch->oper), TAG_RC, "oper");
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(ch->oper, ch->timer), TAG_RC, "connect");
        // Comparator and generator
        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(ch->oper, &ccfg, &ch->cmpr), TAG_RC, "cmpr");
        mcpwm_generator_config_t gcfg = { .gen_gpio_num = gpio };
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(ch->oper, &gcfg, &ch->gen), TAG_RC, "gen");
        // Actions: HIGH at empty, LOW at compare
        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(
            ch->gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
        ), TAG_RC, "gen-timer-act");
        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(
            ch->gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, ch->cmpr, MCPWM_GEN_ACTION_LOW)
        ), TAG_RC, "gen-cmp-act");
        // Start timer
        ESP_RETURN_ON_ERROR(mcpwm_timer_enable(ch->timer), TAG_RC, "timer-enable");
        ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_START_NO_STOP), TAG_RC, "timer-start");
        ch->inited = true;
        ESP_LOGI(TAG_RC, "Initialized MCPWM for leg %d joint %d on GPIO %d (group %d)", leg_index, (int)joint, gpio, group);
    }

    // Apply compare based on desired pwm_us
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(ch->cmpr, (uint32_t)pwm_us), TAG_RC, "set cmp");
    ESP_LOGV(TAG_RC, "leg %d joint %d -> rad=%.3f => %dus", leg_index, (int)joint, radians, pwm_us);
    return ESP_OK;
}

void robot_execute(const whole_body_cmd_t *cmds) {
    if (!cmds) return;
    // The whole_body_cmd_t currently carries IK joint angles in radians per leg (coxa, femur, tibia).
    for (int i = 0; i < NUM_LEGS; ++i) {
        const float coxa  = cmds->joint_cmds[i].joint_angles[0];
        const float femur = cmds->joint_cmds[i].joint_angles[1];
        const float tibia  = cmds->joint_cmds[i].joint_angles[2];
        // Send to actuator layer (calibration and limits will be applied there later)
        (void)robot_set_joint_angle_rad(i, LEG_SERVO_COXA,  coxa);
        (void)robot_set_joint_angle_rad(i, LEG_SERVO_FEMUR, femur);
        (void)robot_set_joint_angle_rad(i, LEG_SERVO_TIBIA, tibia);
    }
}
