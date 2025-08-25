#include "leg.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "sdkconfig.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_check.h"

// Helper: clamp
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Servo timing constants
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_RAD           (-1.57079632679f) // -90 deg in rad
#define SERVO_MAX_RAD           ( 1.57079632679f) //  90 deg in rad
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1 MHz
#define SERVO_TIMEBASE_PERIOD        20000    // 20 ms

static const char *TAG = "leg";

typedef struct leg_s {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper1;
    mcpwm_oper_handle_t oper2;
    mcpwm_oper_handle_t oper3;
    mcpwm_cmpr_handle_t cmpr1;
    mcpwm_cmpr_handle_t cmpr2;
    mcpwm_cmpr_handle_t cmpr3;
    mcpwm_gen_handle_t gen1;
    mcpwm_gen_handle_t gen2;
    mcpwm_gen_handle_t gen3;
    int gpio1, gpio2, gpio3;
    int group_id;
    // segment lengths for IK (units: user-defined, e.g., mm)
    float len_coxa, len_femur, len_tibia;
    // per-joint limits in radians
    float min_rad_coxa, max_rad_coxa;
    float min_rad_femur, max_rad_femur;
    float min_rad_tibia, max_rad_tibia;
    // calibration offsets (radians)
    float coxa_offset_rad; // hip offset
    float femur_offset_rad; // thigh offset
    float tibia_offset_rad; // knee/ankle offset
} leg_ctx_t;

static inline uint32_t angle_to_compare_rad(float radians)
{
    // clamp
    if (radians < SERVO_MIN_RAD) radians = SERVO_MIN_RAD;
    if (radians > SERVO_MAX_RAD) radians = SERVO_MAX_RAD;
    float span_us = (float)(SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);
    float norm = (radians - SERVO_MIN_RAD) / (SERVO_MAX_RAD - SERVO_MIN_RAD);
    uint32_t pulse_us = (uint32_t)(SERVO_MIN_PULSEWIDTH_US + norm * span_us);
    return pulse_us;
}

esp_err_t leg_configure(const leg_config_t* cfg, leg_handle_t* out_leg)
{
    if (!cfg || !out_leg) return ESP_ERR_INVALID_ARG;
    *out_leg = NULL;

    leg_ctx_t* leg = (leg_ctx_t*)calloc(1, sizeof(leg_ctx_t));
    if (!leg) return ESP_ERR_NO_MEM;

    leg->gpio1 = cfg->gpio_coxa;
    leg->gpio2 = cfg->gpio_femur;
    leg->gpio3 = cfg->gpio_tibia;
    leg->group_id = cfg->group_id;
    leg->len_coxa = cfg->len_coxa;
    leg->len_femur = cfg->len_femur;
    leg->len_tibia = cfg->len_tibia;
    leg->coxa_offset_rad = cfg->coxa_offset_rad;
    leg->femur_offset_rad = cfg->femur_offset_rad;
    leg->tibia_offset_rad = cfg->tibia_offset_rad;
    ESP_LOGI(TAG, "Leg lengths (mm): coxa=%.1f, femur=%.1f, tibia=%.1f", leg->len_coxa, leg->len_femur, leg->len_tibia);

    // Set angle limits in radians (defaults to [-pi/2, +pi/2] if unset by providing 0/0)
    leg->min_rad_coxa  = cfg->min_rad_coxa  == 0.0f ? SERVO_MIN_RAD : cfg->min_rad_coxa;
    leg->max_rad_coxa  = cfg->max_rad_coxa  == 0.0f ? SERVO_MAX_RAD : cfg->max_rad_coxa;
    leg->min_rad_femur = cfg->min_rad_femur == 0.0f ? SERVO_MIN_RAD : cfg->min_rad_femur;
    leg->max_rad_femur = cfg->max_rad_femur == 0.0f ? SERVO_MAX_RAD : cfg->max_rad_femur;
    leg->min_rad_tibia = cfg->min_rad_tibia == 0.0f ? SERVO_MIN_RAD : cfg->min_rad_tibia;
    leg->max_rad_tibia = cfg->max_rad_tibia == 0.0f ? SERVO_MAX_RAD : cfg->max_rad_tibia;
    // Ensure min <= max
    if (leg->min_rad_coxa > leg->max_rad_coxa) { float t = leg->min_rad_coxa; leg->min_rad_coxa = leg->max_rad_coxa; leg->max_rad_coxa = t; }
    if (leg->min_rad_femur > leg->max_rad_femur) { float t = leg->min_rad_femur; leg->min_rad_femur = leg->max_rad_femur; leg->max_rad_femur = t; }
    if (leg->min_rad_tibia > leg->max_rad_tibia) { float t = leg->min_rad_tibia; leg->min_rad_tibia = leg->max_rad_tibia; leg->max_rad_tibia = t; }
    // Default offsets to 0 if not provided
    if (isnan(leg->coxa_offset_rad)) leg->coxa_offset_rad = 0.0f;
    if (isnan(leg->femur_offset_rad)) leg->femur_offset_rad = 0.0f;
    if (isnan(leg->tibia_offset_rad)) leg->tibia_offset_rad = 0.0f;

    // Configure and start the MCPWM timer
    mcpwm_timer_config_t timer_config = {
        .group_id = leg->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    esp_err_t err;
    if ((err = mcpwm_new_timer(&timer_config, &leg->timer)) != ESP_OK) { free(leg); return err; }

    // Create three operators in the same group and connect them to the same timer
    mcpwm_operator_config_t operator_config = {.group_id = leg->group_id};
    if ((err = mcpwm_new_operator(&operator_config, &leg->oper1)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_operator(&operator_config, &leg->oper2)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_operator(&operator_config, &leg->oper3)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_operator_connect_timer(leg->oper1, leg->timer)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_operator_connect_timer(leg->oper2, leg->timer)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_operator_connect_timer(leg->oper3, leg->timer)) != ESP_OK) { free(leg); return err; }

    // Create three comparators (one per servo)
    mcpwm_comparator_config_t c_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    if ((err = mcpwm_new_comparator(leg->oper1, &c_cfg, &leg->cmpr1)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_comparator(leg->oper2, &c_cfg, &leg->cmpr2)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_comparator(leg->oper3, &c_cfg, &leg->cmpr3)) != ESP_OK) { free(leg); return err; }

    // Create three generators bound to different GPIOs
    mcpwm_generator_config_t g1 = {.gen_gpio_num = leg->gpio1};
    mcpwm_generator_config_t g2 = {.gen_gpio_num = leg->gpio2};
    mcpwm_generator_config_t g3 = {.gen_gpio_num = leg->gpio3};
    if ((err = mcpwm_new_generator(leg->oper1, &g1, &leg->gen1)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_generator(leg->oper2, &g2, &leg->gen2)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_new_generator(leg->oper3, &g3, &leg->gen3)) != ESP_OK) { free(leg); return err; }

    // Set neutral compare for all (mid-point of each joint's limits)
    float c_mid = 0.5f * (leg->min_rad_coxa + leg->max_rad_coxa);
    float f_mid = 0.5f * (leg->min_rad_femur + leg->max_rad_femur);
    float t_mid = 0.5f * (leg->min_rad_tibia + leg->max_rad_tibia);
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr1, angle_to_compare_rad(c_mid))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr2, angle_to_compare_rad(f_mid))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr3, angle_to_compare_rad(t_mid))) != ESP_OK) { free(leg); return err; }

    // Actions: go HIGH at timer empty, LOW at comparator
    if ((err = mcpwm_generator_set_action_on_timer_event(leg->gen1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_generator_set_action_on_timer_event(leg->gen2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_generator_set_action_on_timer_event(leg->gen3,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))) != ESP_OK) { free(leg); return err; }

    if ((err = mcpwm_generator_set_action_on_compare_event(leg->gen1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, leg->cmpr1, MCPWM_GEN_ACTION_LOW))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_generator_set_action_on_compare_event(leg->gen2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, leg->cmpr2, MCPWM_GEN_ACTION_LOW))) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_generator_set_action_on_compare_event(leg->gen3,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, leg->cmpr3, MCPWM_GEN_ACTION_LOW))) != ESP_OK) { free(leg); return err; }

    // Start timer
    if ((err = mcpwm_timer_enable(leg->timer)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_timer_start_stop(leg->timer, MCPWM_TIMER_START_NO_STOP)) != ESP_OK) { free(leg); return err; }

    *out_leg = (leg_handle_t)leg;
    return ESP_OK;
}

esp_err_t leg_test_neutral(leg_handle_t h)
{
    leg_ctx_t* leg = (leg_ctx_t*)h;
    if (!leg) return ESP_ERR_INVALID_STATE;
    float c_mid = 0.5f * (leg->min_rad_coxa + leg->max_rad_coxa);
    float f_mid = 0.5f * (leg->min_rad_femur + leg->max_rad_femur);
    float t_mid = 0.5f * (leg->min_rad_tibia + leg->max_rad_tibia);
    ESP_LOGI(TAG, "Neutral angles (rad): coxa=%.3f femur=%.3f tibia=%.3f", c_mid, f_mid, t_mid);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr1, angle_to_compare_rad(c_mid)), TAG, "cmp1 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr2, angle_to_compare_rad(f_mid)), TAG, "cmp2 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr3, angle_to_compare_rad(t_mid)), TAG, "cmp3 set");
    return ESP_OK;
}

esp_err_t leg_set_angle_rad(leg_handle_t h, leg_servo_t joint, float radians)
{
    leg_ctx_t* leg = (leg_ctx_t*)h;
    if (!leg) return ESP_ERR_INVALID_STATE;
    // Clamp per joint
    switch (joint) {
    case LEG_SERVO_COXA:  radians = clampf(radians, leg->min_rad_coxa,  leg->max_rad_coxa);  break;
    case LEG_SERVO_FEMUR: radians = clampf(radians, leg->min_rad_femur, leg->max_rad_femur); break;
    case LEG_SERVO_TIBIA: radians = clampf(radians, leg->min_rad_tibia, leg->max_rad_tibia); break;
        default: return ESP_ERR_INVALID_ARG;
    }
    uint32_t cmp = angle_to_compare_rad(radians);
    switch (joint) {
        case LEG_SERVO_COXA:
            return mcpwm_comparator_set_compare_value(leg->cmpr1, cmp);
        case LEG_SERVO_FEMUR:
            return mcpwm_comparator_set_compare_value(leg->cmpr2, cmp);
        case LEG_SERVO_TIBIA:
            return mcpwm_comparator_set_compare_value(leg->cmpr3, cmp);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t leg_move_xyz(leg_handle_t handle, float x, float y, float z)
{
    leg_ctx_t* leg = (leg_ctx_t*)handle;
    if (!leg) return ESP_ERR_INVALID_STATE;

    // Coxa rotation (yaw) in XY plane.
    float yaw = normalize_angle(atan2f(y, x)); // range [-pi, pi]
    float r_xy = sqrtf(x*x + y*y);
    ESP_LOGI(TAG, "IK: r_xy=%.1f, yaw=%.3f deg", r_xy, yaw * (180.0f / M_PI));
    yaw += leg->coxa_offset_rad;
    yaw = clampf(yaw, SERVO_MIN_RAD, SERVO_MAX_RAD);

    float px = r_xy - leg->len_coxa;
    float pz = z;

    ESP_LOGI(TAG, "IK: px=%.1f pz=%.1f", px, pz);

    // calculate cos alpha from law of cosinus
    float L1 = leg->len_femur;
    float L2 = leg->len_tibia;
    float d = clampf(hypotf(px, pz), fabsf(L1 - L2), L1 + L2);

    float cosK = (L2*L2 + L1*L1 - d*d) / (2*L2*L1);
    cosK = clampf(cosK, -1.0f, 1.0f);
    float ankle_angle = acosf(cosK);
    ESP_LOGI(TAG, "IK: d=%.1f, cosK=%.3f, ankle=%.3f deg", d, cosK, ankle_angle * (180.0f / M_PI));
    float ankle = -(ankle_angle - leg->tibia_offset_rad);
    ESP_LOGI(TAG, "IK: d=%.1f, ankle=%.3f deg", d, ankle * (180.0f / M_PI));

    float cosPhi = (L1*L1 + d*d - L2*L2) / (2*L1*d);
    cosPhi = clampf(cosPhi, -1.0f, 1.0f);
    float phi = acosf(cosPhi);
    float alpha = M_PI / 2 - atan2f(d, pz);
    float knee = phi - leg->femur_offset_rad - alpha;
    ESP_LOGI(TAG, "IK: d=%.1f, phi=%.3f deg, knee=%.3f deg, alpha=%.3f deg", d, phi * (180.0f / M_PI), knee * (180.0f / M_PI), alpha * (180.0f / M_PI));


    // Map to servo joints with per-joint clamping
    yaw = clampf(yaw, leg->min_rad_coxa, leg->max_rad_coxa);
    knee = clampf(knee, leg->min_rad_femur, leg->max_rad_femur);
    ankle = clampf(ankle, leg->min_rad_tibia, leg->max_rad_tibia);

    // Command servos
    esp_err_t err = ESP_OK;
    if ((err = leg_set_angle_rad(leg, LEG_SERVO_COXA, yaw)) != ESP_OK) return err;
    if ((err = leg_set_angle_rad(leg, LEG_SERVO_FEMUR, knee)) != ESP_OK) return err;
    if ((err = leg_set_angle_rad(leg, LEG_SERVO_TIBIA, ankle)) != ESP_OK) return err;
    return ESP_OK;
}
