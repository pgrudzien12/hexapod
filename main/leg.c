#include "leg.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_check.h"

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

    ESP_LOGI(TAG, "Configuring leg servos on GPIOs %d, %d, %d", leg->gpio1, leg->gpio2, leg->gpio3);

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

    // Set neutral compare for all (0 radians)
    uint32_t neutral = angle_to_compare_rad(0.0f);
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr1, neutral)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr2, neutral)) != ESP_OK) { free(leg); return err; }
    if ((err = mcpwm_comparator_set_compare_value(leg->cmpr3, neutral)) != ESP_OK) { free(leg); return err; }

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
    uint32_t neutral = angle_to_compare_rad(0.0f);
    ESP_LOGI(TAG, "Setting all three servos to neutral (%u us)", (unsigned)neutral);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr1, neutral), TAG, "cmp1 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr2, neutral), TAG, "cmp2 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(leg->cmpr3, neutral), TAG, "cmp3 set");
    return ESP_OK;
}

esp_err_t leg_set_angle_rad(leg_handle_t h, leg_servo_t joint, float radians)
{
    leg_ctx_t* leg = (leg_ctx_t*)h;
    if (!leg) return ESP_ERR_INVALID_STATE;
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
