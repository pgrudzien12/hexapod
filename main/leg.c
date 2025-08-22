#include "leg.h"

#include <stdbool.h>
#include <stdint.h>
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

typedef struct {
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
    bool initialized;
} leg_ctx_t;

static leg_ctx_t s_leg = {0};

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

esp_err_t leg_configure(int servo1_gpio, int servo2_gpio, int servo3_gpio)
{
    if (s_leg.initialized) {
        return ESP_OK; // already configured
    }

    ESP_LOGI(TAG, "Configuring leg servos on GPIOs %d, %d, %d", servo1_gpio, servo2_gpio, servo3_gpio);

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &s_leg.timer), TAG, "timer");

    // Create three operators in the same group and connect them to the same timer
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &s_leg.oper1), TAG, "oper1");
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &s_leg.oper2), TAG, "oper2");
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &s_leg.oper3), TAG, "oper3");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_leg.oper1, s_leg.timer), TAG, "connect1");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_leg.oper2, s_leg.timer), TAG, "connect2");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_leg.oper3, s_leg.timer), TAG, "connect3");

    // Create three comparators (one per servo)
    mcpwm_comparator_config_t c_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_leg.oper1, &c_cfg, &s_leg.cmpr1), TAG, "cmpr1");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_leg.oper2, &c_cfg, &s_leg.cmpr2), TAG, "cmpr2");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_leg.oper3, &c_cfg, &s_leg.cmpr3), TAG, "cmpr3");

    // Create three generators bound to different GPIOs
    mcpwm_generator_config_t g1 = {.gen_gpio_num = servo1_gpio};
    mcpwm_generator_config_t g2 = {.gen_gpio_num = servo2_gpio};
    mcpwm_generator_config_t g3 = {.gen_gpio_num = servo3_gpio};
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_leg.oper1, &g1, &s_leg.gen1), TAG, "gen1");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_leg.oper2, &g2, &s_leg.gen2), TAG, "gen2");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_leg.oper3, &g3, &s_leg.gen3), TAG, "gen3");

    // Set neutral compare for all (0 radians)
    uint32_t neutral = angle_to_compare_rad(0.0f);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr1, neutral), TAG, "cmp1 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr2, neutral), TAG, "cmp2 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr3, neutral), TAG, "cmp3 set");

    // Actions: go HIGH at timer empty, LOW at comparator
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(s_leg.gen1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)), TAG, "gen1 tmr act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(s_leg.gen2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)), TAG, "gen2 tmr act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(s_leg.gen3,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)), TAG, "gen3 tmr act");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(s_leg.gen1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_leg.cmpr1, MCPWM_GEN_ACTION_LOW)), TAG, "gen1 cmp act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(s_leg.gen2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_leg.cmpr2, MCPWM_GEN_ACTION_LOW)), TAG, "gen2 cmp act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(s_leg.gen3,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_leg.cmpr3, MCPWM_GEN_ACTION_LOW)), TAG, "gen3 cmp act");

    // Start timer
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_leg.timer), TAG, "timer en");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_leg.timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer start");

    s_leg.gpio1 = servo1_gpio;
    s_leg.gpio2 = servo2_gpio;
    s_leg.gpio3 = servo3_gpio;
    s_leg.initialized = true;
    return ESP_OK;
}

esp_err_t leg_test_neutral(void)
{
    if (!s_leg.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t neutral = angle_to_compare_rad(0.0f);
    ESP_LOGI(TAG, "Setting all three servos to neutral (%u us)", (unsigned)neutral);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr1, neutral), TAG, "cmp1 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr2, neutral), TAG, "cmp2 set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_leg.cmpr3, neutral), TAG, "cmp3 set");
    return ESP_OK;
}

esp_err_t leg_set_angle_rad(leg_servo_t joint, float radians)
{
    if (!s_leg.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t cmp = angle_to_compare_rad(radians);
    switch (joint) {
        case LEG_SERVO_COXA:
            return mcpwm_comparator_set_compare_value(s_leg.cmpr1, cmp);
        case LEG_SERVO_FEMUR:
            return mcpwm_comparator_set_compare_value(s_leg.cmpr2, cmp);
        case LEG_SERVO_TIBIA:
            return mcpwm_comparator_set_compare_value(s_leg.cmpr3, cmp);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}
