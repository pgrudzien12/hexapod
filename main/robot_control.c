#include "robot_control.h"
#include "esp_log.h"
#include "esp_check.h"
#include "robot_config.h"
#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG_RC = "robot_control";

// We will mix MCPWM (legs 0,1,4,5) and LEDC (legs 2,3) to reach 18 servos on classic ESP32.
// LEDC provides 16 channels total; we only need 6 of them here. We allocate a single LEDC timer
// configured for 50 Hz (20 ms) with 15-bit resolution giving ~0.61 us per step.
// Coexistence notes:
// - MCPWM and LEDC are independent peripherals; using both simultaneously is supported.
// - Each has its own hardware timer resources; there is no contention besides GPIO matrix routing.
// - Ensure chosen GPIOs are not reused elsewhere. Calibration and angle->pulse mapping is shared.
// - MCPWM yields exact microsecond compare logic; LEDC duty resolution here (~0.61us) is adequate for hobby servos.
// - Total capacity now: MCPWM up to 12 (2 groups * 3 operators * 2 generators) + LEDC 6 (channels 0..5) = 18.

typedef enum {
    SERVO_DRV_NONE = 0,
    SERVO_DRV_MCPWM,
    SERVO_DRV_LEDC,
} servo_driver_t;

#define SERVO_LEDC_TIMER          LEDC_TIMER_0
#define SERVO_LEDC_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_RES_BITS       LEDC_TIMER_15_BIT
#define SERVO_LEDC_FREQUENCY_HZ   50

// Reserve a contiguous block of LEDC channels for legs 2 & 3 (6 joints: coxa,femur,tibia x2)
#define SERVO_LEDC_BASE_CHANNEL   LEDC_CHANNEL_0

static bool s_ledc_timer_inited = false;

// Simple MCPWM context per servo channel (leg,joint)
typedef struct {
    bool inited;
    bool disabled; // set true if resource allocation fails; skip updates
    int group_id;
    int gpio;
    servo_driver_t driver; // which backend drives this channel
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t gen;
    ledc_channel_t ledc_chan; // valid if driver == SERVO_DRV_LEDC
} pwm_chan_t;

static pwm_chan_t g_pwm[NUM_LEGS][3];

// Decide whether a given (leg,joint) should use LEDC based on configuration.
static inline bool use_ledc_for_joint(int leg_index, leg_servo_t joint) {
    return robot_config_get_servo_driver(leg_index, joint) == 1; // 1 -> LEDC per config
}

// Map (leg,joint) to a LEDC channel index starting at SERVO_LEDC_BASE_CHANNEL.
// Leg 2 joints 0..2 -> channels 0..2, Leg 3 joints 0..2 -> channels 3..5
// Build a compact channel index for all configured LEDC joints (lazy). Max expected 6 here.
static int s_ledc_channel_lut[NUM_LEGS][3]; // -1 if not LEDC, else compact index
static int s_ledc_lut_built = 0;
static void build_ledc_lut(void) {
    if (s_ledc_lut_built) return;
    for (int i=0;i<NUM_LEGS;++i) for (int j=0;j<3;++j) s_ledc_channel_lut[i][j] = -1;
    int next = 0;
    for (int i=0;i<NUM_LEGS;++i) {
        for (int j=0;j<3;++j) {
            if (robot_config_get_servo_driver(i,(leg_servo_t)j)==1) {
                s_ledc_channel_lut[i][j] = next++;
            }
        }
    }
    s_ledc_lut_built = 1;
}
static inline ledc_channel_t map_to_ledc_channel(int leg_index, leg_servo_t joint) {
    build_ledc_lut();
    int idx = s_ledc_channel_lut[leg_index][(int)joint];
    if (idx < 0) idx = 0; // fallback (should not happen if caller checked)
    return (ledc_channel_t)(SERVO_LEDC_BASE_CHANNEL + idx);
}

static esp_err_t ensure_ledc_timer_inited(void) {
    if (s_ledc_timer_inited) return ESP_OK;
    ledc_timer_config_t tcfg = {
        .speed_mode       = SERVO_LEDC_SPEED_MODE,
        .duty_resolution  = SERVO_LEDC_RES_BITS,
        .timer_num        = SERVO_LEDC_TIMER,
        .freq_hz          = SERVO_LEDC_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err == ESP_OK) {
        s_ledc_timer_inited = true;
        ESP_LOGI(TAG_RC, "Initialized shared LEDC timer for servo legs 2&3 (freq=%dHz, res=%d bits)", SERVO_LEDC_FREQUENCY_HZ, SERVO_LEDC_RES_BITS);
    } else {
        ESP_LOGE(TAG_RC, "Failed to init LEDC timer (%s)", esp_err_to_name(err));
    }
    return err;
}

// One shared timer per MCPWM group to avoid exhausting group timers
#define MAX_MCPWM_GROUPS 2 /* adjust if your target has more groups */
typedef struct {
    bool inited;
    mcpwm_timer_handle_t timer;
} group_timer_t;
static group_timer_t s_group_timers[MAX_MCPWM_GROUPS];

// Pool a small number of operators per group, each with up to two generators (A/B)
#define MAX_OPERATORS_PER_GROUP 3 // ESP32 classic has up to 3 operators per group
typedef struct {
    bool inited;
    mcpwm_oper_handle_t oper;
    int used_generators; // 0..2
} group_operator_t;
static group_operator_t s_group_op_pool[MAX_MCPWM_GROUPS][MAX_OPERATORS_PER_GROUP];

// Acquire an operator in the given group that still has a free generator slot
static esp_err_t acquire_operator_with_free_gen(int group, mcpwm_oper_handle_t *out_oper) {
    if (group < 0 || group >= MAX_MCPWM_GROUPS) return ESP_ERR_INVALID_ARG;
    // Try reuse first
    for (int i = 0; i < MAX_OPERATORS_PER_GROUP; ++i) {
        if (s_group_op_pool[group][i].inited && s_group_op_pool[group][i].used_generators < 2) {
            *out_oper = s_group_op_pool[group][i].oper;
            s_group_op_pool[group][i].used_generators++;
            ESP_LOGI(TAG_RC, "Reusing MCPWM operator %d in group %d (generators now %d)", i, group, s_group_op_pool[group][i].used_generators);
            return ESP_OK;
        }
    }
    // Create a new operator if capacity allows
    for (int i = 0; i < MAX_OPERATORS_PER_GROUP; ++i) {
        if (!s_group_op_pool[group][i].inited) {
            mcpwm_operator_config_t ocfg = { .group_id = group };
            ESP_RETURN_ON_ERROR(mcpwm_new_operator(&ocfg, &s_group_op_pool[group][i].oper), TAG_RC, "oper");
            ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_group_op_pool[group][i].oper, s_group_timers[group].timer), TAG_RC, "connect");
            s_group_op_pool[group][i].inited = true;
            s_group_op_pool[group][i].used_generators = 1; // we'll assign one now
            *out_oper = s_group_op_pool[group][i].oper;
            ESP_LOGI(TAG_RC, "Initialized MCPWM operator %d for group %d (generators now %d)", i, group, s_group_op_pool[group][i].used_generators);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG_RC, "No free MCPWM operators/generators left in group %d (ops=%d, gens used: %d,%d)",
             group, MAX_OPERATORS_PER_GROUP,
             s_group_op_pool[group][0].used_generators,
             s_group_op_pool[group][1].used_generators);
    return ESP_ERR_NOT_FOUND;
}

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
    if (ch->disabled) {
        return ESP_OK; // silently ignore if this channel previously failed to init
    }
    if (!ch->inited) {
        if (gpio < 0) {
            // ESP_LOGW(TAG_RC, "leg %d joint %d has no GPIO assigned; skipping output", leg_index, (int)joint);
            return ESP_OK;
        }
        ch->group_id = group;
        ch->gpio = gpio;
        if (use_ledc_for_joint(leg_index, joint)) {
            // LEDC path
            if (ensure_ledc_timer_inited() != ESP_OK) {
                ch->disabled = true;
                return ESP_OK;
            }
            ledc_channel_t chan = map_to_ledc_channel(leg_index, joint);
            ch->ledc_chan = chan;
            ledc_channel_config_t ccfg = {
                .gpio_num       = gpio,
                .speed_mode     = SERVO_LEDC_SPEED_MODE,
                .channel        = chan,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = SERVO_LEDC_TIMER,
                .duty           = 0,
                .hpoint         = 0,
                .flags.output_invert = 0,
            };
            esp_err_t lres = ledc_channel_config(&ccfg);
            if (lres != ESP_OK) {
                ESP_LOGE(TAG_RC, "Failed to init LEDC channel for leg %d joint %d (%s)", leg_index, (int)joint, esp_err_to_name(lres));
                ch->disabled = true;
                return ESP_OK;
            }
            ch->driver = SERVO_DRV_LEDC;
            ch->inited = true;
            ESP_LOGI(TAG_RC, "Initialized LEDC for leg %d joint %d on GPIO %d (chan=%d)", leg_index, (int)joint, gpio, (int)chan);
        } else {
            // MCPWM path
            if (group < 0 || group >= MAX_MCPWM_GROUPS) {
                ESP_LOGE(TAG_RC, "Invalid MCPWM group %d", group);
                return ESP_FAIL;
            }
            if (!s_group_timers[group].inited) {
                mcpwm_timer_config_t tcfg = {
                    .group_id = group,
                    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                    .period_ticks = SERVO_TIMEBASE_PERIOD,
                    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                };
                ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tcfg, &s_group_timers[group].timer), TAG_RC, "timer");
                ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_group_timers[group].timer), TAG_RC, "timer-enable");
                ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_group_timers[group].timer, MCPWM_TIMER_START_NO_STOP), TAG_RC, "timer-start");
                s_group_timers[group].inited = true;
                ESP_LOGI(TAG_RC, "Initialized shared MCPWM timer for group %d", group);
            } else {
                ESP_LOGI(TAG_RC, "Reusing shared MCPWM timer for group %d", group);
            }
            esp_err_t op_res = acquire_operator_with_free_gen(group, &ch->oper);
            if (op_res != ESP_OK) {
                ESP_LOGE(TAG_RC, "leg %d joint %d: MCPWM operator capacity exhausted in group %d; disabling channel",
                         leg_index, (int)joint, group);
                ch->disabled = true;
                return ESP_OK;
            }
            mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
            ESP_RETURN_ON_ERROR(mcpwm_new_comparator(ch->oper, &ccfg, &ch->cmpr), TAG_RC, "cmpr");
            mcpwm_generator_config_t gcfg = { .gen_gpio_num = gpio };
            ESP_RETURN_ON_ERROR(mcpwm_new_generator(ch->oper, &gcfg, &ch->gen), TAG_RC, "gen");
            ESP_LOGI(TAG_RC, "Created comparator+generator for leg %d joint %d on GPIO %d", leg_index, (int)joint, gpio);
            ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(
                ch->gen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
            ), TAG_RC, "gen-timer-act");
            ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(
                ch->gen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, ch->cmpr, MCPWM_GEN_ACTION_LOW)
            ), TAG_RC, "gen-cmp-act");
            ch->driver = SERVO_DRV_MCPWM;
            ch->inited = true;
            ESP_LOGI(TAG_RC, "Initialized MCPWM for leg %d joint %d on GPIO %d (group %d)", leg_index, (int)joint, gpio, group);
        }
    }

    // Apply compare based on desired pwm_us
    if (!ch->disabled) {
        if (ch->driver == SERVO_DRV_MCPWM && ch->cmpr) {
            ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(ch->cmpr, (uint32_t)pwm_us), TAG_RC, "set cmp");
        } else if (ch->driver == SERVO_DRV_LEDC) {
            // Convert microseconds to duty counts
            // duty = pwm_us / 20000 * (1<<SERVO_LEDC_RES_BITS)
            uint32_t max_count = (1u << SERVO_LEDC_RES_BITS);
            uint32_t duty = (uint32_t)(((uint64_t)pwm_us * max_count) / SERVO_TIMEBASE_PERIOD);
            if (duty >= max_count) duty = max_count - 1;
            esp_err_t dres = ledc_set_duty(SERVO_LEDC_SPEED_MODE, ch->ledc_chan, duty);
            if (dres == ESP_OK) {
                dres = ledc_update_duty(SERVO_LEDC_SPEED_MODE, ch->ledc_chan);
            }
            if (dres != ESP_OK) {
                ESP_LOGE(TAG_RC, "LEDC duty update failed leg %d joint %d (%s)", leg_index, (int)joint, esp_err_to_name(dres));
            }
        }
    }
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
        // break;
    }
}
