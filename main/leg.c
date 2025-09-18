/*
 * Pure IK implementation: compute joint angles from foot position.
 */

#include "leg.h"
#include <stdlib.h>
#include <math.h>
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "leg";

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

typedef struct leg_s {
    // Geometry only; no hardware details
    float len_coxa, len_femur, len_tibia;
    float coxa_offset_rad, femur_offset_rad, tibia_offset_rad;
} leg_ctx_t;

esp_err_t leg_configure(const leg_config_t* cfg, leg_handle_t* out_leg)
{
    if (!cfg || !out_leg) return ESP_ERR_INVALID_ARG;
    *out_leg = NULL;
    leg_ctx_t* leg = (leg_ctx_t*)calloc(1, sizeof(leg_ctx_t));
    if (!leg) return ESP_ERR_NO_MEM;
    leg->len_coxa = cfg->len_coxa;
    leg->len_femur = cfg->len_femur;
    leg->len_tibia = cfg->len_tibia;
    leg->coxa_offset_rad = cfg->coxa_offset_rad;
    leg->femur_offset_rad = cfg->femur_offset_rad;
    leg->tibia_offset_rad = cfg->tibia_offset_rad;
    *out_leg = (leg_handle_t)leg;
    return ESP_OK;
}

esp_err_t leg_ik_solve(leg_handle_t handle, float x, float y, float z, leg_angles_t *out_angles)
{
    if (!handle || !out_angles) return ESP_ERR_INVALID_ARG;
    leg_ctx_t* leg = (leg_ctx_t*)handle;

    // Coxa rotation (yaw) in XY plane.
    float yaw = normalize_angle(atan2f(y, x)); // range [-pi, pi]
    float r_xy = sqrtf(x*x + y*y);
    yaw += leg->coxa_offset_rad;
    ESP_LOGD(TAG, "IK: r_xy=%.3f, yaw=%.3f deg offset=%.3f deg", r_xy, yaw * (180.0f / M_PI), leg->coxa_offset_rad * (180.0f / M_PI));

    // Reduce to 2D (sagittal) by subtracting coxa length along X
    float px = r_xy - leg->len_coxa;
    float pz = z; // +Z down
    ESP_LOGD(TAG, "IK: px=%.3f pz=%.3f", px, pz);

    // Law of cosines to find knee (tibia) angle
    float L1 = leg->len_femur;
    float L2 = leg->len_tibia;
    float d = clampf(hypotf(px, pz), fabsf(L1 - L2), L1 + L2);
    ESP_LOGD(TAG, "IK: d=%.3f, L1=%.3f, L2=%.3f", d, L1, L2);

    float cosK = (L2*L2 + L1*L1 - d*d) / (2.0f*L2*L1);
    cosK = clampf(cosK, -1.0f, 1.0f);
    float tibia = -(acosf(cosK) - leg->tibia_offset_rad); // negative bends downwards under our +Z down convention

    ESP_LOGD(TAG, "IK: cosK=%.3f tibia=%.3f deg", cosK, tibia * (180.0f / M_PI));

    // Hip pitch (femur): triangle geometry
    float cosPhi = (L1*L1 + d*d - L2*L2) / (2.0f*L1*d);
    cosPhi = clampf(cosPhi, -1.0f, 1.0f);
    float phi = acosf(cosPhi);
    float alpha = M_PI * 0.5f - atan2f(d, pz); // swing plane alignment
    float femur = phi - leg->femur_offset_rad - alpha;
    ESP_LOGD(TAG, "IK: cosPhi=%.3f phi=%.3f deg alpha=%.3f deg femur=%.3f deg", cosPhi, phi * (180.0f / M_PI), alpha * (180.0f / M_PI), femur * (180.0f / M_PI));

    out_angles->coxa = yaw;
    out_angles->femur = femur;
    out_angles->tibia = tibia;
    return ESP_OK;
}

// old working algorithm
// esp_err_t leg_move_xyz(leg_handle_t handle, float x, float y, float z)
// {
//     leg_ctx_t* leg = (leg_ctx_t*)handle;
//     if (!leg) return ESP_ERR_INVALID_STATE;

//     // Coxa rotation (yaw) in XY plane.
//     float yaw = normalize_angle(atan2f(y, x)); // range [-pi, pi]
//     float r_xy = sqrtf(x*x + y*y);
//     ESP_LOGD(TAG, "IK: r_xy=%.1f, yaw=%.3f deg", r_xy, yaw * (180.0f / M_PI));
//     yaw += leg->coxa_offset_rad;
//     yaw = clampf(yaw, SERVO_MIN_RAD, SERVO_MAX_RAD);

//     float px = r_xy - leg->len_coxa;
//     float pz = z;

//     ESP_LOGD(TAG, "IK: px=%.1f pz=%.1f", px, pz);

//     // calculate cos alpha from law of cosinus
//     float L1 = leg->len_femur;
//     float L2 = leg->len_tibia;
//     float d = clampf(hypotf(px, pz), fabsf(L1 - L2), L1 + L2);

//     float cosK = (L2*L2 + L1*L1 - d*d) / (2*L2*L1);
//     cosK = clampf(cosK, -1.0f, 1.0f);
//     float ankle_angle = acosf(cosK);
//     ESP_LOGD(TAG, "IK: d=%.1f, cosK=%.3f, ankle=%.3f deg", d, cosK, ankle_angle * (180.0f / M_PI));
//     float ankle = -(ankle_angle - leg->tibia_offset_rad);
//     ESP_LOGD(TAG, "IK: d=%.1f, ankle=%.3f deg", d, ankle * (180.0f / M_PI));

//     float cosPhi = (L1*L1 + d*d - L2*L2) / (2*L1*d);
//     cosPhi = clampf(cosPhi, -1.0f, 1.0f);
//     float phi = acosf(cosPhi);
//     float alpha = M_PI / 2 - atan2f(d, pz);
//     float knee = phi - leg->femur_offset_rad - alpha;
//     ESP_LOGD(TAG, "IK: d=%.1f, phi=%.3f deg, knee=%.3f deg, alpha=%.3f deg", d, phi * (180.0f / M_PI), knee * (180.0f / M_PI), alpha * (180.0f / M_PI));


//     // Map to servo joints with per-joint clamping
//     yaw = clampf(yaw, leg->min_rad_coxa, leg->max_rad_coxa);
//     knee = clampf(knee, leg->min_rad_femur, leg->max_rad_femur);
//     ankle = clampf(ankle, leg->min_rad_tibia, leg->max_rad_tibia);

//     // Command servos
//     esp_err_t err = ESP_OK;
//     if ((err = leg_set_angle_rad(leg, LEG_SERVO_COXA, yaw)) != ESP_OK) return err;
//     if ((err = leg_set_angle_rad(leg, LEG_SERVO_FEMUR, knee)) != ESP_OK) return err;
//     if ((err = leg_set_angle_rad(leg, LEG_SERVO_TIBIA, ankle)) != ESP_OK) return err;
//     return ESP_OK;
// }

