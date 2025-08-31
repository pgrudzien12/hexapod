/*
 * Pure IK implementation: compute joint angles from foot position.
 */

#include "leg.h"
#include <stdlib.h>
#include <math.h>
#include "esp_err.h"

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

    // Reduce to 2D (sagittal) by subtracting coxa length along X
    float px = r_xy - leg->len_coxa;
    float pz = z; // +Z down

    // Law of cosines to find knee (tibia) angle
    float L1 = leg->len_femur;
    float L2 = leg->len_tibia;
    float d = clampf(hypotf(px, pz), fabsf(L1 - L2), L1 + L2);

    float cosK = (L2*L2 + L1*L1 - d*d) / (2.0f*L2*L1);
    cosK = clampf(cosK, -1.0f, 1.0f);
    float tibia = -acosf(cosK); // negative bends downwards under our +Z down convention

    // Hip pitch (femur): triangle geometry
    float cosPhi = (L1*L1 + d*d - L2*L2) / (2.0f*L1*d);
    cosPhi = clampf(cosPhi, -1.0f, 1.0f);
    float phi = acosf(cosPhi);
    float alpha = M_PI * 0.5f - atan2f(d, pz); // swing plane alignment
    float femur = phi - alpha;

    out_angles->coxa = yaw;
    out_angles->femur = femur;
    out_angles->tibia = tibia;
    return ESP_OK;
}

