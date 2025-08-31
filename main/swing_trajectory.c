#include "swing_trajectory.h"
#include "gait_scheduler.h"
#include <math.h>
#include <stdbool.h>
#include <assert.h>
#include "robot_config.h"

void swing_trajectory_init(swing_trajectory_t *trajectory, float step_length, float clearance_height) {
    trajectory->step_length = step_length;
    trajectory->clearance_height = clearance_height;
    // TODO: Make these configurable via Kconfig or runtime config
    trajectory->y_range_m = 0.05f; // +/-5 cm lateral
    trajectory->z_min_m = 0.05f;   // 5 cm min body height (avoid hitting battery)
    trajectory->z_max_m = 0.15f;   // 15 cm max body height
    for (int i = 0; i < NUM_LEGS; ++i) {
        trajectory->desired_positions[i].x = 0.0f;
        trajectory->desired_positions[i].y = 0.0f;
        trajectory->desired_positions[i].z = 0.0f;
    }
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float fracf(float x) {
    float f = x - floorf(x);
    return (f < 0.0f) ? (f + 1.0f) : f;
}

void swing_trajectory_generate(swing_trajectory_t *trajectory, const gait_scheduler_t *scheduler, const user_command_t *cmd) {
    assert(trajectory != NULL);
    assert(scheduler != NULL);
    assert(cmd != NULL);
    // Normalize vx to [-1,1] (controller provides -1..1, but clamp for safety)
    float vx_n = clampf(cmd->vx, -1.0f, 1.0f);
    // scale step length by speed magnitude and user step_scale; zero when disabled
    float speed_mag = fabsf(vx_n);
    float scale = clampf(cmd->step_scale, 0.0f, 1.0f);
    bool enabled = cmd->enable;
    if (!enabled) speed_mag = 0.0f;
    float L = trajectory->step_length * scale * speed_mag; // effective step length (meters)
    float clr = trajectory->clearance_height * (cmd->terrain_climb ? 1.5f : 1.0f);
    // Map normalized pose to meters
    // New: map z_target in [-1,1] to [z_min_m, z_max_m] to enforce safe body height
    // TODO: Confirm axis convention (+Z down vs up) and adjust sign if needed
    float z_n = clampf(cmd->z_target, -1.0f, 1.0f);
    float body_z = trajectory->z_min_m + (0.5f * (z_n + 1.0f)) * (trajectory->z_max_m - trajectory->z_min_m);
    float body_y = clampf(cmd->y_offset, -1.0f, 1.0f) * trajectory->y_range_m; // meters

    // Determine swing fraction S per gait (0 < S < 1)
    float S;
    switch (cmd->gait) {
        case GAIT_RIPPLE: S = 1.0f / 3.0f; break;                   // two legs swing per 3 windows
        case GAIT_WAVE:   S = 0.6f / (float)NUM_LEGS; break;         // ~0.1
        case GAIT_TRIPOD: default: S = 0.5f; break;                  // balanced
    }
    assert(S > 0.0f && S < 1.0f);

    float phase = scheduler->phase;

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_position_t *p = &trajectory->desired_positions[i];
        float p_i; // local phase of the individual leg
        if (cmd->gait == GAIT_TRIPOD) {
            bool inA = (i == 0 || i == 3 || i == 4);
            float offset = inA ? 0.0f : 0.5f;
            p_i = fracf(phase + offset);
        } else if (cmd->gait == GAIT_RIPPLE) {
            // RIPPLE: group by i%3 into three windows
            float offset = (i % 3) / 3.0f; // 0, 1/3, 2/3
            p_i = fracf(phase - offset);
        } else {
            // WAVE: evenly staggered by leg index
            p_i = fracf(phase + (i / (float)NUM_LEGS));
        }

        // Only swing (lift foot) when there's a non-zero commanded step length
        // This avoids up/down motion at zero speed.
        bool swing = enabled && (L > 1e-6f) && (p_i < S);
        // tau progresses 0..1 within the active subphase (swing or support)
        float tau = swing ? (p_i / S) : ((p_i - S) / (1.0f - S));
        tau = clampf(tau, 0.0f, 1.0f);

        // Cycloid-like arc for swing; flat for support
        float x_rel;
        float z_rel;
        if (swing) {
            // forward from -L/2 to +L/2 with vertical arc
            x_rel = (-0.5f + tau) * L * (vx_n >= 0.0f ? 1.0f : -1.0f);
            // smooth arch peaking at mid; sin(pi*tau) shape
            z_rel = -clr * sinf((float)M_PI * tau);
        } else {
            // support: backward from +L/2 to -L/2 at ground
            x_rel = (0.5f - tau) * L * (vx_n >= 0.0f ? 1.0f : -1.0f);
            z_rel = 0.0f;
        }

    // Convert offsets to absolute body-frame targets by adding each leg's base pose
    float bx, by, bz, yaw;
    robot_config_get_base_pose(i, &bx, &by, &bz, &yaw);
    // Apply neutral stance reach in leg-local frame (X_outward, Y_forward), rotated to body frame
    float out_m = robot_config_get_stance_out_m(i);
    float fwd_m = robot_config_get_stance_fwd_m(i);
    float cy = cosf(yaw), sy = sinf(yaw);
    float dx_stance = cy * out_m - sy * fwd_m;
    float dy_stance = sy * out_m + cy * fwd_m;

    p->x = bx + dx_stance + x_rel;
    p->y = by + dy_stance + body_y;
    p->z = bz + body_z + z_rel; // include base z in case it's non-zero
        // TODO: Add simple body roll/pitch offsets when pose_mode is active
    }
}
