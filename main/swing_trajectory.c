#include "swing_trajectory.h"
#include "gait_scheduler.h"
#include <math.h>
#include <stdbool.h>

void swing_trajectory_init(swing_trajectory_t *trajectory, float step_length, float clearance_height) {
    trajectory->step_length = step_length;
    trajectory->clearance_height = clearance_height;
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
    if (!trajectory) {
        return;
    }
    // Normalize vx to [-1,1] (controller provides -1..1, but clamp for safety)
    float vx_n = 0.0f;
    if (cmd) {
        vx_n = clampf(cmd->vx, -1.0f, 1.0f);
    }
    // scale step length by speed magnitude and user step_scale; zero when disabled
    float speed_mag = fabsf(vx_n);
    float scale = (cmd ? clampf(cmd->step_scale, 0.0f, 1.0f) : 0.5f);
    if (!(cmd && cmd->enable)) {
        speed_mag = 0.0f;
    }
    float L = trajectory->step_length * scale * speed_mag; // effective step length
    float clr = trajectory->clearance_height * (cmd && cmd->terrain_climb ? 1.5f : 1.0f);
    float body_z = cmd ? cmd->z_target : 0.0f; // map to meters elsewhere
    float body_y = cmd ? cmd->y_offset : 0.0f;

    // Determine swing fraction S per gait
    float S = 0.5f; // default tripod
    if (cmd) {
        if (cmd->gait == GAIT_RIPPLE) {
            S = 1.0f / (float)NUM_LEGS; // ~0.1667
        } else if (cmd->gait == GAIT_WAVE) {
            S = 0.6f / (float)NUM_LEGS; // ~0.1, matches scheduler placeholder
        } else {
            S = 0.5f;
        }
    }

    float phase = scheduler ? scheduler->phase : 0.0f;

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_position_t *p = &trajectory->desired_positions[i];
        float p_i;
        if (cmd && cmd->gait == GAIT_TRIPOD) {
            bool inA = (i == 0 || i == 3 || i == 4);
            float offset = inA ? 0.0f : 0.5f;
            p_i = fracf(phase + offset);
        } else {
            // RIPPLE/WAVE: evenly staggered
            p_i = fracf(phase + (i / (float)NUM_LEGS));
        }

        bool swing = (p_i < S) ? true : false;
        float tau = swing ? (p_i / S) : ((S < 1.0f) ? ((p_i - S) / (1.0f - S)) : 0.0f);
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

        p->x = x_rel;
        p->y = body_y;
        p->z = body_z + z_rel; // body_z baseline; negative z_rel lifts foot if +down convention upstream
    }
}
