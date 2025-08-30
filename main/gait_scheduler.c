#include "gait_scheduler.h"

void gait_scheduler_init(gait_scheduler_t *scheduler, float cycle_time) {
    // Initialize scheduler parameters
    scheduler->cycle_time = cycle_time;
    scheduler->phase = 0.0f;
    for (int i = 0; i < NUM_LEGS; ++i) {
        scheduler->leg_states[i] = LEG_SUPPORT;
    }
}

void gait_scheduler_update(gait_scheduler_t *scheduler, float dt, const user_command_t *cmd) {
    // Advance phase only when enabled and commanded velocity is non-zero
    // TODO: Consider separate forward/turning components and modulate phase rate by
    //       a base frequency parameter instead of reusing cycle_time directly.
    if (cmd && cmd->enable) {
        // crude frequency scaling: base 1/cycle_time Hz, scaled by step_scale and |vx|
        float speed = (cmd->vx < 0.0f) ? -cmd->vx : cmd->vx; // use magnitude
        float freq = (scheduler->cycle_time > 0.0f) ? (1.0f / scheduler->cycle_time) : 1.0f;
        float scale = (cmd->step_scale > 0.0f) ? cmd->step_scale : 0.5f;
        float phase_rate = freq * (0.5f + 0.5f * (speed > 1.0f ? 1.0f : speed)) * scale; // 0.5..1x based on speed
        scheduler->phase += phase_rate * dt;
        if (scheduler->phase >= 1.0f) {
            scheduler->phase -= 1.0f;
        }
    } else {
        // hold phase and keep all legs in support when disabled
        scheduler->phase = scheduler->phase;
        for (int i = 0; i < NUM_LEGS; ++i) {
            scheduler->leg_states[i] = LEG_SUPPORT;
        }
        return;
    }

    // Set leg states based on gait type and current phase (placeholder patterns)
    // TODO: Replace with well-defined groupings and exact phase windows per gait.
    switch (cmd ? cmd->gait : GAIT_TRIPOD) {
        case GAIT_TRIPOD: {
            // tripod groups: {0,3,4} swing when phase < 0.5, others support; then swap
            bool groupA = (scheduler->phase < 0.5f);
            for (int i = 0; i < NUM_LEGS; ++i) {
                bool inA = (i == 0 || i == 3 || i == 4);
                bool swing = (groupA ? inA : !inA);
                scheduler->leg_states[i] = swing ? LEG_SWING : LEG_SUPPORT;
            }
            break;
        }
        case GAIT_RIPPLE: {
            // simple ripple: each leg gets a 1/6th phase window to swing
            for (int i = 0; i < NUM_LEGS; ++i) {
                float leg_phase = scheduler->phase + (i / (float)NUM_LEGS);
                while (leg_phase >= 1.0f) leg_phase -= 1.0f;
                scheduler->leg_states[i] = (leg_phase < (1.0f / NUM_LEGS)) ? LEG_SWING : LEG_SUPPORT;
            }
            break;
        }
        case GAIT_WAVE: default: {
            // wave: one leg swings at a time, narrower window
            for (int i = 0; i < NUM_LEGS; ++i) {
                float leg_phase = scheduler->phase + (i / (float)NUM_LEGS);
                while (leg_phase >= 1.0f) leg_phase -= 1.0f;
                scheduler->leg_states[i] = (leg_phase < (0.6f / NUM_LEGS)) ? LEG_SWING : LEG_SUPPORT;
            }
            break;
        }
    }
}
