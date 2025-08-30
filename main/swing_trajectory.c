#include "swing_trajectory.h"
#include "gait_scheduler.h"

void swing_trajectory_init(swing_trajectory_t *trajectory, float step_length, float clearance_height) {
    trajectory->step_length = step_length;
    trajectory->clearance_height = clearance_height;
    for (int i = 0; i < NUM_LEGS; ++i) {
        trajectory->desired_positions[i].x = 0.0f;
        trajectory->desired_positions[i].y = 0.0f;
        trajectory->desired_positions[i].z = 0.0f;
    }
}

void swing_trajectory_generate(swing_trajectory_t *trajectory, const gait_scheduler_t *scheduler, const user_command_t *cmd) {
    // Basic placeholder: if leg is swinging, move foot forward with clearance; if supporting, move backward flat
    float step = trajectory->step_length * (cmd ? (cmd->step_scale > 0.0f ? cmd->step_scale : 0.5f) : 0.5f);
    float clr = trajectory->clearance_height * (cmd && cmd->terrain_climb ? 1.5f : 1.0f);
    float vx = cmd ? cmd->vx : 0.0f; // -1..1 normalized currently
    float body_z = cmd ? cmd->z_target : 0.0f; // map to meters elsewhere
    float body_y = cmd ? cmd->y_offset : 0.0f;

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_position_t *p = &trajectory->desired_positions[i];
        bool isSwing = scheduler ? (scheduler->leg_states[i] == LEG_SWING) : false;
        // X: forward/back relative to body; Y: lateral offset; Z: height
        if (isSwing) {
            // simple arc: forward and up
            p->x = step * vx;
            p->y = body_y;
            p->z = body_z - clr; // +down convention assumed elsewhere; here negative to lift
        } else {
            // support phase: move backward slightly to simulate stance
            p->x = -0.5f * step * vx;
            p->y = body_y;
            p->z = body_z;
        }
    }
}
