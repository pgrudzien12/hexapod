#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <stdint.h>
#include "user_command.h"

#define NUM_LEGS 6

typedef enum {
    LEG_SUPPORT,
    LEG_SWING
} leg_state_t;

typedef struct {
    leg_state_t leg_states[NUM_LEGS];
    float cycle_time;
    float phase;
} gait_scheduler_t;

void gait_scheduler_init(gait_scheduler_t *scheduler, float cycle_time);
// Update leg phases/states based on command and timestep
void gait_scheduler_update(gait_scheduler_t *scheduler, float dt, const user_command_t *cmd);

#endif // GAIT_SCHEDULER_H
