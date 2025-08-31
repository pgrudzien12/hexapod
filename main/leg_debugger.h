#ifndef LEG_DEBUGGER_H
#define LEG_DEBUGGER_H

#include "whole_body_control.h"

// Initialize the leg debugger with config-derived settings
void leg_debugger_init(int enabled, int leg_index, float delta_thresh_rad, unsigned int min_interval_ms);

// Feed the latest joint angles; logs deltas for the configured leg when they change
void leg_debugger_update(const whole_body_cmd_t *cmds);

#endif // LEG_DEBUGGER_H
