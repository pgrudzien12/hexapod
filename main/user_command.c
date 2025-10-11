#include <stddef.h>
#include "user_command.h"
#include "controller.h"
#include "esp_log.h"

static const char* TAG = "user_command";

void user_command_init(void) {
    // Start controller task with default configuration (defaults to FLYSKY iBUS driver);
    // Future: configuration portal will supply a runtime-selected driver & parameters.
    controller_init(0);
}

void user_command_poll(user_command_t *cmd) {
    if (!cmd) return; 
    uint16_t ch[CONTROLLER_MAX_CHANNELS] = {0};
    controller_state_t st = {0};
    if (controller_get_channels(ch)) {
        controller_decode(ch, &st);
        // Map to command per table
        cmd->vx = st.left_vert;   // scale to m/s elsewhere
        cmd->wz = st.left_horiz;  // map to rad/s elsewhere
        cmd->z_target = st.right_vert; // normalized -1..1, map to meters later
        cmd->y_offset = st.right_horiz;
        cmd->pose_mode = st.swb_pose;
        cmd->terrain_climb = st.swd_terrain;
        cmd->step_scale = st.sra_knob; // 0..1
        cmd->enable = st.swa_arm;
        // Gait mapping
        switch (st.swc_gait) {
            case GAIT_MODE_WAVE:   cmd->gait = GAIT_WAVE; break;
            case GAIT_MODE_RIPPLE: cmd->gait = GAIT_RIPPLE; break;
            case GAIT_MODE_TRIPOD: default: cmd->gait = GAIT_TRIPOD; break;
        }
    } else {                   
        // safe defaults if no data
        cmd->vx = 0.0f;
        cmd->wz = 0.0f;  
        cmd->z_target = 0.0f;
        cmd->y_offset = 0.0f;
        cmd->gait = GAIT_TRIPOD;
        cmd->enable = false;
        cmd->pose_mode = false;
        cmd->terrain_climb = false;
        cmd->step_scale = 0.5f;

        ESP_LOGI(TAG, "No controller data available, using defaults");
    }
}