#ifndef USER_COMMAND_H
#define USER_COMMAND_H

#include <stdbool.h>

typedef enum {
    GAIT_TRIPOD = 0,
    GAIT_RIPPLE = 1,
    GAIT_WAVE   = 2,
} gait_type_t;

typedef struct {
    // Command inputs (normalized from controller_decode unless otherwise noted)
    // motion (normalized -1..+1)
    float vx;       // forward command (from CH2 Left Vert), normalized -1..+1
    float wz;       // yaw command (from CH4 Left Horiz), normalized -1..+1
    // body pose targets (normalized -1..+1); scale to meters in trajectory/WBC layer
    float z_target; // normalized body height target (from CH3 Right Vert)
    float y_offset; // normalized lateral shift (from CH1 Right Horiz)
    // mode controls
    gait_type_t gait;
    bool enable;
    bool pose_mode;      // SWB
    bool terrain_climb;  // SWD
    float step_scale;    // 0..1 from SRA (CH10)
} user_command_t;

void user_command_init(void);
// Non-blocking poll; fills cmd with latest desired command
void user_command_poll(user_command_t *cmd);

#endif // USER_COMMAND_H