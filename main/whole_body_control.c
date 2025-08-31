#include "whole_body_control.h"
#include "leg.h"
#include "robot_config.h"
#include <math.h>
#include <assert.h>
#include "esp_log.h"

// Temporary convention bridging:
// - swing_trajectory outputs body-frame positions (meters):
//     X_body: forward (+), Y_body: left (+), Z_body: down (+)
// - leg IK expects leg-local:
//     X_leg: outward from body, Y_leg: forward, Z_leg: down (see leg.h)
// - For now, we don't have per-leg mount rotations/positions; we only mirror left/right:
//     left legs (0,1,2): X_leg = +Y_body; right legs (3,4,5): X_leg = -Y_body
//     Y_leg = X_body for all
//     Z_leg = Z_body for all (already down-positive)
// TODO: Replace with full per-leg transform using mount poses and yaw angles.

const char *TAG = "wbc";

static inline int is_right_leg(int idx) {
    return (idx >= 3); // 0,1,2 left; 3,4,5 right
}

void whole_body_control_compute(const swing_trajectory_t *trajectory, whole_body_cmd_t *cmds) {
    assert(trajectory != NULL);
    assert(cmds != NULL);

    for (int i = 0; i < NUM_LEGS; ++i) {
        const foot_position_t *b = &trajectory->desired_positions[i];
        float x_body = b->x; // forward (+)
        float y_body = b->y; // left (+)
        float z_body = b->z; // down (+)

        // Transform body-frame target to leg-local using mount pose (r_base, yaw).
        float bx, by, bz, psi;
        robot_config_get_base_pose(i, &bx, &by, &bz, &psi);

        // Translate to hip origin
        float px = x_body - bx;
        float py = y_body - by;
        float pz = z_body - bz;
        if (i < 3)
            ESP_LOGI(TAG, "Leg %d: Body (%.3f, %.3f, %.3f) -> Leg: (%.3f, %.3f, %.3f)", i, x_body, y_body, z_body, px, py, pz);

        // Rotate by -yaw to align leg frame (X_leg outward, Y_leg forward, Z_leg down)
        float c = cosf(-psi), s = sinf(-psi);
        float x_leg = c * px - s * py; // outward
        float y_leg = s * px + c * py; // forward
        float z_leg = pz;              // down (unchanged for yaw rotation)

        if (i < 3)
            ESP_LOGI(TAG, "Leg %d: Rotated Leg: (%.3f, %.3f, %.3f)", i, x_leg, y_leg, z_leg);

        // Look up per-leg IK geometry from robot_config.
        leg_handle_t leg = robot_config_get_leg(i);
        if (!leg) {
            // If no geometry, produce neutral zeros; robot_control should handle safely.
            cmds->joint_cmds[i].joint_angles[0] = 0.0f;
            cmds->joint_cmds[i].joint_angles[1] = 0.0f;
            cmds->joint_cmds[i].joint_angles[2] = 0.0f;
            continue;
        }

        // Run IK to get joint angles for this foot target.
        leg_angles_t q;
        if (leg_ik_solve(leg, x_leg, y_leg, z_leg, &q) == ESP_OK) {
            // Store actual joint angles (radians). Order: coxa, femur, tibia.
            // NOTE: Calibration offsets/limits are applied later in robot_control.
            cmds->joint_cmds[i].joint_angles[0] = q.coxa;
            cmds->joint_cmds[i].joint_angles[1] = q.femur;
            cmds->joint_cmds[i].joint_angles[2] = q.tibia;
        } else {
            // IK failed (out of reach, etc.). For now, fall back to zeros.
            // TODO: Add error signaling so robot_control can engage a safe stop.
            cmds->joint_cmds[i].joint_angles[0] = 0.0f;
            cmds->joint_cmds[i].joint_angles[1] = 0.0f;
            cmds->joint_cmds[i].joint_angles[2] = 0.0f;
        }
    }
}
