#include "whole_body_control.h"
#include "leg.h"
#include "robot_config.h"
#include <math.h>
#include <assert.h>
#include "esp_log.h"
#include "esp_timer.h"

// Frame bridging:
// - swing_trajectory outputs body-frame foot targets (meters):
//     Body: X forward (+), Y left (+), Z up (+)
// - leg IK expects leg-local frame:
//     Leg: X outward (+), Y forward (+), Z up (+)
// - We use per-leg base pose (position + yaw) from robot_config to rotate/translate.
// - No mirroring hacks remain; yaw handles left/right mounting. Ensure base yaw values
//   remain +90 deg for left side outward, -90 deg for right side outward in RH frame.

static const char *TAG = "wbc";

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
        // ESP_LOGI(TAG, "Leg %d: Body (%.3f, %.3f, %.3f) -> Leg: (%.3f, %.3f, %.3f)", i, x_body, y_body, z_body, px, py, pz);

        // Rotate by -yaw to align leg frame (X_leg outward, Y_leg forward, Z_leg down)
        float c = cosf(-psi), s = sinf(-psi);
        float x_leg = c * px - s * py; // outward
        float y_leg = s * px + c * py; // forward
        float z_leg = pz;              // down (unchanged for yaw rotation)

        // ESP_LOGI(TAG, "Leg %d: Rotated Leg: (%.3f, %.3f, %.3f)", i, x_leg, y_leg, z_leg);

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
            // if (i != -1) {
            //     ESP_LOGI(TAG, "(%lld)Leg %d IK: BodyXYZ(%.3f, %.3f, %.3f) -> LegXYZ(%.3f, %.3f, %.3f) -> LegAng(%.3f, %.3f, %.3f)", esp_timer_get_time(), i, x_body, y_body, z_body, x_leg, y_leg, z_leg, q.coxa, q.femur, q.tibia);
            // }
            cmds->joint_cmds[i].joint_angles[0] = q.coxa;
            cmds->joint_cmds[i].joint_angles[1] = q.femur;
            cmds->joint_cmds[i].joint_angles[2] = q.tibia;
        } else {
            // IK failed (out of reach, etc.). For now, fall back to zeros.
            // TODO: Add error signaling so robot_control can engage a safe stop.
            ESP_LOGW(TAG, "Leg %d IK failed: BodyXYZ(%.3f, %.3f, %.3f) -> LegXYZ(%.3f, %.3f, %.3f) -> LegAng(0, 0, 0)", i, x_body, y_body, z_body, x_leg, y_leg, z_leg);

            cmds->joint_cmds[i].joint_angles[0] = 0.0f;
            cmds->joint_cmds[i].joint_angles[1] = 0.0f;
            cmds->joint_cmds[i].joint_angles[2] = 0.0f;
        }
    }
}