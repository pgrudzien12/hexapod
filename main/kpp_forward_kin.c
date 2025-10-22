/*
 * KPP Forward Kinematics Implementation
 * 
 * Converts joint angles to leg endpoint positions in body frame.
 * Uses existing leg IK geometry but performs forward transformation.
 * 
 * License: Apache-2.0
 */

#include "kpp_system.h"
#include "robot_config.h"
#include "leg.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "kpp_forward_kin";

// Forward declarations for internal functions
static esp_err_t kpp_leg_forward_kinematics(leg_handle_t leg_handle, 
                                           const float joint_angles[3], 
                                           float leg_position[3]);
static void kpp_transform_leg_to_body(int leg_index, 
                                     const float leg_pos[3], 
                                     float body_pos[3]);

esp_err_t kpp_forward_kinematics(const float joint_angles[NUM_LEGS][3], float leg_positions[NUM_LEGS][3])
{
    if (!joint_angles || !leg_positions) {
        ESP_LOGE(TAG, "Null pointer in kpp_forward_kinematics");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t overall_result = ESP_OK;

    // Process each leg
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Get leg handle from robot config
        leg_handle_t leg_handle = robot_config_get_leg(leg);
        if (!leg_handle) {
            ESP_LOGW(TAG, "No leg handle for leg %d", leg);
            // Set to zero position as fallback
            leg_positions[leg][0] = 0.0f;
            leg_positions[leg][1] = 0.0f;
            leg_positions[leg][2] = 0.0f;
            overall_result = ESP_ERR_INVALID_STATE;
            continue;
        }

        // Calculate leg endpoint position in leg-local frame
        float leg_local_pos[3];
        esp_err_t ret = kpp_leg_forward_kinematics(leg_handle, joint_angles[leg], leg_local_pos);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Forward kinematics failed for leg %d: %s", leg, esp_err_to_name(ret));
            // Set to default position as fallback
            leg_local_pos[0] = 0.15f; // Default stance outward
            leg_local_pos[1] = 0.0f;  // Default stance forward
            leg_local_pos[2] = -0.1f; // Default stance down
            overall_result = ESP_ERR_INVALID_STATE;
        }

        // Transform from leg-local frame to body frame
        kpp_transform_leg_to_body(leg, leg_local_pos, leg_positions[leg]);

        ESP_LOGD(TAG, "Leg %d: angles=[%.3f,%.3f,%.3f] -> local=[%.3f,%.3f,%.3f] -> body=[%.3f,%.3f,%.3f]",
                 leg, 
                 joint_angles[leg][0], joint_angles[leg][1], joint_angles[leg][2],
                 leg_local_pos[0], leg_local_pos[1], leg_local_pos[2],
                 leg_positions[leg][0], leg_positions[leg][1], leg_positions[leg][2]);
    }

    return overall_result;
}

// =============================================================================
// Internal Helper Functions
// =============================================================================

static esp_err_t kpp_leg_forward_kinematics(leg_handle_t leg_handle, 
                                           const float joint_angles[3], 
                                           float leg_position[3])
{
    if (!leg_handle || !joint_angles || !leg_position) {
        return ESP_ERR_INVALID_ARG;
    }

    // Access leg geometry directly from the exposed structure
    // No casting needed - clean and type-safe access
    leg_geometry_t* leg = leg_handle;
    float len_coxa = leg->len_coxa;
    float len_femur = leg->len_femur;
    float len_tibia = leg->len_tibia;

    // Extract joint angles (already includes offsets from servo commands)
    float coxa_angle = joint_angles[0];   // Hip yaw
    float femur_angle = joint_angles[1];  // Hip pitch
    float tibia_angle = joint_angles[2];  // Knee

    // Forward kinematics calculation
    // Leg-local coordinate frame: X outward, Y forward, Z up
    
    // Step 1: Calculate position after coxa rotation (hip yaw)
    // Coxa joint rotates about Z axis
    float coxa_x = len_coxa;  // Outward offset to femur joint
    
    // Step 2: Calculate femur endpoint in coxa frame
    // Femur rotates about Y axis (pitch)
    float femur_end_x = len_femur * cosf(femur_angle);
    float femur_end_z = len_femur * sinf(femur_angle);
    
    // Step 3: Calculate tibia endpoint relative to femur endpoint
    // Tibia continues from femur at the knee angle
    // Note: tibia_angle is relative to femur, so total angle is femur_angle + tibia_angle
    float total_tibia_angle = femur_angle + tibia_angle;
    float tibia_end_x = len_tibia * cosf(total_tibia_angle);
    float tibia_end_z = len_tibia * sinf(total_tibia_angle);
    
    // Step 4: Sum up the chain in leg-local coordinates before rotation
    float local_x = coxa_x + femur_end_x + tibia_end_x;  // Total outward distance
    float local_y = 0.0f;                                // No Y component in leg plane
    float local_z = femur_end_z + tibia_end_z;           // Total height
    
    // Step 5: Apply coxa rotation to get final leg-local position
    leg_position[0] = local_x * cosf(coxa_angle) - local_y * sinf(coxa_angle);  // X outward
    leg_position[1] = local_x * sinf(coxa_angle) + local_y * cosf(coxa_angle);  // Y forward  
    leg_position[2] = local_z;                                                   // Z up

    ESP_LOGD(TAG, "Forward kinematics: geometry=[%.3f,%.3f,%.3f] angles=[%.3f,%.3f,%.3f] -> pos=[%.3f,%.3f,%.3f]",
             len_coxa, len_femur, len_tibia,
             coxa_angle, femur_angle, tibia_angle,
             leg_position[0], leg_position[1], leg_position[2]);

    return ESP_OK;
}

static void kpp_transform_leg_to_body(int leg_index, const float leg_pos[3], float body_pos[3])
{
    if (!leg_pos || !body_pos) {
        return;
    }

    // Get leg mounting pose from robot config
    float mount_x, mount_y, mount_z, mount_yaw;
    robot_config_get_base_pose(leg_index, &mount_x, &mount_y, &mount_z, &mount_yaw);

    // Transform from leg-local frame to body frame
    // Leg frame: X outward, Y forward, Z up
    // Body frame: X forward, Y left, Z up
    
    // Apply yaw rotation first (rotation about Z axis)
    float cos_yaw = cosf(mount_yaw);
    float sin_yaw = sinf(mount_yaw);
    
    float rotated_x = leg_pos[0] * cos_yaw - leg_pos[1] * sin_yaw;
    float rotated_y = leg_pos[0] * sin_yaw + leg_pos[1] * cos_yaw;
    float rotated_z = leg_pos[2];  // Z unchanged by yaw rotation
    
    // Add mount position offset
    body_pos[0] = mount_x + rotated_x;  // Body X (forward)
    body_pos[1] = mount_y + rotated_y;  // Body Y (left)
    body_pos[2] = mount_z + rotated_z;  // Body Z (up)
}