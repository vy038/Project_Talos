// ik_solver.c
#include "ik_solver.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "IK";

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

esp_err_t xIKSolverInit(void) {
    ESP_LOGI(TAG, "IK Solver initialized for 6-DOF arm");
    ESP_LOGI(TAG, "Base height: %.1f mm, Base tilt: %.1f deg", ARM_BASE_HEIGHT, ARM_BASE_ANGLE);
    ESP_LOGI(TAG, "Links: %.1f, %.1f, %.1f, %.1f mm", 
             ARM_LINK1_LENGTH, ARM_LINK2_LENGTH, ARM_LINK3_LENGTH, ARM_LINK4_LENGTH);
    ESP_LOGI(TAG, "Gripper angle offset: %.1f deg", ARM_GRIPPER_ANGLE);
    return ESP_OK;
}

bool bIKIsReachable(const ik_target_t *target) {
    if (!target) return false;
    
    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);
    float gripper_offset_rad = DEG_TO_RAD(ARM_GRIPPER_ANGLE);
    
    // Offset target by base height (shoulder pivot is at ARM_BASE_HEIGHT above origin)
    float target_z_offset = target->z - ARM_BASE_HEIGHT;
    
    // Transform to arm plane
    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);
    
    // Effective gripper angle = requested pitch + fixed gripper offset
    float effective_pitch_rad = DEG_TO_RAD(target->pitch) + gripper_offset_rad;
    float yaw_rad = DEG_TO_RAD(target->yaw);
    
    // Wrist center position (subtract gripper offset)
    float wrist_x = plane_x - ARM_LINK4_LENGTH * cosf(effective_pitch_rad) * cosf(yaw_rad);
    float wrist_z = plane_z - ARM_LINK4_LENGTH * sinf(effective_pitch_rad);
    
    // Check 2-link reach
    float dist = sqrtf(wrist_x * wrist_x + wrist_z * wrist_z);
    float max_reach = ARM_LINK1_LENGTH + ARM_LINK2_LENGTH;
    float min_reach = fabsf(ARM_LINK1_LENGTH - ARM_LINK2_LENGTH);
    
    return (dist <= max_reach && dist >= min_reach);
}

esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution) {
    if (!target || !solution) {
        return ESP_ERR_INVALID_ARG;
    }
    
    solution->valid = false;
    
    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);
    float gripper_offset_rad = DEG_TO_RAD(ARM_GRIPPER_ANGLE);

    // Step 1: Base rotation (world frame XY plane)
    solution->base_rotation = RAD_TO_DEG(atan2f(target->y, target->x));
    solution->base_rotation = CLAMP(solution->base_rotation, BASE_ROTATION_MIN, BASE_ROTATION_MAX);

    // Step 2: Offset target position relative to shoulder pivot
    // The shoulder joint is at height ARM_BASE_HEIGHT above world origin
    float target_z_offset = target->z - ARM_BASE_HEIGHT;
    
    // Step 3: Transform to tilted arm plane
    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);

    // Step 4: Find wrist center by working backwards from gripper tip
    // Account for fixed gripper angle: actual gripper direction = wrist angle + gripper offset
    // So when user requests pitch=0 (horizontal gripper), wrist must compensate for gripper droop
    float effective_pitch_rad = DEG_TO_RAD(target->pitch) + gripper_offset_rad;
    float yaw_rad = DEG_TO_RAD(target->yaw);
    
    float wrist_x = plane_x - ARM_LINK4_LENGTH * cosf(effective_pitch_rad) * cosf(yaw_rad);
    float wrist_z = plane_z - ARM_LINK4_LENGTH * sinf(effective_pitch_rad);

    // Step 5: Check reachability
    float dist_to_wrist = sqrtf(wrist_x * wrist_x + wrist_z * wrist_z);
    float max_reach = ARM_LINK1_LENGTH + ARM_LINK2_LENGTH;
    float min_reach = fabsf(ARM_LINK1_LENGTH - ARM_LINK2_LENGTH);
    
    if (dist_to_wrist > max_reach || dist_to_wrist < min_reach) {
        ESP_LOGW(TAG, "Wrist unreachable: dist=%.2f, range=[%.2f, %.2f]",
                 dist_to_wrist, min_reach, max_reach);
        return ESP_ERR_INVALID_ARG;
    }

    // Step 6: Solve elbow (law of cosines)
    float cos_elbow = (ARM_LINK1_LENGTH * ARM_LINK1_LENGTH + 
                       ARM_LINK2_LENGTH * ARM_LINK2_LENGTH - 
                       dist_to_wrist * dist_to_wrist) / 
                      (2.0f * ARM_LINK1_LENGTH * ARM_LINK2_LENGTH);
    cos_elbow = CLAMP(cos_elbow, -1.0f, 1.0f);
    
    solution->elbow = 180.0f - RAD_TO_DEG(acosf(cos_elbow));
    solution->elbow = CLAMP(solution->elbow, ELBOW_MIN, ELBOW_MAX);

    // Step 7: Solve shoulder
    float angle_to_wrist = atan2f(wrist_z, wrist_x);
    
    float cos_shoulder_internal = (ARM_LINK1_LENGTH * ARM_LINK1_LENGTH + 
                                   dist_to_wrist * dist_to_wrist - 
                                   ARM_LINK2_LENGTH * ARM_LINK2_LENGTH) / 
                                  (2.0f * ARM_LINK1_LENGTH * dist_to_wrist);
    cos_shoulder_internal = CLAMP(cos_shoulder_internal, -1.0f, 1.0f);
    
    solution->shoulder = RAD_TO_DEG(angle_to_wrist + acosf(cos_shoulder_internal));
    solution->shoulder = CLAMP(solution->shoulder, SHOULDER_MIN, SHOULDER_MAX);

    // Step 8: Solve wrist pitch
    // Chain: shoulder → elbow → wrist → gripper
    // Gripper angle in arm plane = shoulder + (elbow - 180) + wrist_pitch + gripper_offset
    // User wants: gripper angle = target->pitch
    // So: wrist_pitch = target->pitch - shoulder - (elbow - 180) - gripper_offset
    float cumulative_angle = solution->shoulder + solution->elbow - 180.0f;
    solution->wrist_pitch = target->pitch - cumulative_angle - ARM_GRIPPER_ANGLE;
    
    // Normalize to [-180, 180]
    while (solution->wrist_pitch > 180.0f) solution->wrist_pitch -= 360.0f;
    while (solution->wrist_pitch < -180.0f) solution->wrist_pitch += 360.0f;
    
    solution->wrist_pitch = CLAMP(solution->wrist_pitch, WRIST_PITCH_MIN, WRIST_PITCH_MAX);

    // Step 9: Roll and yaw pass-through
    solution->wrist_roll = CLAMP(target->roll, WRIST_ROLL_MIN, WRIST_ROLL_MAX);
    solution->wrist_yaw = CLAMP(target->yaw, WRIST_YAW_MIN, WRIST_YAW_MAX);

    solution->valid = true;

    ESP_LOGD(TAG, "IK: base=%.1f, shoulder=%.1f, elbow=%.1f, wp=%.1f, wr=%.1f, wy=%.1f",
             solution->base_rotation, solution->shoulder, solution->elbow,
             solution->wrist_pitch, solution->wrist_roll, solution->wrist_yaw);

    return ESP_OK;
}

esp_err_t xIKForward(const ik_solution_t *solution, ik_target_t *position) {
    if (!solution || !position) {
        return ESP_ERR_INVALID_ARG;
    }

    float base_rad = DEG_TO_RAD(solution->base_rotation);
    float shoulder_rad = DEG_TO_RAD(solution->shoulder);
    float elbow_rad = DEG_TO_RAD(solution->elbow);
    float wrist_pitch_rad = DEG_TO_RAD(solution->wrist_pitch);
    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);
    float gripper_offset_rad = DEG_TO_RAD(ARM_GRIPPER_ANGLE);

    // Cumulative angles through arm chain
    float angle1 = shoulder_rad;
    float angle2 = shoulder_rad + elbow_rad - M_PI;
    float angle3 = angle2 + wrist_pitch_rad + gripper_offset_rad;  // Include gripper offset

    // Position in arm plane
    float x_plane = ARM_LINK1_LENGTH * cosf(angle1) + 
                    ARM_LINK2_LENGTH * cosf(angle2) +
                    (ARM_LINK3_LENGTH + ARM_LINK4_LENGTH) * cosf(angle3);
    
    float z_plane = ARM_LINK1_LENGTH * sinf(angle1) + 
                    ARM_LINK2_LENGTH * sinf(angle2) +
                    (ARM_LINK3_LENGTH + ARM_LINK4_LENGTH) * sinf(angle3);

    // Transform from arm plane to world
    float r_world = x_plane * cosf(base_tilt_rad) - z_plane * sinf(base_tilt_rad);
    float z_world = x_plane * sinf(base_tilt_rad) + z_plane * cosf(base_tilt_rad);
    
    // Add back base height offset
    position->z = z_world + ARM_BASE_HEIGHT;
    
    // Apply base rotation
    position->x = r_world * cosf(base_rad);
    position->y = r_world * sinf(base_rad);

    // Output orientation (what the gripper is actually pointing at)
    position->roll = solution->wrist_roll;
    position->pitch = solution->wrist_pitch + solution->shoulder + solution->elbow - 180.0f + ARM_GRIPPER_ANGLE;
    position->yaw = solution->wrist_yaw;

    return ESP_OK;
}