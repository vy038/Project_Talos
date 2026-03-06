/**
 * @file ik_solver.c
 * @brief Analytic inverse kinematics solver for the 3-DOF robot arm.
 *
 * See ik_solver.h for coordinate conventions and tuning constants.
 */
#include "ik_solver.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "IK";

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

esp_err_t xIKSolverInit(void) {
    ESP_LOGI(TAG, "IK Solver initialized for 3-DOF arm");
    ESP_LOGI(TAG, "Base height: %.1f mm, Base tilt: %.1f deg", ARM_BASE_HEIGHT, ARM_BASE_ANGLE);
    ESP_LOGI(TAG, "Links: %.1f, %.1f mm", ARM_LINK1_LENGTH, ARM_LINK2_LENGTH);
    return ESP_OK;
}

bool bIKIsReachable(const ik_target_t *target) {
    if (!target) return false;

    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);

    float target_z_offset = target->z - ARM_BASE_HEIGHT;

    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);

    float dist = sqrtf(plane_x * plane_x + plane_z * plane_z);
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

    // base rotation (world frame XY plane)
    solution->base_rotation = RAD_TO_DEG(atan2f(target->y, target->x));
    solution->base_rotation = CLAMP(solution->base_rotation, BASE_ROTATION_MIN, BASE_ROTATION_MAX);

    // offset by shoulder pivot height
    float target_z_offset = target->z - ARM_BASE_HEIGHT;

    // transform to tilted arm plane
    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);

    // check reachability
    float dist = sqrtf(plane_x * plane_x + plane_z * plane_z);
    float max_reach = ARM_LINK1_LENGTH + ARM_LINK2_LENGTH;
    float min_reach = fabsf(ARM_LINK1_LENGTH - ARM_LINK2_LENGTH);

    if (dist > max_reach || dist < min_reach) {
        ESP_LOGW(TAG, "Target unreachable: dist=%.2f, range=[%.2f, %.2f]",
                 dist, min_reach, max_reach);
        return ESP_ERR_INVALID_ARG;
    }

    // solve elbow (law of cosines)
    float cos_elbow = (ARM_LINK1_LENGTH * ARM_LINK1_LENGTH +
                       ARM_LINK2_LENGTH * ARM_LINK2_LENGTH -
                       dist * dist) /
                      (2.0f * ARM_LINK1_LENGTH * ARM_LINK2_LENGTH);
    cos_elbow = CLAMP(cos_elbow, -1.0f, 1.0f);

    solution->elbow = 180.0f - RAD_TO_DEG(acosf(cos_elbow));
    solution->elbow = CLAMP(solution->elbow, ELBOW_MIN, ELBOW_MAX);

    // solve shoulder
    float angle_to_target = atan2f(plane_z, plane_x);

    float cos_shoulder_internal = (ARM_LINK1_LENGTH * ARM_LINK1_LENGTH +
                                   dist * dist -
                                   ARM_LINK2_LENGTH * ARM_LINK2_LENGTH) /
                                  (2.0f * ARM_LINK1_LENGTH * dist);
    cos_shoulder_internal = CLAMP(cos_shoulder_internal, -1.0f, 1.0f);

    solution->shoulder = RAD_TO_DEG(angle_to_target + acosf(cos_shoulder_internal));
    solution->shoulder = CLAMP(solution->shoulder, SHOULDER_MIN, SHOULDER_MAX);

    solution->valid = true;

    ESP_LOGD(TAG, "IK: base=%.1f, shoulder=%.1f, elbow=%.1f",
             solution->base_rotation, solution->shoulder, solution->elbow);

    return ESP_OK;
}

esp_err_t xIKCameraToArm(const ik_target_t *camera_coords, ik_target_t *arm_coords) {
    if (!camera_coords || !arm_coords) {
        return ESP_ERR_INVALID_ARG;
    }

    arm_coords->x = camera_coords->x + CAMERA_OFFSET_FORWARD_MM;
    arm_coords->y = camera_coords->y + CAMERA_OFFSET_LATERAL_MM;
    arm_coords->z = camera_coords->z + CAMERA_OFFSET_VERTICAL_MM;

    ESP_LOGD(TAG, "Camera->Arm: (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f)",
             camera_coords->x, camera_coords->y, camera_coords->z,
             arm_coords->x,    arm_coords->y,    arm_coords->z);

    return ESP_OK;
}

esp_err_t xIKForward(const ik_solution_t *solution, ik_target_t *position) {
    if (!solution || !position) {
        return ESP_ERR_INVALID_ARG;
    }

    float base_rad = DEG_TO_RAD(solution->base_rotation);
    float shoulder_rad = DEG_TO_RAD(solution->shoulder);
    float elbow_rad = DEG_TO_RAD(solution->elbow);
    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);

    // cumulative angles through 2-link chain
    float angle1 = shoulder_rad;
    float angle2 = shoulder_rad + elbow_rad - M_PI;

    // tip position in arm plane
    float x_plane = ARM_LINK1_LENGTH * cosf(angle1) +
                    ARM_LINK2_LENGTH * cosf(angle2);

    float z_plane = ARM_LINK1_LENGTH * sinf(angle1) +
                    ARM_LINK2_LENGTH * sinf(angle2);

    // transform from arm plane to world (undo tilt)
    float r_world = x_plane * cosf(base_tilt_rad) - z_plane * sinf(base_tilt_rad);
    float z_world = x_plane * sinf(base_tilt_rad) + z_plane * cosf(base_tilt_rad);

    position->z = z_world + ARM_BASE_HEIGHT;

    // apply base rotation to get world XY
    position->x = r_world * cosf(base_rad);
    position->y = r_world * sinf(base_rad);

    return ESP_OK;
}
