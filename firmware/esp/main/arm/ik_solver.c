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

// each link has a perpendicular offset, so the real reach is the hypotenuse not the nominal length
// precomputed in xIKSolverInit so we dont sqrt every solve call
static float s_l1_eff = 0.0f;
static float s_l2_eff = 0.0f;
static float s_l3_eff = 0.0f;

esp_err_t xIKSolverInit(void) {
    s_l1_eff = sqrtf(ARM_LINK1_LENGTH * ARM_LINK1_LENGTH + ARM_LINK1_OFFSET * ARM_LINK1_OFFSET);
    s_l2_eff = sqrtf(ARM_LINK2_LENGTH * ARM_LINK2_LENGTH + ARM_LINK2_OFFSET * ARM_LINK2_OFFSET);
    s_l3_eff = sqrtf(ARM_LINK3_LENGTH * ARM_LINK3_LENGTH + ARM_LINK3_OFFSET * ARM_LINK3_OFFSET);

    ESP_LOGI(TAG, "IK Solver initialized for 3-DOF arm");
    ESP_LOGI(TAG, "Base height: %.1f mm, Base tilt: %.1f deg", ARM_BASE_HEIGHT, ARM_BASE_ANGLE);
    ESP_LOGI(TAG, "Link1: %.1f mm (offset %.1f mm -> eff %.2f mm)",
             ARM_LINK1_LENGTH, ARM_LINK1_OFFSET, s_l1_eff);
    ESP_LOGI(TAG, "Link2: %.1f mm (offset %.1f mm -> eff %.2f mm)",
             ARM_LINK2_LENGTH, ARM_LINK2_OFFSET, s_l2_eff);
    ESP_LOGI(TAG, "Link3 (gripper): %.1f mm (offset %.1f mm -> eff %.2f mm)",
             ARM_LINK3_LENGTH, ARM_LINK3_OFFSET, s_l3_eff);
    ESP_LOGI(TAG, "Wrist reach: [%.1f, %.1f] mm",
             fabsf(s_l1_eff - s_l2_eff), s_l1_eff + s_l2_eff);
    return ESP_OK;
}

bool bIKIsReachable(const ik_target_t *target) {
    if (!target) return false;

    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);

    float target_z_offset = target->z - ARM_BASE_HEIGHT;
    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);

    // back out link3 to find where the wrist needs to be, not the gripper tip
    plane_x -= ARM_LINK3_LENGTH;
    plane_z += ARM_LINK3_OFFSET;

    float dist = sqrtf(plane_x * plane_x + plane_z * plane_z);
    float max_reach = s_l1_eff + s_l2_eff;
    float min_reach = fabsf(s_l1_eff - s_l2_eff);

    return (dist <= max_reach && dist >= min_reach);
}

esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution) {
    if (!target || !solution) {
        return ESP_ERR_INVALID_ARG;
    }

    solution->valid = false;

    float base_tilt_rad = DEG_TO_RAD(ARM_BASE_ANGLE);

    // base rotation points the arm plane toward the target in XY
    solution->base_rotation = RAD_TO_DEG(atan2f(target->y, target->x));
    solution->base_rotation = CLAMP(solution->base_rotation, BASE_ROTATION_MIN, BASE_ROTATION_MAX);

    // offset by shoulder height, then project into the tilted arm plane
    float target_z_offset = target->z - ARM_BASE_HEIGHT;
    float r_xy = sqrtf(target->x * target->x + target->y * target->y);
    float plane_x = r_xy * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_xy * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);

    // the solver places the wrist, not the gripper tip, so subtract link3 first
    // link3 goes forward LINK3_LENGTH and down LINK3_OFFSET in arm plane coords
    plane_x -= ARM_LINK3_LENGTH;
    plane_z += ARM_LINK3_OFFSET;

    // reachability check using effective lengths
    float dist = sqrtf(plane_x * plane_x + plane_z * plane_z);
    float max_reach = s_l1_eff + s_l2_eff;
    float min_reach = fabsf(s_l1_eff - s_l2_eff);

    if (dist > max_reach || dist < min_reach) {
        ESP_LOGW(TAG, "Target unreachable: wrist dist=%.2f, range=[%.2f, %.2f]",
                 dist, min_reach, max_reach);
        return ESP_ERR_INVALID_ARG;
    }

    // law of cosines for elbow using effective link lengths
    float cos_elbow = (s_l1_eff * s_l1_eff + s_l2_eff * s_l2_eff - dist * dist) /
                      (2.0f * s_l1_eff * s_l2_eff);
    cos_elbow = CLAMP(cos_elbow, -1.0f, 1.0f);

    solution->elbow = 180.0f - RAD_TO_DEG(acosf(cos_elbow));
    solution->elbow = CLAMP(solution->elbow, ELBOW_MIN, ELBOW_MAX);

    // shoulder angle
    float angle_to_target = atan2f(plane_z, plane_x);

    float cos_shoulder_internal = (s_l1_eff * s_l1_eff + dist * dist - s_l2_eff * s_l2_eff) /
                                  (2.0f * s_l1_eff * dist);
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

    // cumulative angles through the 2-link chain to the wrist
    float angle1 = shoulder_rad;
    float angle2 = shoulder_rad + elbow_rad - M_PI;

    float x_plane = s_l1_eff * cosf(angle1) + s_l2_eff * cosf(angle2);
    float z_plane = s_l1_eff * sinf(angle1) + s_l2_eff * sinf(angle2);

    // add link3 to get gripper tip position
    x_plane += ARM_LINK3_LENGTH;
    z_plane -= ARM_LINK3_OFFSET;

    // transform from arm plane back to world frame
    float r_world = x_plane * cosf(base_tilt_rad) - z_plane * sinf(base_tilt_rad);
    float z_world = x_plane * sinf(base_tilt_rad) + z_plane * cosf(base_tilt_rad);

    position->z = z_world + ARM_BASE_HEIGHT;
    position->x = r_world * cosf(base_rad);
    position->y = r_world * sinf(base_rad);

    return ESP_OK;
}
