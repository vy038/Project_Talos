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

/**
 * Solve inverse kinematics for a 3-DOF arm with a forward-tilted base.
 *
 * High level: the gripper tip must reach (target->x, target->y, target->z) in
 * the robot body frame (x=forward, y=left, z=up). We solve in four stages and
 * log every intermediate so misbehaviour can be pinpointed from serial output.
 *
 *   [1] Base rotation: yaw the whole arm in the horizontal plane to face the
 *       ball. Computed from atan2(y, x) and shifted by +90° because the servo
 *       convention puts 90° at center (forward), not 0°.
 *
 *   [2] Project the target into the arm's tilted vertical plane: the shoulder
 *       axis is tilted ARM_BASE_ANGLE forward, so the (x, z) coordinates have
 *       to be rotated to match the arm's reference frame.
 *
 *   [3] Subtract link3 (the rigid gripper segment): the 2-link shoulder/elbow
 *       solver positions the *wrist*, not the gripper tip. We back off by the
 *       link3 vector to get the wrist target.
 *
 *   [4] Law of cosines on the (shoulder, elbow, wrist) triangle to recover
 *       shoulder + elbow joint angles. Each is clamped to its joint limit and
 *       a warning is logged if the unclamped value was outside the limit.
 *
 * Every joint clamp is announced via WARN, every step values at INFO so you
 * can watch the pipeline in real time on the serial console.
 */
esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution) {
    if (!target || !solution) {
        return ESP_ERR_INVALID_ARG;
    }

    solution->valid = false;

    ESP_LOGI(TAG, "── xIKSolve start ──");
    ESP_LOGI(TAG, "[in ] target arm-frame (x=%.1f, y=%.1f, z=%.1f) mm",
             target->x, target->y, target->z);

    float base_tilt_rad   = DEG_TO_RAD(ARM_BASE_ANGLE);
    float target_z_offset = target->z - ARM_BASE_HEIGHT;     // height above shoulder pivot
    float r_horiz         = sqrtf(target->x * target->x + target->y * target->y);

    ESP_LOGI(TAG, "[geo] base_tilt=%.1f° base_height=%.1f r_horiz=%.2f z_off=%.2f",
             ARM_BASE_ANGLE, ARM_BASE_HEIGHT, r_horiz, target_z_offset);

    // [1] Base rotation — atan2 frame (0=forward) shifted to servo frame (90=center)
    float base_raw = 90.0f + RAD_TO_DEG(atan2f(target->y, target->x));
    solution->base_rotation = CLAMP(base_raw, BASE_ROTATION_MIN, BASE_ROTATION_MAX);
    if (solution->base_rotation != base_raw) {
        ESP_LOGW(TAG, "[1] base CLAMPED %.2f -> %.2f° (limits %.0f..%.0f)",
                 base_raw, solution->base_rotation,
                 BASE_ROTATION_MIN, BASE_ROTATION_MAX);
    } else {
        ESP_LOGI(TAG, "[1] base_rotation=%.2f° (raw atan2 -> +90 offset)", solution->base_rotation);
    }

    // [2] Rotate target into the arm's tilted vertical plane.
    // plane_x = projected forward reach, plane_z = projected height (in arm plane)
    float plane_x = r_horiz * cosf(base_tilt_rad) + target_z_offset * sinf(base_tilt_rad);
    float plane_z = -r_horiz * sinf(base_tilt_rad) + target_z_offset * cosf(base_tilt_rad);
    ESP_LOGI(TAG, "[2] tilted plane (gripper-tip target): plane_x=%.2f plane_z=%.2f", plane_x, plane_z);

    // [3] Step back along link3 to find the *wrist* target.
    // link3 extends LINK3_LENGTH forward and LINK3_OFFSET downward in the arm plane.
    plane_x -= ARM_LINK3_LENGTH;
    plane_z += ARM_LINK3_OFFSET;
    ESP_LOGI(TAG, "[3] wrist target after link3 subtract: plane_x=%.2f plane_z=%.2f", plane_x, plane_z);

    // [4] Reachability — wrist must fall in the annulus between |L1-L2| and L1+L2.
    float dist      = sqrtf(plane_x * plane_x + plane_z * plane_z);
    float max_reach = s_l1_eff + s_l2_eff;
    float min_reach = fabsf(s_l1_eff - s_l2_eff);
    ESP_LOGI(TAG, "[4] wrist dist=%.2f, reachable range=[%.2f, %.2f]", dist, min_reach, max_reach);

    if (dist > max_reach || dist < min_reach) {
        ESP_LOGW(TAG, "[4] UNREACHABLE: dist=%.2f outside [%.2f, %.2f] — returning error",
                 dist, min_reach, max_reach);
        return ESP_ERR_INVALID_ARG;
    }

    // [4a] Elbow: interior angle of (shoulder, elbow, wrist) triangle.
    // FK convention: elbow_servo=180 → links collinear (extended), elbow_servo=0 → folded.
    // The law-of-cosines interior angle equals the servo angle directly:
    //   extended (dist=L1+L2) → cos=-1 → acos=180° ✓
    //   folded  (dist=|L1-L2|) → cos=+1 → acos=0°   ✓
    float cos_elbow = (s_l1_eff * s_l1_eff + s_l2_eff * s_l2_eff - dist * dist) /
                      (2.0f * s_l1_eff * s_l2_eff);
    cos_elbow = CLAMP(cos_elbow, -1.0f, 1.0f);
    float elbow_raw = RAD_TO_DEG(acosf(cos_elbow));
    solution->elbow = CLAMP(elbow_raw, ELBOW_MIN, ELBOW_MAX);
    if (solution->elbow != elbow_raw) {
        ESP_LOGW(TAG, "[4a] elbow CLAMPED %.2f -> %.2f° (limits %.0f..%.0f, cos=%.3f)",
                 elbow_raw, solution->elbow, ELBOW_MIN, ELBOW_MAX, cos_elbow);
    } else {
        ESP_LOGI(TAG, "[4a] elbow=%.2f° (cos=%.3f)", solution->elbow, cos_elbow);
    }

    // [4b] Shoulder: angle to wrist target + angle inside the triangle at shoulder.
    float angle_to_target = atan2f(plane_z, plane_x);
    float cos_shoulder_internal = (s_l1_eff * s_l1_eff + dist * dist - s_l2_eff * s_l2_eff) /
                                  (2.0f * s_l1_eff * dist);
    cos_shoulder_internal = CLAMP(cos_shoulder_internal, -1.0f, 1.0f);
    float shoulder_raw = RAD_TO_DEG(angle_to_target + acosf(cos_shoulder_internal));
    solution->shoulder = CLAMP(shoulder_raw, SHOULDER_MIN, SHOULDER_MAX);
    if (solution->shoulder != shoulder_raw) {
        ESP_LOGW(TAG, "[4b] shoulder CLAMPED %.2f -> %.2f° (limits %.0f..%.0f, "
                      "angle_to_target=%.2f° internal=%.2f°)",
                 shoulder_raw, solution->shoulder, SHOULDER_MIN, SHOULDER_MAX,
                 RAD_TO_DEG(angle_to_target), RAD_TO_DEG(acosf(cos_shoulder_internal)));
    } else {
        ESP_LOGI(TAG, "[4b] shoulder=%.2f° (angle_to_target=%.2f°, internal=%.2f°)",
                 solution->shoulder, RAD_TO_DEG(angle_to_target),
                 RAD_TO_DEG(acosf(cos_shoulder_internal)));
    }

    solution->valid = true;
    ESP_LOGI(TAG, "[out] base=%.2f shoulder=%.2f elbow=%.2f",
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

    // subtract 90 to convert servo frame (90=forward) back to math frame (0=forward)
    float base_rad = DEG_TO_RAD(solution->base_rotation - 90.0f);
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
