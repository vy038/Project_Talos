/**
 * @file ik_solver.h
 * @brief Inverse kinematics solver for the 3-DOF robot arm.
 *
 * Provides analytic IK for a shoulder-elbow-base arm with an optional
 * base-mounting tilt. All coordinates use the robot body frame:
 *   x = forward, y = left, z = up.
 *
 * Includes a helper to convert camera-frame object coordinates to arm-frame
 * coordinates using the known physical offset between the two origins.
 */

#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* -------------------------------------------------------------------------- */
/* Arm geometry — fill in from CAD model (mm / degrees)                       */
/* -------------------------------------------------------------------------- */

/** Base mounting tilt in degrees (0 = shoulder axis horizontal, 90 = vertical). */
#define ARM_BASE_ANGLE       38.0f

/** Vertical distance from robot body frame origin to shoulder pivot (mm). */
#define ARM_BASE_HEIGHT      12.0f

/** Shoulder-to-elbow link length (mm). */
#define ARM_LINK1_LENGTH     98.0f

/** Shoulder-to-elbow offset length (mm) (to the right of robot). */
#define ARM_LINK1_OFFSET     18.0f

/** Elbow-to-wrist link length (mm). */
#define ARM_LINK2_LENGTH     140.0f

/** Elbow-to-wrist offset length (mm) (to the left of robot). */
#define ARM_LINK2_OFFSET     21.39f

/** Wrist-to-gripper-tip link length (mm). */
#define ARM_LINK3_LENGTH     91.46f

/** Wrist-to-gripper-tip offset length (mm) (downwards). */
#define ARM_LINK3_OFFSET     26.83f

/* -------------------------------------------------------------------------- */
/* Joint limits (degrees)                                                      */
/* -------------------------------------------------------------------------- */

#define BASE_ROTATION_MIN       50.0f
#define BASE_ROTATION_MAX       130.0f
#define SHOULDER_MIN            75.0f
#define SHOULDER_MAX            150.0f
#define ELBOW_MIN               55.0f
#define ELBOW_MAX               125.0f
#define WRIST_MIN               70.0f
#define WRIST_MAX               110.0f
#define GRIPPER_MIN             100.0f
#define GRIPPER_MAX             140.0f

/* -------------------------------------------------------------------------- */
/* Workspace limits                                                            */
/* -------------------------------------------------------------------------- */

/** Maximum reach of the fully extended arm (mm). */
#define MAX_REACH               (ARM_LINK1_LENGTH + ARM_LINK2_LENGTH)

/** Minimum reach — set to avoid singularity at origin (mm). */
#define MIN_REACH               20.0f

/* -------------------------------------------------------------------------- */
/* Camera-to-arm offset                                                        */
/*                                                                             */
/* Physical offset from the arm base origin to the camera optical centre.      */
/* Measured in the robot body frame (x = forward, y = left, z = up).          */
/* All values in mm.  Set to 0.0f until measured from CAD / hardware.         */
/* -------------------------------------------------------------------------- */

/** Forward (X) distance from arm origin to camera (mm). */
#define CAMERA_OFFSET_FORWARD_MM    0.0f

/** Lateral (Y) distance from arm origin to camera — positive = left (mm). */
#define CAMERA_OFFSET_LATERAL_MM    0.0f

/** Vertical (Z) distance from arm origin to camera — positive = up (mm). */
#define CAMERA_OFFSET_VERTICAL_MM   0.0f

/* -------------------------------------------------------------------------- */
/* Types                                                                       */
/* -------------------------------------------------------------------------- */

/** 3D target position in mm, expressed in the robot body frame. */
typedef struct {
    float x;   /**< Forward axis (mm). */
    float y;   /**< Left axis (mm). */
    float z;   /**< Up axis (mm). */
} ik_target_t;

/** Joint angles that satisfy a given IK target. */
typedef struct {
    float base_rotation;    /**< Base turret rotation (degrees). */
    float shoulder;         /**< Shoulder pitch (degrees). */
    float elbow;            /**< Elbow pitch (degrees). */
    bool  valid;            /**< True if the solution is within joint limits. */
} ik_solution_t;

/* -------------------------------------------------------------------------- */
/* Functions                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the IK solver and log arm geometry.
 *
 * Must be called once before any other IK function.
 *
 * @return ESP_OK always.
 */
esp_err_t xIKSolverInit(void);

/**
 * @brief Compute joint angles for a Cartesian target (inverse kinematics).
 *
 * Solves the 3-DOF arm analytically using the law of cosines.  The input
 * coordinates must be in the robot body frame.  Use @ref xIKCameraToArm
 * first if the coordinates originate from the camera.
 *
 * @param[in]  target   Desired gripper position in the robot body frame (mm).
 * @param[out] solution Resulting joint angles.  @c solution->valid is set to
 *                      false if the target is unreachable.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if target is unreachable or
 *         if either pointer is NULL.
 */
esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution);

/**
 * @brief Check whether a Cartesian target is within the arm's workspace.
 *
 * @param[in] target Position to check in the robot body frame (mm).
 * @return true if the target can be reached, false otherwise.
 */
bool bIKIsReachable(const ik_target_t *target);

/**
 * @brief Compute the gripper position from joint angles (forward kinematics).
 *
 * @param[in]  solution Joint angles to evaluate.
 * @param[out] position Resulting gripper position in the robot body frame (mm).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if either pointer is NULL.
 */
esp_err_t xIKForward(const ik_solution_t *solution, ik_target_t *position);

/**
 * @brief Convert camera-frame coordinates to arm-frame coordinates.
 *
 * Applies the fixed physical offset between the camera optical centre and the
 * arm base origin (@ref CAMERA_OFFSET_FORWARD_MM, @ref CAMERA_OFFSET_LATERAL_MM,
 * @ref CAMERA_OFFSET_VERTICAL_MM).  The result can be passed directly to
 * @ref xIKSolve.
 *
 * Assumes both frames share the same axis orientation (x=forward, y=left,
 * z=up) with no rotational difference between them.
 *
 * @param[in]  camera_coords Object position reported by the camera (mm).
 * @param[out] arm_coords    Equivalent position in the arm body frame (mm).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if either pointer is NULL.
 */
esp_err_t xIKCameraToArm(const ik_target_t *camera_coords, ik_target_t *arm_coords);

#endif /* IK_SOLVER_H */
