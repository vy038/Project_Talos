// ik_solver.h
#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Link lengths in mm - measure from CAD model
#define ARM_BASE_ANGLE       0.0f  // Base mounting angle in degrees (0 = horizontal, 90 = vertical)
#define ARM_BASE_HEIGHT      0.0f  // Vertical distance from origin to shoulder pivot (mm)
#define ARM_LINK1_LENGTH     0.0f  // Shoulder to elbow
#define ARM_LINK2_LENGTH     0.0f  // Elbow to gripper tip

// Joint limits in degrees - get from servo datasheets and mechanical stops
#define BASE_ROTATION_MIN       0.0f
#define BASE_ROTATION_MAX       180.0f
#define SHOULDER_MIN            0.0f
#define SHOULDER_MAX            180.0f
#define ELBOW_MIN               0.0f
#define ELBOW_MAX               180.0f

// Workspace limits
#define MAX_REACH               (ARM_LINK1_LENGTH + ARM_LINK2_LENGTH)
#define MIN_REACH               20.0f

typedef struct {
    float x;        // Target position in mm (robot frame: x=forward)
    float y;        // Target position in mm (robot frame: y=left)
    float z;        // Target position in mm (robot frame: z=up)
} ik_target_t;

typedef struct {
    float base_rotation;    // Base turret rotation
    float shoulder;         // Shoulder pitch
    float elbow;            // Elbow pitch
    bool valid;             // Whether solution is valid
} ik_solution_t;

esp_err_t xIKSolverInit(void);

esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution);

bool bIKIsReachable(const ik_target_t *target);

esp_err_t xIKForward(const ik_solution_t *solution, ik_target_t *position);

#endif
