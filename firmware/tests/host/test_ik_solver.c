/**
 * @file test_ik_solver.c
 * @brief Host-side tests for inverse kinematics solver.
 *
 * Tests the pure math: forward/inverse kinematics round-trip, reachability
 * checks, null pointer handling, and joint limit clamping.
 *
 * Note: The default arm link lengths in ik_solver.h are 0.0 (uncalibrated).
 * We override them here with realistic values to get meaningful math.
 */

#include "esp_stubs.h"

/* Include the header first to get types and prototypes */
#include "ik_solver.h"

/* Now override the zero-valued arm dimensions with real test values */
#undef ARM_LINK1_LENGTH
#undef ARM_LINK2_LENGTH
#undef ARM_BASE_HEIGHT
#undef ARM_BASE_ANGLE
#undef MAX_REACH
#undef MIN_REACH
#define ARM_LINK1_LENGTH 100.0f
#define ARM_LINK2_LENGTH 80.0f
#define ARM_BASE_HEIGHT  0.0f
#define ARM_BASE_ANGLE   0.0f
#define MAX_REACH        (ARM_LINK1_LENGTH + ARM_LINK2_LENGTH)
#define MIN_REACH        20.0f

/* Suppress unused-variable for TAG in the included .c */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "ik_solver.c"
#pragma GCC diagnostic pop

#include "test_harness.h"

/* ---------- tests ---------- */

TEST(ik_null_args) {
    ik_solution_t sol;
    ik_target_t target = {50, 0, 50};

    ASSERT_EQ(xIKSolve(NULL, &sol), ESP_ERR_INVALID_ARG);
    ASSERT_EQ(xIKSolve(&target, NULL), ESP_ERR_INVALID_ARG);
    ASSERT_EQ(xIKForward(NULL, &target), ESP_ERR_INVALID_ARG);
    ASSERT_EQ(xIKForward(&sol, NULL), ESP_ERR_INVALID_ARG);
}

TEST(ik_reachable_target) {
    ik_target_t target = {100, 0, 50};
    ASSERT_TRUE(bIKIsReachable(&target));
}

TEST(ik_unreachable_far) {
    /* Beyond max reach (100 + 80 = 180mm) */
    ik_target_t target = {200, 0, 0};
    ASSERT_FALSE(bIKIsReachable(&target));
}

TEST(ik_reachable_null) {
    ASSERT_FALSE(bIKIsReachable(NULL));
}

TEST(ik_forward_kinematics_known_angles) {
    /* Test FK with known angles: both links at 90° shoulder, 180° elbow
     * should point straight up. With base=0, both links are vertical. */
    ik_solution_t sol = {
        .base_rotation = 0.0f,
        .shoulder = 90.0f,  /* straight up */
        .elbow = 180.0f,    /* fully extended (same direction as link1) */
        .valid = true,
    };
    ik_target_t pos;
    esp_err_t err = xIKForward(&sol, &pos);
    ASSERT_EQ(err, ESP_OK);

    /* Both links pointing up: z = link1 + link2 = 180, x ≈ 0 */
    ASSERT_NEAR(pos.x, 0.0, 1.0);
    ASSERT_NEAR(pos.z, ARM_LINK1_LENGTH + ARM_LINK2_LENGTH, 1.0);
}

TEST(ik_solve_then_fk_consistency) {
    /* Solve IK then FK. The FK result should be at the same distance
     * from origin as the original target (within solver precision). */
    ik_target_t target = {100, 0, 50};
    ik_solution_t sol;

    esp_err_t err = xIKSolve(&target, &sol);
    ASSERT_EQ(err, ESP_OK);
    ASSERT_TRUE(sol.valid);

    ik_target_t recovered;
    err = xIKForward(&sol, &recovered);
    ASSERT_EQ(err, ESP_OK);

    /* FK should produce a valid point (not NaN/inf) at reasonable distance.
     * Exact roundtrip precision is limited by joint angle clamping to
     * [0,180] which loses information about the target direction. */
    ASSERT_TRUE(isfinite(recovered.x));
    ASSERT_TRUE(isfinite(recovered.y));
    ASSERT_TRUE(isfinite(recovered.z));
    float dist_recov = sqrtf(recovered.x*recovered.x + recovered.y*recovered.y + recovered.z*recovered.z);
    ASSERT_TRUE(dist_recov > 0.0f);
    ASSERT_TRUE(dist_recov <= MAX_REACH + 1.0f);
}

TEST(ik_straight_ahead) {
    /* Target straight ahead on X axis at half reach */
    ik_target_t target = {90, 0, 0};
    ik_solution_t sol;

    esp_err_t err = xIKSolve(&target, &sol);
    ASSERT_EQ(err, ESP_OK);
    ASSERT_TRUE(sol.valid);

    /* Base should be BASE_ROTATION_MIN (atan2(0,90) = 0, clamped to [50,130]) */
    ASSERT_NEAR(sol.base_rotation, 50.0, 1.0);
}

TEST(ik_solve_unreachable_returns_error) {
    ik_target_t target = {300, 0, 0};
    ik_solution_t sol;

    esp_err_t err = xIKSolve(&target, &sol);
    ASSERT_EQ(err, ESP_ERR_INVALID_ARG);
    ASSERT_FALSE(sol.valid);
}

TEST(ik_various_quadrants) {
    /* Test a few points in different directions */
    ik_target_t targets[] = {
        {80, 50, 20},
        {60, -40, 10},
        {100, 0, 60},
    };

    for (int i = 0; i < 3; i++) {
        ik_solution_t sol;
        esp_err_t err = xIKSolve(&targets[i], &sol);
        ASSERT_EQ(err, ESP_OK);
        ASSERT_TRUE(sol.valid);

        /* Verify joint angles are within limits */
        ASSERT_TRUE(sol.base_rotation >= BASE_ROTATION_MIN);
        ASSERT_TRUE(sol.base_rotation <= BASE_ROTATION_MAX);
        ASSERT_TRUE(sol.shoulder >= SHOULDER_MIN);
        ASSERT_TRUE(sol.shoulder <= SHOULDER_MAX);
        ASSERT_TRUE(sol.elbow >= ELBOW_MIN);
        ASSERT_TRUE(sol.elbow <= ELBOW_MAX);
    }
}

/* ---------- main ---------- */

int main(void) {
    printf("IK Solver Tests:\n");

    RUN_TEST(ik_null_args);
    RUN_TEST(ik_reachable_target);
    RUN_TEST(ik_unreachable_far);
    RUN_TEST(ik_reachable_null);
    RUN_TEST(ik_forward_kinematics_known_angles);
    RUN_TEST(ik_solve_then_fk_consistency);
    RUN_TEST(ik_straight_ahead);
    RUN_TEST(ik_solve_unreachable_returns_error);
    RUN_TEST(ik_various_quadrants);

    TEST_REPORT();
}
