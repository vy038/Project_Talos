/**
 * @file test_harness.h
 * @brief Minimal unit test harness for host-side testing.
 *        No dependencies beyond libc.
 */
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static int _tests_run = 0;
static int _tests_passed = 0;
static int _tests_failed = 0;

#define TEST(name)                                          \
    static void test_##name(void);                          \
    static void run_test_##name(void) {                     \
        _tests_run++;                                       \
        printf("  [RUN ] %s\n", #name);                     \
        test_##name();                                      \
    }                                                       \
    static void test_##name(void)

#define ASSERT_TRUE(expr) do {                              \
    if (!(expr)) {                                          \
        printf("  [FAIL] %s:%d: %s\n",                      \
               __FILE__, __LINE__, #expr);                  \
        _tests_failed++; return;                            \
    }                                                       \
} while(0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_EQ(a, b) do {                                \
    if ((a) != (b)) {                                       \
        printf("  [FAIL] %s:%d: %s == %s (%d != %d)\n",    \
               __FILE__, __LINE__, #a, #b, (int)(a), (int)(b)); \
        _tests_failed++; return;                            \
    }                                                       \
} while(0)

#define ASSERT_NEAR(a, b, tol) do {                         \
    if (fabs((double)(a) - (double)(b)) > (double)(tol)) {  \
        printf("  [FAIL] %s:%d: |%s - %s| > %s "           \
               "(%.6f vs %.6f)\n",                          \
               __FILE__, __LINE__, #a, #b, #tol,            \
               (double)(a), (double)(b));                   \
        _tests_failed++; return;                            \
    }                                                       \
} while(0)

#define RUN_TEST(name) run_test_##name()

#define TEST_REPORT() do {                                  \
    _tests_passed = _tests_run - _tests_failed;             \
    printf("\n  %d/%d tests passed",                        \
           _tests_passed, _tests_run);                      \
    if (_tests_failed > 0)                                  \
        printf(", %d FAILED", _tests_failed);               \
    printf("\n");                                            \
    return _tests_failed > 0 ? 1 : 0;                      \
} while(0)

#endif
