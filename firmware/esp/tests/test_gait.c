/**
 * @file test_gait.c
 * @brief Gait generator test - cycles through walking stages
 *
 * Initializes PCA9685 and gait system, then runs through each movement
 * command (neutral, forward, backward, turn left, turn right) with each
 * gait type (tripod, wave, ripple). Logs angles for debugging.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "i2c.h"
#include "gait_generator.h"

#define TAG "TEST_GAIT"

#define TEST_SPEED          0.5f
#define TEST_WALK_DURATION_MS   4000    // how long to run each movement command
#define TEST_NEUTRAL_PAUSE_MS   2000    // pause at neutral between commands

static const char *gait_name(gait_type_t g) {
    switch (g) {
        case GAIT_TRIPOD: return "TRIPOD";
        case GAIT_WAVE:   return "WAVE";
        case GAIT_RIPPLE: return "RIPPLE";
        default:          return "UNKNOWN";
    }
}

static const char *command_name(move_command_t c) {
    switch (c) {
        case MOVE_STOP:       return "STOP";
        case MOVE_FORWARD:    return "FORWARD";
        case MOVE_BACKWARD:   return "BACKWARD";
        case MOVE_TURN_LEFT:  return "TURN_LEFT";
        case MOVE_TURN_RIGHT: return "TURN_RIGHT";
        default:              return "UNKNOWN";
    }
}

static void print_angles(const leg_angles_t *a) {
    printf("  Leg | Hip     | Knee\n");
    printf("  ----+---------+--------\n");
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("   %d  | %6.1f  | %6.1f\n", i, a->hip_angle[i], a->knee_angle[i]);
    }
}

/**
 * @brief Run a single movement command for a set duration, calling xGaitUpdate
 *        at the correct interval and printing angle snapshots.
 */
static esp_err_t run_movement(move_command_t cmd, float speed, uint32_t duration_ms) {
    printf("\n--- %s at speed %.1f for %lu ms ---\n", command_name(cmd), speed, (unsigned long)duration_ms);
    vGaitSetCommand(cmd, speed);

    uint32_t elapsed = 0;
    uint32_t snapshot_interval = duration_ms / 4;  // print 4 snapshots
    uint32_t next_snapshot = snapshot_interval;

    while (elapsed < duration_ms) {
        esp_err_t ret = xGaitUpdate(NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "xGaitUpdate failed: %s", esp_err_to_name(ret));
            return ret;
        }

        elapsed += GAIT_UPDATE_MS;

        if (elapsed >= next_snapshot) {
            printf("\n  [%lu ms] Angle snapshot:\n", (unsigned long)elapsed);
            leg_angles_t angles = xGaitGetAngles();
            print_angles(&angles);
            next_snapshot += snapshot_interval;
        }

        vTaskDelay(pdMS_TO_TICKS(GAIT_UPDATE_MS));
    }

    // stop and return to neutral
    vGaitSetCommand(MOVE_STOP, 0.0f);
    esp_err_t ret = xGaitStandNeutral();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "xGaitStandNeutral failed: %s", esp_err_to_name(ret));
        return ret;
    }

    printf("  Returned to neutral.\n");
    vTaskDelay(pdMS_TO_TICKS(TEST_NEUTRAL_PAUSE_MS));
    return ESP_OK;
}

void test_gait(void) {
    printf("Gait Generator Test\n");
    printf("  Walk duration : %d ms per command\n", TEST_WALK_DURATION_MS);
    printf("  Speed         : %.1f\n", TEST_SPEED);
    printf("  Update rate   : %d ms\n\n", GAIT_UPDATE_MS);

    // init PCA9685
    esp_err_t ret = xPCA9685Init(I2C_MASTER_NUM, PCA9685_BODY_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return;
    }

    // init gait system (sets neutral stance)
    ret = xGaitInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "xGaitInit failed: %s", esp_err_to_name(ret));
        return;
    }
    printf("Gait init OK, standing neutral.\n");
    vTaskDelay(pdMS_TO_TICKS(TEST_NEUTRAL_PAUSE_MS));

    // movement commands to test
    const move_command_t commands[] = {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_TURN_LEFT,
        MOVE_TURN_RIGHT,
    };
    const int num_commands = sizeof(commands) / sizeof(commands[0]);

    // gait types to test
    const gait_type_t gaits[] = {
        GAIT_TRIPOD,
        GAIT_WAVE,
        GAIT_RIPPLE,
    };
    const int num_gaits = sizeof(gaits) / sizeof(gaits[0]);

    for (int g = 0; g < num_gaits; g++) {
        printf("\n========================================\n");
        printf("  GAIT: %s\n", gait_name(gaits[g]));
        printf("========================================\n");

        vGaitSetType(gaits[g]);

        for (int c = 0; c < num_commands; c++) {
            ret = run_movement(commands[c], TEST_SPEED, TEST_WALK_DURATION_MS);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Movement test failed, aborting.");
                return;
            }
        }
    }

    // final neutral
    ret = xGaitStandNeutral();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Final neutral failed: %s", esp_err_to_name(ret));
        return;
    }

    printf("\n========================================\n");
    printf("  Gait test complete!\n");
    printf("========================================\n");
}
