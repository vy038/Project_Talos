#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float base;
    float shoulder;
    float elbow;
    float gripper;
} arm_angles_t;

typedef enum {
    ARM_IDLE,
    ARM_MOVING,
    ARM_ERROR
} arm_state_t;

/**
 * @brief Initialize arm control system
 *
 * Sets up necessary peripherals, initializes state, and prepares for motion commands.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmControlInit(void);

/**
 * @brief Set target angles for the arm joints
 *
 * This function updates the desired angles for each joint. The actual movement towards these angles will be handled by the update function.
 *
 * @param angles Pointer to arm_angles_t structure containing target angles in degrees (0-180)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmSetAngles(const arm_angles_t *angles);

/**
 * @brief Set angle for a single joint with retry logic
 *
 * This function attempts to set the angle for a specific joint, retrying on failure and performing I2C bus recovery if necessary.
 *
 * @param channel Servo channel (e.g., ARM_BASE_CH)
 * @param angle Desired angle in degrees (0-180)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmSetAngleWithRetry(uint8_t channel, uint8_t angle);

/**
 * @brief Update arm joint angles towards target
 *
 * This function should be called periodically (e.g., in a timer or main loop) to smoothly move the arm towards the target angles. It will handle incremental updates and state transitions.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmUpdate(void);

/**
 * @brief Set gripper angle
 *
 * Convenience function to set just the gripper angle without affecting other joints.
 *
 * @param angle Desired gripper angle in degrees (0-180)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmGripper(float angle);

/**
 * @brief Get current state of the arm
 *
 * @return arm_state_t Current state (ARM_IDLE, ARM_MOVING, ARM_ERROR)
 */
arm_state_t xArmGetState(void);

/**
 * @brief Check if arm has reached target angles
 *
 * Compares current angles to target angles and returns true if all joints are within a defined tolerance.
 *
 * @return true if arm is at target, false otherwise
 */
bool bArmAtTarget(void);

#endif
