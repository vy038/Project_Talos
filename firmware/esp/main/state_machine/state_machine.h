#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "esp_err.h"

typedef enum {
    STATE_IDLE,
    STATE_WALK,
    STATE_MANIPULATE,
    STATE_ERROR
} robot_state_t;

/**
 * @brief Initialize state machine
 *
 * Sets initial state and transition parameters
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xStateMachineInit(void);

/**
 * @brief Update state machine
 *
 * Evaluates transition conditions and updates current state
 */
void vUpdateState(void);

/**
 * @brief Get current state
 *
 * Returns active state
 *
 * @return robot_state_t Current state
 */
robot_state_t eGetCurrentState(void);

/**
 * @brief Handle idle state
 *
 * Executes idle state behavior
 */
void vHandleIdleState(void);

/**
 * @brief Handle walk state
 *
 * Executes walking state behavior
 */
void vHandleWalkState(void);

/**
 * @brief Handle manipulate state
 *
 * Executes manipulation state behavior
 */
void vHandleManipulateState(void);

/**
 * @brief Handle error state
 *
 * Executes error state behavior and recovery
 */
void vHandleErrorState(void);

#endif