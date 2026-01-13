#include "state_machine.h"

esp_err_t xStateMachineInit(void) {
    return ESP_OK;
}

void vUpdateState(void) {
}

robot_state_t eGetCurrentState(void) {
    return STATE_IDLE;
}

void vHandleIdleState(void) {
}

void vHandleWalkState(void) {
}

void vHandleManipulateState(void) {
}

void vHandleErrorState(void) {
}