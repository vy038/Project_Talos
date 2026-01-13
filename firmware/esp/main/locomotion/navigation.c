#include "navigation.h"

esp_err_t xNavigationInit(void) {
    return ESP_OK;
}

void vReadDistanceSensors(float distances[NUM_DISTANCE_SENSORS]) {
}

bool bDetectObstacles(float distances[NUM_DISTANCE_SENSORS]) {
    return false;
}

void vCalculateAvoidance(float distances[NUM_DISTANCE_SENSORS], float *velocity_adjust, float *direction_adjust) {
}

void vUpdateTargetVelocity(float *forward, float *turn) {
}