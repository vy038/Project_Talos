#include "leg_controller.h"

esp_err_t xLegControllerInit(void) {
    return ESP_OK;
}

esp_err_t xLegIk3Dof(int leg_index, float x, float y, float z, float *hip, float *knee, float *ankle) {
    return ESP_OK;
}

bool bValidateReachable(int leg_index, float x, float y, float z) {
    return true;
}

void vHipToWorldTransform(int leg_index, float local[3], float world[3]) {
}

void vApplyServoOffsets(int leg_index, float angles[LEG_DOF], float corrected[LEG_DOF]) {
}