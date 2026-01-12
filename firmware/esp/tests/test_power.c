// test power
#include "power_monitor.h"

void test_power_monitor(void) {
    printf("\nPower Monitor Test:\n");
    
    xACS712Init();
    
    for (int i = 0; i < 10; i++) {
        float current;
        xACS712ReadCurrent(&current);
        printf("Current: %.3f A\n", current);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}