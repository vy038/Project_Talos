// test vl53l0x
#include "vl53l0x.h"

void test_vl53l0x(void) {
    printf("\nVL53L0X Test:\n");
    
    VL53L0X_Dev_t dev;
    vl53l0x_init(&dev);
    
    for (int i = 0; i < 10; i++) {
        uint16_t distance;
        uint8_t status;
        vl53l0x_read_range(&dev, &distance, &status);
        printf("Distance: %d mm (status=%d)\n", distance, status);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}