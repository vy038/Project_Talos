// test_vl53l0x.c
#include "vl53l0x.h"

void test_vl53l0x(uint8_t scl, uint8_t sda) {
    printf("\nVL53L0X Test:\n");
    
    // Configure: (port, scl, sda, xshut, address, io_2v8)
    vl53l0x_t *dev = vl53l0x_config(I2C_NUM_0, scl, sda, -1, 0x29, 1);
    if (!dev) {
        printf("Config failed\n");
        return;
    }
    
    const char *err = vl53l0x_init(dev);
    if (err) {
        printf("Init failed: %s\n", err);
        return;
    }
    
    for (int i = 0; i < 10; i++) {
        uint16_t distance = vl53l0x_readRangeSingleMillimeters(dev);
        if (vl53l0x_timeoutOccurred(dev)) {
            printf("Timeout\n");
        } else {
            printf("Distance: %d mm\n", distance);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vl53l0x_end(dev);
}