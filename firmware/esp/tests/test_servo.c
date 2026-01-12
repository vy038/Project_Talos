// test servo
#include "servo.h"

void test_servo(uint8_t addr, uint8_t channel) {
    printf("\nServo Test (addr=0x%02X, ch=%d):\n", addr, channel);
    
    xPCA9685Init(I2C_NUM_0, addr, 50);
    
    // Sweep 0 to 180
    for (int angle = 0; angle <= 180; angle += 30) {
        xPCA9685SetAngle(I2C_NUM_0, addr, channel, angle);
        printf("Angle: %d\n", angle);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}