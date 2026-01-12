// test mpu
#include "mpu6050.h"

void test_mpu6050(void) {
    printf("\nMPU6050 Test:\n");
    
    xMPU6050_init();
    xMPU6050_calibrate();
    
    mpu6050_data_t data;
    for (int i = 0; i < 10; i++) {
        xMPU6050_read(&data);
        printf("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f\n",
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}