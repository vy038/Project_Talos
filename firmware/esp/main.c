#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"  // Your I2C driver header

void app_main(void) {
    printf("Starting I2C Scanner Test...\n");
    
    // Initialize I2C (you'll need this function in i2c.c)
    i2c_master_init();
    
    // Run the scan
    i2c_scan();
    
    printf("Scan complete!\n");
}