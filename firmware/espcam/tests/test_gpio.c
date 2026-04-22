/**
 * @file test_gpio.c
 * @brief GPIO 40/41 toggle test
 *
 * Toggles GPIO 40 and 41 to verify they respond to output changes.
 * Use multimeter to probe the pins and verify voltage swings 0V <-> 3.3V.
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void test_gpio(void) {
    printf("\n========================================\n");
    printf("  GPIO 40/41 Toggle Test\n");
    printf("========================================\n\n");

    printf("Configuring GPIO 40 and 41 as outputs...\n");
    gpio_set_direction(GPIO_NUM_40, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_41, GPIO_MODE_OUTPUT);
    printf("GPIO configured.\n\n");

    printf("Probe GPIO 40 and 41 with multimeter.\n");
    printf("Voltage should toggle between 0V and 3.3V every 500ms.\n\n");

    for (int cycle = 0; cycle < 10; cycle++) {
        // HIGH
        gpio_set_level(GPIO_NUM_40, 1);
        gpio_set_level(GPIO_NUM_41, 1);
        printf("[%d] HIGH  - GPIO40=3.3V, GPIO41=3.3V\n", cycle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // LOW
        gpio_set_level(GPIO_NUM_40, 0);
        gpio_set_level(GPIO_NUM_41, 0);
        printf("[%d] LOW   - GPIO40=0V,   GPIO41=0V\n", cycle);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    printf("\nTest complete.\n");
}
