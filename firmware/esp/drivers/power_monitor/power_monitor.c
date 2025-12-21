#include "power_monitor.h"
#include "driver/adc.h"
#include <stdio.h>

void power_monitor_init(void) {
    printf("TODO: Initialize power monitor\n");
    // TODO: Configure ADC on GPIO 34
    // TODO: Set up ACS712 calibration
}

float power_monitor_read_current(void) {
    // TODO: Read ADC value
    // TODO: Convert to voltage (0-3.3V)
    // TODO: Apply ACS712 formula: current = (voltage - 2.5V) / 0.066
    return 0.0f;
}
