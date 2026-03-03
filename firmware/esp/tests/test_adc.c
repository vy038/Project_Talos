/**
 * @file test_adc.c
 * @brief ADC channel test - reads raw, voltage, and averaged voltage
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "adc_helpers.h"

#define TAG "TEST_ADC"

#define ADC_AVG_SAMPLES 20

void test_adc(void) {
    printf("ADC Channel Test\n\n");

    printf("Battery voltage (GPIO 34, CH6):\n");
    int raw = iAnalogReadRaw(VBAT_ADC_CHANNEL);
    float voltage = fAnalogReadVoltage(VBAT_ADC_CHANNEL);
    float avg = fAnalogReadVoltageAvg(VBAT_ADC_CHANNEL, ADC_AVG_SAMPLES);
    printf("  Raw     : %d\n", raw);
    printf("  Voltage : %.3f V\n", voltage);
    printf("  Avg (%2d): %.3f V\n\n", ADC_AVG_SAMPLES, avg);

    printf("Current sensor (GPIO 35, CH7):\n");
    raw = iAnalogReadRaw(CURRENT_ADC_CHANNEL);
    voltage = fAnalogReadVoltage(CURRENT_ADC_CHANNEL);
    avg = fAnalogReadVoltageAvg(CURRENT_ADC_CHANNEL, ADC_AVG_SAMPLES);
    printf("  Raw     : %d\n", raw);
    printf("  Voltage : %.3f V\n", voltage);
    printf("  Avg (%2d): %.3f V\n", ADC_AVG_SAMPLES, avg);
}
