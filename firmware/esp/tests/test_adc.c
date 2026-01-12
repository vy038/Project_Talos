// test adc
#include "adc_helpers.h"

void adc_test(adc_channel_t channel) {
    printf("\nADC Test :\n");

    printf(iAnalogReadRaw(channel));
    printf("\n");
    printf(fAnalogReadVoltage(channel));
    printf("\n");
    printf(fAnalogReadVoltageAvg(channel, 20));
    printf("\n");
}