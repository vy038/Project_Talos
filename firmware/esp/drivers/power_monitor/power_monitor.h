#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "esp_err.h"
#include <stdint.h>

// TODO: Initialize ACS712 current sensor
esp_err_t power_monitor_init(void);

// TODO: Read current from ACS712 (returns Amps)
float power_monitor_read_current(void);

#endif
