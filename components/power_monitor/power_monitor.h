#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

// TODO: Initialize ACS712 current sensor
void power_monitor_init(void);

// TODO: Read current from ACS712 (returns Amps)
float power_monitor_read_current(void);

#endif
