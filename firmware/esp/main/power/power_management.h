/**
 * @file power_management.h
 * @brief Current monitoring and emergency protection via ACS712
 *
 * Reads ACS712 current sensor periodically. Two threat levels:
 *   WARNING:   high current but not dangerous. reduce speed. maybe stalled servo.
 *   EMERGENCY: dangerously high or sustained. kill servo power immediately.
 *
 * Uses consecutive-count filtering to ignore servo inrush spikes (~50ms, 3-5x
 * running current). Requiring 2-3 consecutive high readings at 20ms intervals
 * catches real problems while ignoring transients.
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "esp_err.h"
#include <stdbool.h>

// TODO: measure idle current and tune
#define POWER_NORMAL_AMPS       1.5f    // robot standing still, servos powered
#define POWER_WARNING_AMPS      4.0f    // heavy activity. 2-3x idle typical.
#define POWER_EMERGENCY_AMPS    7.0f    // set below BMS cutoff. if BMS=10A, set 7A.

#define POWER_WARNING_COUNT     3       // consecutive readings before warning
#define POWER_EMERGENCY_COUNT   2       // react faster for emergencies
#define POWER_POLL_MS           20

typedef enum {
    POWER_OK,
    POWER_WARNING,
    POWER_EMERGENCY,
} power_status_t;

typedef struct {
    float current_amps;
    float peak_amps;
    power_status_t status;
    uint32_t warning_count;
    uint32_t emergency_count;
} power_info_t;

typedef void (*power_event_cb_t)(power_status_t status, float current_amps);

/**
 * @brief Enable or disable power monitoring at runtime.
 *
 * Disabled by default. Enable only when ACS712 is physically connected
 * a floating ADC pin produces garbage readings that trigger false emergencies.
 */
void vPowerSetEnabled(bool enabled);

/**
 * @brief Init power monitoring. Call after ACS712 init.
 * 
 * Initializes internal state and takes a baseline current reading. Must be called
 * 
 * @return ESP_OK if successful, ESP_ERR_INVALID_STATE if ACS712 not ready, other error codes for I2C read failures
 */
esp_err_t xPowerInit(void);

/**
 * @brief Register callback for power events. Keep it short (set flag or post queue).
 * 
 * Callback is triggered on status changes after consecutive counts. 
 */
void vPowerSetCallback(power_event_cb_t callback);

/**
 * @brief Read current once, update status. Call from task or polling loop.
 * 
 * Reads current from ACS712, updates internal state and status with consecutive count filtering. Triggers callback if status changes.
 * 
 * @return power_status_t current status after reading. Note: status only changes after consecutive counts, so may not reflect a single high reading immediately.
 */
power_status_t xPowerCheck(void);

/**
 * @brief Get current power info struct with stats.
 * 
 * Includes current amps, peak amps since last reset, current status, and event counts. Note: current_amps is updated on each check, but peak_amps and counts persist until reset.
 * 
 * @return power_info_t current info snapshot. Note: current_amps is updated on each check, but peak_amps and counts persist until reset.
 */
power_info_t xPowerGetInfo(void);

/**
 * @brief Reset peak current and event counts, but keep current reading and status for continuity.
 * 
 * Useful for clearing stats after an event or at the start of a new activity, while maintaining awareness of the current state. Note: does not change current_amps or status, just resets peak_amps and counts.
 */
void vPowerResetStats(void);

/**
 * @brief Get the most recent current reading in amps. Updated on each check.
 * 
 * Provides a quick way to get the current amps without needing the full info struct. Note: may not reflect a single high reading immediately due to consecutive count filtering.
 * 
 * @return float current amps. Note: may not reflect a single high reading immediately due to consecutive count filtering.
 */
float fPowerGetCurrent(void);

#endif