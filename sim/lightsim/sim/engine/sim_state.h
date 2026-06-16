/**
 * @file sim_state.h
 * @brief Central simulation state tracking and JSON emission
 *
 * Collects servo positions, IMU state, I2C logs, and other telemetry
 * from HAL stubs and emits JSON-lines to stdout for the bridge server.
 */

#ifndef SIM_STATE_H
#define SIM_STATE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  State tracking                                                             */
/* ========================================================================== */

/** Set a servo angle in the state tracker */
void sim_set_servo(int board, uint8_t channel, float angle, uint16_t pulse_us);

/** Log an I2C transaction for the debug view */
void sim_log_i2c(const char *op, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);

/** Get/set IMU override values (called by mpu6050_stub) */
void sim_get_imu_override(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz);
void sim_set_imu_override(float ax, float ay, float az,
                          float gx, float gy, float gz);

/** Get/set simulated VL53L0X ToF distance in mm (called by vl53l0x_stub) */
uint16_t sim_get_tof_distance_mm(void);
void     sim_set_tof_distance_mm(uint16_t mm);

/** Register a callback that returns the current robot state (int).
 *  Called from main_sim.c after firmware starts. sim_state.c never
 *  includes any firmware headers — main_sim.c is the bridge. */
void sim_register_state_getter(int (*getter)(void));

/** Fallback: set robot state directly (used if no getter registered) */
void sim_set_robot_state(int state);

/* ========================================================================== */
/*  JSON emission                                                              */
/* ========================================================================== */

/** Emit the current full state as a JSON line to stdout */
void sim_emit_state(void);

/** Initialize the state system */
void sim_state_init(void);

/** Enable/disable JSON output on stdout */
void sim_enable_json_output(bool enable);

/** Check if JSON output is enabled */
bool sim_json_enabled(void);

/** Called from vTaskDelay shim to periodically emit state */
void sim_on_tick(uint32_t delay_ms);

/** Set/check exit flag — vTaskDelay calls this to terminate cleanly */
void sim_request_exit(void);
bool sim_exit_requested(void);

/** Backend simulation speed multiplier (scales vTaskDelay's real-time sleep) */
void sim_set_time_scale(float scale);
float sim_get_time_scale(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_STATE_H */
