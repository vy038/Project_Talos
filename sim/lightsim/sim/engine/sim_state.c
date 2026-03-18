/**
 * @file sim_state.c
 * @brief Central simulation state - tracks servos, IMU, I2C, emits JSON
 */

#include "sim_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ========================================================================== */
/*  JSON output control                                                        */
/* ========================================================================== */

static bool json_output_enabled = false;
static uint32_t emit_accumulator_ms = 0;
#define EMIT_INTERVAL_MS 20  /* Match firmware's 50Hz update rate */

void sim_enable_json_output(bool enable) {
    json_output_enabled = enable;
}

bool sim_json_enabled(void) {
    return json_output_enabled;
}

static volatile bool exit_requested = false;

void sim_request_exit(void) {
    exit_requested = true;
}

bool sim_exit_requested(void) {
    return exit_requested;
}

void sim_on_tick(uint32_t delay_ms) {
    /* Check if we should terminate (SIGTERM received) */
    if (exit_requested) {
        fprintf(stderr, "\n[SIM] Exit requested — terminating.\n");
        fflush(stderr);
        fflush(stdout);
        exit(0);
    }

    if (!json_output_enabled) return;
    emit_accumulator_ms += delay_ms;
    if (emit_accumulator_ms >= EMIT_INTERVAL_MS) {
        emit_accumulator_ms = 0;
        sim_emit_state();
    }
}

/* ========================================================================== */
/*  Servo state                                                                */
/* ========================================================================== */

typedef struct {
    float angle;
    uint16_t pulse_us;
    bool dirty;
} servo_state_t;

/* 2 boards x 16 channels */
/* Arm channels (board 0, ch 6-9) default to 90° neutral since xArmControlInit
   is not called in all test configurations (e.g. TEST_GAIT). */
static servo_state_t servos[2][16] = {
    [0][6] = {90.0f, 0, false},
    [0][7] = {90.0f, 0, false},
    [0][8] = {90.0f, 0, false},
    [0][9] = {90.0f, 0, false},
};

void sim_set_servo(int board, uint8_t channel, float angle, uint16_t pulse_us) {
    if (board < 0 || board > 1 || channel > 15) return;
    servos[board][channel].angle = angle;
    servos[board][channel].pulse_us = pulse_us;
    servos[board][channel].dirty = true;
}

/* ========================================================================== */
/*  I2C log (ring buffer)                                                      */
/* ========================================================================== */

#define I2C_LOG_SIZE 64

typedef struct {
    int64_t timestamp_us;
    char op[10];
    uint8_t addr;
    uint8_t reg;
    uint8_t data[8];
    size_t data_len;
} i2c_entry_t;

static i2c_entry_t i2c_log[I2C_LOG_SIZE];
static int i2c_log_head = 0;
static int i2c_log_count = 0;

static int64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
}

void sim_log_i2c(const char *op, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    i2c_entry_t *entry = &i2c_log[i2c_log_head];
    entry->timestamp_us = get_time_us();
    strncpy(entry->op, op, sizeof(entry->op) - 1);
    entry->op[sizeof(entry->op) - 1] = '\0';
    entry->addr = addr;
    entry->reg = reg;
    entry->data_len = (len > 8) ? 8 : len;
    if (data && entry->data_len > 0) {
        memcpy(entry->data, data, entry->data_len);
    }
    i2c_log_head = (i2c_log_head + 1) % I2C_LOG_SIZE;
    if (i2c_log_count < I2C_LOG_SIZE) i2c_log_count++;
}

/* ========================================================================== */
/*  IMU override                                                               */
/* ========================================================================== */

static float imu_ax = 0.0f, imu_ay = 0.0f, imu_az = -9.81f;
static float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;

void sim_get_imu_override(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz) {
    *ax = imu_ax; *ay = imu_ay; *az = imu_az;
    *gx = imu_gx; *gy = imu_gy; *gz = imu_gz;
}

void sim_set_imu_override(float ax, float ay, float az,
                          float gx, float gy, float gz) {
    imu_ax = ax; imu_ay = ay; imu_az = az;
    imu_gx = gx; imu_gy = gy; imu_gz = gz;
}

/* ========================================================================== */
/*  Init                                                                       */
/* ========================================================================== */

void sim_state_init(void) {
    memset(servos, 0, sizeof(servos));
    for (int b = 0; b < 2; b++) {
        for (int c = 0; c < 16; c++) {
            servos[b][c].angle = 90.0f;
            servos[b][c].pulse_us = 1500;
        }
    }
}

/* ========================================================================== */
/*  JSON emission                                                              */
/* ========================================================================== */

void sim_emit_state(void) {
    /* Emit JSON line to stdout for the bridge server */
    printf("{\"type\":\"state_update\",\"timestamp_us\":%lld,", (long long)get_time_us());

    /* Body servos (board 0) */
    printf("\"body_servos\":[");
    for (int i = 0; i < 16; i++) {
        printf("%.1f%s", servos[0][i].angle, i < 15 ? "," : "");
    }
    printf("],");

    /* Arm servos (board 1) */
    printf("\"arm_servos\":[");
    for (int i = 0; i < 16; i++) {
        printf("%.1f%s", servos[1][i].angle, i < 15 ? "," : "");
    }
    printf("],");

    /* Leg angles (decoded from body servo channels) */
    static const int hip_ch[6]  = {15, 14, 13, 2, 1, 0};
    static const int knee_ch[6] = {12, 11, 10, 5, 4, 3};

    printf("\"legs\":{\"hip\":[");
    for (int i = 0; i < 6; i++) {
        printf("%.1f%s", servos[0][hip_ch[i]].angle, i < 5 ? "," : "");
    }
    printf("],\"knee\":[");
    for (int i = 0; i < 6; i++) {
        printf("%.1f%s", servos[0][knee_ch[i]].angle, i < 5 ? "," : "");
    }
    printf("]},");

    /* Arm angles (channels 6-9 on board 0, since ARM shares BODY addr) */
    printf("\"arm\":{\"base\":%.1f,\"shoulder\":%.1f,\"elbow\":%.1f,\"gripper\":%.1f},",
           servos[0][6].angle, servos[0][7].angle,
           servos[0][8].angle, servos[0][9].angle);

    /* IMU */
    printf("\"imu\":{\"accel\":[%.3f,%.3f,%.3f],\"gyro\":[%.3f,%.3f,%.3f]},",
           imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz);

    /* Recent I2C log (last 8 entries) */
    printf("\"i2c_log\":[");
    int start = (i2c_log_count < 8) ? 0 : i2c_log_head - 8;
    if (start < 0) start += I2C_LOG_SIZE;
    int n = (i2c_log_count < 8) ? i2c_log_count : 8;
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % I2C_LOG_SIZE;
        i2c_entry_t *e = &i2c_log[idx];
        printf("{\"t\":%lld,\"op\":\"%s\",\"addr\":\"0x%02X\",\"reg\":\"0x%02X\"",
               (long long)e->timestamp_us, e->op, e->addr, e->reg);
        if (e->data_len > 0) {
            printf(",\"data\":\"");
            for (size_t j = 0; j < e->data_len; j++) {
                printf("%02X%s", e->data[j], j < e->data_len - 1 ? " " : "");
            }
            printf("\"");
        }
        printf("}%s", i < n - 1 ? "," : "");
    }
    printf("]}\n");

    fflush(stdout);
}
