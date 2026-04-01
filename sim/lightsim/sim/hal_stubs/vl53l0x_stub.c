/**
 * @file vl53l0x_stub.c
 * @brief Virtual VL53L0X ToF sensor replacing espcam/components/VL53L0X/vl53l0x.c
 *
 * Returns a configurable simulated distance. The bridge server sets
 * sim_set_tof_distance() from the ball position computed in the JS frontend
 * (same pinhole-derived distance used by viewer3d.js).
 *
 * The real vl53l0x driver uses its own internal I2C setup (vl53l0x_config).
 * All register-level calls are no-ops here.
 */

#include "vl53l0x.h"
#include "sim_state.h"
#include <stdlib.h>
#include <stdio.h>

/* Internal device struct — real driver allocates one, we just return a dummy */
struct vl53l0x_s {
    uint8_t  address;
    int      timeout_occurred;
};

static struct vl53l0x_s sim_dev = { .address = 0x29, .timeout_occurred = 0 };

/* ========================================================================== */
/*  Lifecycle                                                                 */
/* ========================================================================== */

vl53l0x_t *vl53l0x_config(int8_t port, int8_t scl, int8_t sda,
                            int8_t xshut, uint8_t address, uint8_t io_2v8)
{
    (void)port; (void)scl; (void)sda; (void)xshut; (void)io_2v8;
    sim_dev.address = address;
    fprintf(stderr, "I (VL53L0X_SIM) Virtual VL53L0X configured at 0x%02X\n", address);
    return &sim_dev;
}

const char *vl53l0x_init(vl53l0x_t *v) {
    (void)v;
    fprintf(stderr, "I (VL53L0X_SIM) Virtual VL53L0X initialized (continuous mode ready)\n");
    return NULL; /* NULL = OK in this driver's convention */
}

void vl53l0x_end(vl53l0x_t *v) { (void)v; }

/* ========================================================================== */
/*  Address / register stubs (no-ops)                                        */
/* ========================================================================== */

void     vl53l0x_setAddress(vl53l0x_t *v, uint8_t a) { if (v) v->address = a; }
uint8_t  vl53l0x_getAddress(vl53l0x_t *v)             { return v ? v->address : 0x29; }

void     vl53l0x_writeReg8Bit (vl53l0x_t *v, uint8_t r, uint8_t val)   { (void)v;(void)r;(void)val; }
void     vl53l0x_writeReg16Bit(vl53l0x_t *v, uint8_t r, uint16_t val)  { (void)v;(void)r;(void)val; }
void     vl53l0x_writeReg32Bit(vl53l0x_t *v, uint8_t r, uint32_t val)  { (void)v;(void)r;(void)val; }
uint8_t  vl53l0x_readReg8Bit  (vl53l0x_t *v, uint8_t r) { (void)v;(void)r; return 0; }
uint16_t vl53l0x_readReg16Bit (vl53l0x_t *v, uint8_t r) { (void)v;(void)r; return 0; }
uint32_t vl53l0x_readReg32Bit (vl53l0x_t *v, uint8_t r) { (void)v;(void)r; return 0; }
void     vl53l0x_writeMulti   (vl53l0x_t *v, uint8_t r, const uint8_t *src, uint8_t n) { (void)v;(void)r;(void)src;(void)n; }
void     vl53l0x_readMulti    (vl53l0x_t *v, uint8_t r, uint8_t *dst,       uint8_t n) { (void)v;(void)r; if(dst) { for(int i=0;i<n;i++) dst[i]=0; } }

/* ========================================================================== */
/*  Timing / signal rate stubs                                               */
/* ========================================================================== */

const char *vl53l0x_setSignalRateLimit(vl53l0x_t *v, float l)   { (void)v;(void)l; return NULL; }
float       vl53l0x_getSignalRateLimit(vl53l0x_t *v)             { (void)v; return 0.25f; }

const char *vl53l0x_setMeasurementTimingBudget(vl53l0x_t *v, uint32_t us) { (void)v;(void)us; return NULL; }
uint32_t    vl53l0x_getMeasurementTimingBudget(vl53l0x_t *v)               { (void)v; return 33000; }

const char *vl53l0x_setVcselPulsePeriod(vl53l0x_t *v, vl53l0x_vcselPeriodType t, uint8_t p) { (void)v;(void)t;(void)p; return NULL; }
uint8_t     vl53l0x_getVcselPulsePeriod(vl53l0x_t *v, vl53l0x_vcselPeriodType t)            { (void)v;(void)t; return 14; }

/* ========================================================================== */
/*  Continuous mode / range reading                                          */
/* ========================================================================== */

void vl53l0x_startContinuous(vl53l0x_t *v, uint32_t period_ms) { (void)v;(void)period_ms; }
void vl53l0x_stopContinuous (vl53l0x_t *v)                      { (void)v; }

uint16_t vl53l0x_readRangeContinuousMillimeters(vl53l0x_t *v) {
    (void)v;
    return sim_get_tof_distance_mm();
}

uint16_t vl53l0x_readRangeSingleMillimeters(vl53l0x_t *v) {
    (void)v;
    return sim_get_tof_distance_mm();
}

/* ========================================================================== */
/*  Timeout / error                                                          */
/* ========================================================================== */

void vl53l0x_setTimeout    (vl53l0x_t *v, uint16_t t) { (void)v;(void)t; }
uint16_t vl53l0x_getTimeout(vl53l0x_t *v)              { (void)v; return 500; }
int vl53l0x_timeoutOccurred(vl53l0x_t *v) { (void)v; return 0; }
int vl53l0x_i2cFail        (vl53l0x_t *v) { (void)v; return 0; }
