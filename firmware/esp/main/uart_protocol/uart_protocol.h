#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*
 * Talos inter-ESP UART packet protocol
 *
 * All packets:
 *   [0xAA] [0x55] [type] [payload...] [checksum]
 *   checksum = XOR of bytes[2 .. len-2]
 *
 * Type 0x01 — ball detection (13 bytes total):
 *   [0xAA][0x55][0x01][det][x_hi][x_lo][y_hi][y_lo][px_r_hi][px_r_lo][tof_hi][tof_lo][chk]
 *   det   : 1=detected, 0=not detected
 *   x     : centroid x (0=left, 319=right)
 *   y     : centroid y (0=top,  239=bottom)
 *   px_r  : apparent pixel radius (sqrt(blob_pixels/pi)), 0 if not detected
 *   tof   : VL53L0X distance in mm, 0 if sensor unavailable or no reading
 */

#define TALOS_PKT_START0        0xAAu
#define TALOS_PKT_START1        0x55u
#define TALOS_PKT_TYPE_DETECT   0x01u
#define TALOS_PKT_LEN           13u
#define TALOS_PKT_PAYLOAD_LEN   9u

typedef struct {
    bool     detected;
    uint16_t x;       /* centroid x */
    uint16_t y;       /* centroid y */
    uint16_t px_r;    /* apparent pixel radius, 0 if not detected */
    uint16_t tof_mm;  /* VL53L0X distance in mm, 0 if unavailable */
} talos_detection_t;

/* Zero-init before first use */
typedef struct {
    uint8_t buf[TALOS_PKT_LEN];
    uint8_t pos;
} talos_framer_t;

/**
 * @brief Build a 13-byte detection packet into buf.
 * @param buf  Output buffer, must be >= TALOS_PKT_LEN bytes.
 * @param det  Detection to encode.
 */
void vUARTProtoBuildDetection(uint8_t *buf, const talos_detection_t *det);

/**
 * @brief Feed one byte into the framer.
 *
 * Returns true (and fills *out) when a complete, valid packet is received.
 * Resets automatically on checksum failure or lost sync.
 */
bool bUARTProtoFeedByte(talos_framer_t *framer, uint8_t byte, talos_detection_t *out);

/**
 * @brief Feed a buffer through the framer.
 *
 * Stops on first complete packet and sets *consumed to bytes processed.
 * Call in a loop to drain a buffer containing multiple packets.
 */
bool bUARTProtoFeedBuf(talos_framer_t *framer, const uint8_t *buf, size_t len,
                       size_t *consumed, talos_detection_t *out);
