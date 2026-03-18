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
 * Type 0x01 — ball detection (11 bytes total):
 *   [0xAA][0x55][0x01][det][x_hi][x_lo][y_hi][y_lo][r_hi][r_lo][chk]
 *   det  : 1=detected, 0=not detected
 *   x    : centroid x (0=left, 319=right)
 *   y    : centroid y (0=top,  239=bottom)
 *   r    : fused distance mm if detected, else 0
 */

#define TALOS_PKT_START0        0xAAu
#define TALOS_PKT_START1        0x55u
#define TALOS_PKT_TYPE_DETECT   0x01u
#define TALOS_PKT_LEN           11u
#define TALOS_PKT_PAYLOAD_LEN   7u

typedef struct {
    bool     detected;
    uint16_t x;     /* centroid x */
    uint16_t y;     /* centroid y */
    uint16_t r;     /* dist_mm when detected, else 0 */
} talos_detection_t;

/* Zero-init before first use */
typedef struct {
    uint8_t buf[TALOS_PKT_LEN];
    uint8_t pos;
} talos_framer_t;

/**
 * @brief Build an 11-byte detection packet into buf.
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
