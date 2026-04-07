#include "uart_protocol.h"

static uint8_t compute_checksum(const uint8_t *buf)
{
    uint8_t chk = 0;
    for (int i = 2; i < (int)TALOS_PKT_LEN - 1; i++) {
        chk ^= buf[i];
    }
    return chk;
}

void vUARTProtoBuildDetection(uint8_t *buf, const talos_detection_t *det)
{
    buf[0]  = TALOS_PKT_START0;
    buf[1]  = TALOS_PKT_START1;
    buf[2]  = TALOS_PKT_TYPE_DETECT;
    buf[3]  = det->detected ? 1u : 0u;
    buf[4]  = (uint8_t)((det->x      >> 8) & 0xFF);
    buf[5]  = (uint8_t)( det->x             & 0xFF);
    buf[6]  = (uint8_t)((det->y      >> 8) & 0xFF);
    buf[7]  = (uint8_t)( det->y             & 0xFF);
    buf[8]  = (uint8_t)((det->px_r   >> 8) & 0xFF);
    buf[9]  = (uint8_t)( det->px_r          & 0xFF);
    buf[10] = (uint8_t)((det->tof_mm >> 8) & 0xFF);
    buf[11] = (uint8_t)( det->tof_mm        & 0xFF);
    buf[12] = compute_checksum(buf);
}

bool bUARTProtoFeedByte(talos_framer_t *framer, uint8_t byte, talos_detection_t *out)
{
    uint8_t pos = framer->pos;

    if (pos == 0) {
        if (byte != TALOS_PKT_START0) return false;
        framer->buf[0] = byte;
        framer->pos    = 1;
        return false;
    }

    if (pos == 1) {
        if (byte != TALOS_PKT_START1) {
            framer->pos    = (byte == TALOS_PKT_START0) ? 1 : 0;
            framer->buf[0] = TALOS_PKT_START0;
            return false;
        }
        framer->buf[1] = byte;
        framer->pos    = 2;
        return false;
    }

    framer->buf[pos] = byte;
    framer->pos      = pos + 1;

    if (framer->pos < TALOS_PKT_LEN) return false;

    framer->pos = 0;

    if (framer->buf[TALOS_PKT_LEN - 1] != compute_checksum(framer->buf)) return false;
    if (framer->buf[2] != TALOS_PKT_TYPE_DETECT)                          return false;

    out->detected = (framer->buf[3] != 0);
    out->x      = ((uint16_t)framer->buf[4]  << 8) | framer->buf[5];
    out->y      = ((uint16_t)framer->buf[6]  << 8) | framer->buf[7];
    out->px_r   = ((uint16_t)framer->buf[8]  << 8) | framer->buf[9];
    out->tof_mm = ((uint16_t)framer->buf[10] << 8) | framer->buf[11];
    return true;
}

bool bUARTProtoFeedBuf(talos_framer_t *framer, const uint8_t *buf, size_t len,
                       size_t *consumed, talos_detection_t *out)
{
    for (size_t i = 0; i < len; i++) {
        if (bUARTProtoFeedByte(framer, buf[i], out)) {
            *consumed = i + 1;
            return true;
        }
    }
    *consumed = len;
    return false;
}
