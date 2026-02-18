/**
 * @file test_uart_protocol.c
 * @brief Host-side tests for UART detection packet parsing.
 *
 * Tests bStateMachineParseUART() logic from state_machine.c.
 * The function is copied here verbatim since state_machine.c has HW deps
 * that can't compile on the host. Keep in sync with the original.
 */

#include "esp_stubs.h"
#include "state_machine.h"
#include "test_harness.h"

/* --- Verbatim copy of bStateMachineParseUART from state_machine.c --- */

bool bStateMachineParseUART(const uint8_t *buf, size_t len, detection_result_t *result) {
    if (len < UART_MSG_LENGTH) return false;

    for (size_t i = 0; i <= len - UART_MSG_LENGTH; i++) {
        if (buf[i] != UART_MSG_START_0 || buf[i + 1] != UART_MSG_START_1) {
            continue;
        }

        const uint8_t *msg = &buf[i];

        if (msg[2] != UART_MSG_TYPE_DETECT) {
            continue;
        }

        uint8_t checksum = 0;
        for (int j = 2; j < 10; j++) {
            checksum ^= msg[j];
        }

        if (checksum != msg[10]) {
            continue;
        }

        result->detected    = (msg[3] != 0);
        result->ball_x      = (uint16_t)(msg[4] << 8) | msg[5];
        result->ball_y      = (uint16_t)(msg[6] << 8) | msg[7];
        result->ball_radius = (uint16_t)(msg[8] << 8) | msg[9];
        result->fresh       = true;

        return true;
    }

    return false;
}

/* ---------- helpers ---------- */

static void build_detect_packet(uint8_t *buf, bool detected,
                                uint16_t x, uint16_t y, uint16_t r) {
    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = 0x01;
    buf[3] = detected ? 1 : 0;
    buf[4] = (x >> 8) & 0xFF;
    buf[5] = x & 0xFF;
    buf[6] = (y >> 8) & 0xFF;
    buf[7] = y & 0xFF;
    buf[8] = (r >> 8) & 0xFF;
    buf[9] = r & 0xFF;
    uint8_t cksum = 0;
    for (int i = 2; i < 10; i++) cksum ^= buf[i];
    buf[10] = cksum;
}

/* ---------- tests ---------- */

TEST(parse_valid_detection) {
    uint8_t buf[11];
    build_detect_packet(buf, true, 160, 120, 45);

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 11, &result);

    ASSERT_TRUE(ok);
    ASSERT_TRUE(result.detected);
    ASSERT_EQ(result.ball_x, 160);
    ASSERT_EQ(result.ball_y, 120);
    ASSERT_EQ(result.ball_radius, 45);
}

TEST(parse_no_detection) {
    uint8_t buf[11];
    build_detect_packet(buf, false, 0, 0, 0);

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 11, &result);

    ASSERT_TRUE(ok);
    ASSERT_FALSE(result.detected);
}

TEST(parse_bad_checksum) {
    uint8_t buf[11];
    build_detect_packet(buf, true, 100, 50, 30);
    buf[10] ^= 0xFF;

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 11, &result);

    ASSERT_FALSE(ok);
}

TEST(parse_truncated_buffer) {
    uint8_t buf[5] = {0xAA, 0x55, 0x01, 0x01, 0x00};

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 5, &result);

    ASSERT_FALSE(ok);
}

TEST(parse_empty_buffer) {
    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(NULL, 0, &result);

    ASSERT_FALSE(ok);
}

TEST(parse_packet_with_leading_garbage) {
    uint8_t buf[16];
    buf[0] = 0x00; buf[1] = 0xFF; buf[2] = 0x42; buf[3] = 0x13; buf[4] = 0x37;
    build_detect_packet(&buf[5], true, 200, 100, 55);

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 16, &result);

    ASSERT_TRUE(ok);
    ASSERT_EQ(result.ball_x, 200);
    ASSERT_EQ(result.ball_y, 100);
    ASSERT_EQ(result.ball_radius, 55);
}

TEST(parse_wrong_message_type) {
    uint8_t buf[11];
    build_detect_packet(buf, true, 160, 120, 45);
    buf[2] = 0x99;
    uint8_t cksum = 0;
    for (int i = 2; i < 10; i++) cksum ^= buf[i];
    buf[10] = cksum;

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 11, &result);

    ASSERT_FALSE(ok);
}

TEST(parse_max_values) {
    uint8_t buf[11];
    build_detect_packet(buf, true, 319, 239, 65535);

    detection_result_t result = {0};
    bool ok = bStateMachineParseUART(buf, 11, &result);

    ASSERT_TRUE(ok);
    ASSERT_EQ(result.ball_x, 319);
    ASSERT_EQ(result.ball_y, 239);
    ASSERT_EQ(result.ball_radius, 65535);
}

/* ---------- main ---------- */

int main(void) {
    printf("UART Protocol Tests:\n");

    RUN_TEST(parse_valid_detection);
    RUN_TEST(parse_no_detection);
    RUN_TEST(parse_bad_checksum);
    RUN_TEST(parse_truncated_buffer);
    RUN_TEST(parse_empty_buffer);
    RUN_TEST(parse_packet_with_leading_garbage);
    RUN_TEST(parse_wrong_message_type);
    RUN_TEST(parse_max_values);

    TEST_REPORT();
}
