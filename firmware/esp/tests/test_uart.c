// test uart
#include "uart.h"
#include <string.h>

void test_uart(void) {
    printf("\nUART Test:\n");
    
    xUARTInit();
    
    const char *msg = "HELLO";
    xUARTWrite((uint8_t*)msg, strlen(msg));
    printf("Sent: %s\n", msg);
    
    uint8_t rx[128];
    size_t len;
    xUARTRead(rx, 128, &len);
    rx[len] = '\0';
    printf("Received: %s (%d bytes)\n", rx, len);
}