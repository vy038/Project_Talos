// test_camera.c
#include "camera.h"
#include "esp_camera.h"

void test_camera(void) {
    printf("\nCamera Test:\n");
    
    camera_init();
    
    for (int i = 0; i < 5; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            printf("Frame %d: %d bytes, %dx%d\n", 
                   i, fb->len, fb->width, fb->height);
            esp_camera_fb_return(fb);
        } else {
            printf("Frame %d: FAILED\n", i);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}