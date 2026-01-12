// test camera
#include "camera.h"

void test_camera(void) {
    printf("\nCamera Test:\n");
    
    camera_config_t cfg = {.width = 640, .height = 480};
    camera_init(&cfg);
    
    for (int i = 0; i < 5; i++) {
        camera_fb_t *fb = camera_capture_frame();
        printf("Frame %d: %d bytes\n", i, fb ? fb->len : 0);
        if (fb) camera_return_frame(fb);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}