#include "camera_manager.h"
#include "sd_manager.h"

// ==========================================
// OV2640 Camera Pin Definitions
// ==========================================
#define PWDN_GPIO_NUM     8   // CAM_PWDN
#define RESET_GPIO_NUM    17  // CAM_RST
#define XCLK_GPIO_NUM     9  // CAM_XCLK
#define SIOD_GPIO_NUM     15  // CAM_SDA
#define SIOC_GPIO_NUM     16  // CAM_SCL

// Data Pins (Y9 to Y2 correspond to D7 to D0)
#define Y9_GPIO_NUM       46  // CAM_D7
#define Y8_GPIO_NUM       10  // CAM_D6
#define Y7_GPIO_NUM       11  // CAM_D5
#define Y6_GPIO_NUM       13  // CAM_D4
#define Y5_GPIO_NUM       21  // CAM_D3
#define Y4_GPIO_NUM       48  // CAM_D2
#define Y3_GPIO_NUM       47  // CAM_D1
#define Y2_GPIO_NUM       14  // CAM_D0 

#define VSYNC_GPIO_NUM    18  // CAM_VSYNC
#define HREF_GPIO_NUM     3   // CAM_HREF
#define PCLK_GPIO_NUM     12  // CAM_PCLK
// ==========================================

bool camera_init() {
    log_i("Initializing OV2640 Camera...");

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    
    // Pin assignments
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; 
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    
    // OV2640 specific clock
    config.xclk_freq_hz = 20000000;
    
    // Format and Quality Settings
    config.frame_size = FRAMESIZE_QVGA;        // 320x240
    config.pixel_format = PIXFORMAT_GRAYSCALE; // 直接抓灰階，省內存、防崩潰
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;   // 確保放入 PSRAM
    config.jpeg_quality = 12;                  // (在 Grayscale 模式下此參數會被忽略，但保留無妨)
    config.fb_count = 1;                       // 單緩衝區，最穩

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        log_e("Camera init failed with error 0x%x", err);
        return false;
    }

    log_i("Camera init SUCCESS!");
    return true;
}

void take_test_photo() {
    log_i("Taking a test photo...");
    
    // Capture a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        log_e("Camera capture failed! Check power or pin connections.");
        return;
    }
    
    log_i("Capture SUCCESS! Image size: %zu bytes", fb->len);
    log_i("Image Width: %d, Height: %d", fb->width, fb->height);
    
    saveImageToSd(fb);

    // IMPORTANT: Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
}