#include <Arduino.h>
#include <SPI.h>
#include "esp_camera.h"
#include <GxEPD2_3C.h>

// ==========================================
// 1. 硬體定義：電子紙安全腳位 (Z19c 三色)
// ==========================================
const int EPD_CS   = 1;  
const int EPD_DC   = 2;  
const int EPD_RST  = 42; 
const int EPD_BUSY = 14; 
const int EPD_MOSI = 47; 
const int EPD_SCK  = 21; 

GxEPD2_3C<GxEPD2_213_Z19c, GxEPD2_213_Z19c::HEIGHT> display(GxEPD2_213_Z19c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// 電子紙目標尺寸 (橫向)
#define EPD_WIDTH  212
#define EPD_HEIGHT 104

// ==========================================
// 2. 硬體定義：Goouuu ESP32-S3-CAM 相機腳位
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define href_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

// ==========================================
// 圖片處理全域變數 (放在 PSRAM)
// ==========================================
uint8_t *processed_image_bw = NULL; // 儲存處理好的 212x104 灰階資料

// 初始化相機
esp_err_t init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
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
    config.pin_href = href_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA; // 320x240，方便縮放裁切
    config.pixel_format = PIXFORMAT_GRAYSCALE; // 直接抓灰階照片，省內存好處理
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM; // 相機緩衝區放在 PSRAM
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // 初始化
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return err;
    }
    return ESP_OK;
}

// ==========================================
// 高品質圖片處理：Floyd-Steinberg 抖動演算法
// ==========================================
void capture_process_display() {
    Serial.println("Taking picture...");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    Serial.println("Processing image in PSRAM...");
    // 原始圖片尺寸 (QVGA)
    int src_w = fb->width;
    int src_h = fb->height;

    // 1. 縮放與裁切 (Nearest Neighbor)：將 320x240 轉為 212x104 並存入 processed_image_bw
    // (簡單的居中裁切邏輯)
    int crop_x = (src_w - EPD_WIDTH) / 2;
    int crop_y = (src_h - EPD_HEIGHT) / 2;

    for (int y = 0; y < EPD_HEIGHT; y++) {
        for (int x = 0; x < EPD_WIDTH; x++) {
            // 獲取相應的原始相機像素位置
            int src_x = x + crop_x;
            int src_y = y + crop_y;
            
            // 安全檢查邊界
            if(src_x < 0 || src_x >= src_w || src_y < 0 || src_y >= src_h) {
                processed_image_bw[y * EPD_WIDTH + x] = 255; // 填白
                continue;
            }

            // 直接複製灰階值 (0-255)
            processed_image_bw[y * EPD_WIDTH + x] = fb->buf[src_y * src_w + src_x];
        }
    }
    
    esp_camera_fb_return(fb); // 處理完立刻釋放相機緩衝區

    // 2. 執行 Floyd-Steinberg 抖動 (高品質黑白轉換)
    // 演算法需要帶符號整數來處理累計誤差
    for (int y = 0; y < EPD_HEIGHT; y++) {
        for (int x = 0; x < EPD_WIDTH; x++) {
            int old_pixel = processed_image_bw[y * EPD_WIDTH + x];
            int new_pixel = (old_pixel < 128) ? 0 : 255; // 二值化 (黑或白)
            
            // 將新值存回 (0 為黑，255 為白)
            processed_image_bw[y * EPD_WIDTH + x] = new_pixel;
            
            // 計算誤差
            int quant_error = old_pixel - new_pixel;
            
            // 將誤差傳播給周圍像素 (擴散)
            // 右邊像素: x+1, y
            if (x + 1 < EPD_WIDTH) {
                int p = processed_image_bw[y * EPD_WIDTH + (x + 1)];
                processed_image_bw[y * EPD_WIDTH + (x + 1)] = constrain(p + (quant_error * 7 / 16), 0, 255);
            }
            // 左下像素: x-1, y+1
            if (x - 1 >= 0 && y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + (x - 1)];
                processed_image_bw[(y + 1) * EPD_WIDTH + (x - 1)] = constrain(p + (quant_error * 3 / 16), 0, 255);
            }
            // 下面像素: x, y+1
            if (y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + x];
                processed_image_bw[(y + 1) * EPD_WIDTH + x] = constrain(p + (quant_error * 5 / 16), 0, 255);
            }
            // 右下像素: x+1, y+1
            if (x + 1 < EPD_WIDTH && y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + (x + 1)];
                processed_image_bw[(y + 1) * EPD_WIDTH + (x + 1)] = constrain(p + (quant_error * 1 / 16), 0, 255);
            }
        }
    }

    // 3. 顯示到電子紙
    Serial.println("Refreshing E-Paper...");
    display.setFullWindow();
    display.firstPage();
    do {
        // 先刷白底
        display.fillScreen(GxEPD_WHITE);
        
        // 遍歷處理好的 PSRAM 資料，逐點繪製
        for (int y = 0; y < EPD_HEIGHT; y++) {
            for (int x = 0; x < EPD_WIDTH; x++) {
                // 如果是 0，繪製黑色；如果是 255，繪製白色
                if (processed_image_bw[y * EPD_WIDTH + x] == 0) {
                    display.drawPixel(x, y, GxEPD_BLACK);
                }
                // (未玩紅色，需要紅色請繪製 GxEPD_RED)
            }
        }
    } while (display.nextPage());

    Serial.println("Update done. See the photo!");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32-S3 Cam to E-Paper ---");

    // 檢查有沒有偵測到 PSRAM (關鍵！)
    if (!psramFound()) {
        Serial.println("⚠️ Error: PSRAM not found! Need build_flags in platformio.ini");
        while (1) delay(100);
    }
    Serial.printf("PSRAM size: %d MB\n", ESP.getPsramSize() / 1024 / 1024);

    // 在 PSRAM 分配緩衝區 (212 * 104 bytes)
    processed_image_bw = (uint8_t *)ps_malloc(EPD_WIDTH * EPD_HEIGHT);
    if (!processed_image_bw) {
        Serial.println("Failed to allocate image buffer in PSRAM");
        while (1) delay(100);
    }

    // 初始化相機
    if (init_camera() != ESP_OK) while (1) delay(100);

    // 初始化電子紙
    SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
    display.init(115200, true, 2, false);
    display.setRotation(1); // 橫向

    // 執行第一次拍照顯示
    capture_process_display();
}

void loop() {
    // 這裡是空的，避免電子紙一直刷新
    // 如果想要定時拍照，可以寫在這裡，並加上長 delay
    delay(10000);
}