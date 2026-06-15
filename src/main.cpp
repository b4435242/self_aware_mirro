#include <Arduino.h>
#include <SPI.h>
#include "esp_camera.h"
#include <GxEPD2_3C.h>
#include <WiFiManager.h>
#include <time.h>
#include "FS.h"
#include "SD.h"
#include <HWCDC.h>
#include "sd_manager.h"
#include "camera_manager.h"

HWCDC USBSerial;

#define SPI_SCK  5   // Shared SCK
#define SPI_MOSI 6   // Shared MOSI
#define TF_MISO  4   // SD Card MISO
#define TF_CS    7   // SD Card CS

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

// 電子紙目標尺寸 (改為直向)
#define EPD_WIDTH  104
#define EPD_HEIGHT 212


// ==========================================
// 3. 按鈕定義 (使用外部按鈕模組與模擬電源)
// ==========================================
const int BUTTON_PIN = 3;   // 訊號輸出腳
const int FAKE_VCC   = 41;  // 模擬 VCC 腳
const int FAKE_GND   = 45;  // 模擬 GND 腳

// ==========================================
// 圖片處理全域變數 (放在 PSRAM)
// ==========================================
uint8_t *processed_image_bw = NULL; // 儲存處理好的 212x104 灰階資料


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

void psram_init(){
    log_i("Checking PSRAM availability...");
    if (!psramFound()) {
        log_e("FATAL Error: PSRAM not found!");
        log_e("Please ensure 'build_flags = -D BOARD_HAS_PSRAM' is in platformio.ini");
        while (1) delay(100); // 系統死循環卡在這裡，保護後續硬體不崩潰
    }
    log_i("PSRAM Mount SUCCESS! Size: %d MB", ESP.getPsramSize() / (1024 * 1024));

    log_i("Allocating Image Buffer in PSRAM...");
    processed_image_bw = (uint8_t *)ps_malloc(EPD_WIDTH * EPD_HEIGHT);
    if (!processed_image_bw) {
        log_e("FATAL Error: Failed to allocate EPD image buffer in PSRAM!");
        while (1) delay(100); 
    }
    log_i("Buffer Allocated SUCCESS! Size: %d bytes", EPD_WIDTH * EPD_HEIGHT);
    log_i("----------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    psram_init();

    // Call the modular init function
    if (sd_init(SPI_SCK, TF_MISO, SPI_MOSI, TF_CS)) {
        
        // Print info and run read/write test if initialization succeeded
        printSDCardInfo();
        
        const char* testFilePath = "/boot_log.txt";
        const char* testMessage = "System successfully mounted SD via standard SPI.\n";

        // Note: We now pass the standard 'SD' object, not SD_MMC
        writeToFile(SD, testFilePath, testMessage);
        readFromFile(SD, testFilePath);
        
    } else {
        log_e("Critical Error: SD subsystem failed to initialize.");
    }

    // 2. Initialize Camera subsystem
    if (camera_init()) {
        take_test_photo();
    } else {
        log_e("Subsystem Warning: Camera initialization failed.");
    }

}

void loop() {
    delay(3000);
}