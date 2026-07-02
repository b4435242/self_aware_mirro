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
#include "epd_manager.h"

HWCDC USBSerial;


#define SPI_SCK  5   // Shared SCK
#define SPI_MOSI 6   // Shared MOSI
#define TF_MISO  4   // SD Card MISO
#define TF_CS    7   // SD Card CS

const int BUTTON_PIN = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    init_epd_buffer();

    // Call the modular init function
    if (sd_init(SPI_SCK, TF_MISO, SPI_MOSI, TF_CS)) {        
        printSDCardInfo();
    } else {
        log_e("Critical Error: SD subsystem failed to initialize.");
    }

    // 2. Initialize Camera subsystem
    if (!camera_init()) {
        log_e("Subsystem Warning: Camera initialization failed.");
    }

    if (initEpd()) {
        log_i("✅ 電子紙模組啟動成功，開始顯示測試畫面！");
    } else {
        log_e("❌ 電子紙模組啟動失敗，請檢查硬體連線！");
    }
}

void loop() {
    // 偵測按鈕是否被按下 (LOW 為導通狀態)
    if (digitalRead(BUTTON_PIN) == LOW) {
        
        // 軟體消除彈跳 (Debounce)
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
            log_i(">>> 按鈕觸發：開始拍照與顯示流程 <<<");
            
            // 擷取相機影格
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                // 將影像資料交給 EPD Manager 處理並重新整理螢幕
                display_camera_frame(fb->buf, fb->width, fb->height);
                
                // 釋放相機記憶體
                esp_camera_fb_return(fb);
            } else {
                log_e("Camera capture failed");
            }

            // 等待按鈕放開，防止連續觸發
            while (digitalRead(BUTTON_PIN) == LOW) {
                delay(10);
            }
            log_i(">>> 流程結束，回到待機狀態 <<<");
        }
    }
    
    // 讓出 CPU 時間，防止看門狗重置
    delay(10);
}