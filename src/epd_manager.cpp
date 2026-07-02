#include "epd_manager.h"
#include <Fonts/FreeMonoBold9pt7b.h> 

// 🔹 建立電子紙物件 (2.13吋 3色 Z19c)
GxEPD2_3C<GxEPD2_213_Z19c, GxEPD2_213_Z19c::HEIGHT> display(GxEPD2_213_Z19c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

uint8_t *processed_image_bw = NULL;

/**
 * @brief 初始化電子紙顯示器
 * @return true 成功 / false 失敗
 */
bool initEpd() {
    log_i("開始初始化電子紙 (腳位: CS=%d, DC=%d, RST=%d, BUSY=%d)...", EPD_CS, EPD_DC, EPD_RST, EPD_BUSY);

    // 示範 log_e 的使用：檢查重要腳位是否有被定義 (避免設定為 -1 等無效腳位)
    if (EPD_CS < 0 || EPD_DC < 0) {
        log_e("❌ 電子紙腳位設定錯誤！");
        return false;
    }

    SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
    // 參數 115200 是 GxEPD2 內部除錯用的鮑率
    display.init(115200, true, 2, false); 
    
    log_i("✅ 電子紙初始化完成！");
    return true;
}

/**
 * @brief 顯示 Hello World 測試畫面
 */
void testHelloEpd() {
    log_i("🔄 準備更新電子紙畫面...");
    
    display.setRotation(1); 
    display.setFont(&FreeMonoBold9pt7b); 
    display.setTextSize(1); 

    display.setFullWindow();
    display.firstPage();
    do {
        // 1. 填滿白色背景
        display.fillScreen(GxEPD_WHITE);
        
        // 2. 顯示黑色文字 "Hello"
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(20, 50); 
        display.print("Hello ");
        
        // 3. 顯示紅色文字 "World!"
        display.setTextColor(GxEPD_RED);
        display.print("World!");
        
    } while (display.nextPage());

    log_i("✅ 畫面更新完畢！");
}

void init_epd_buffer() {
    log_i("Checking PSRAM availability...");
    if (!psramFound()) {
        log_e("FATAL Error: PSRAM not found!");
        while (1) delay(100);
    }
    
    log_i("Allocating Image Buffer in PSRAM...");
    processed_image_bw = (uint8_t *)ps_malloc(EPD_WIDTH * EPD_HEIGHT);
    if (!processed_image_bw) {
        log_e("FATAL Error: Failed to allocate EPD image buffer in PSRAM!");
        while (1) delay(100); 
    }
    log_i("Buffer Allocated SUCCESS! Size: %d bytes", EPD_WIDTH * EPD_HEIGHT);
    log_i("----------------------------------------");
}

void display_camera_frame(uint8_t *cam_buf, int src_w, int src_h) {
    if (!processed_image_bw || !cam_buf) return;

    log_i("Processing image in PSRAM...");
    
    int crop_x = (src_w - EPD_WIDTH) / 2;
    int crop_y = (src_h - EPD_HEIGHT) / 2;

    // 1. 縮放與裁切
    for (int y = 0; y < EPD_HEIGHT; y++) {
        for (int x = 0; x < EPD_WIDTH; x++) {
            int src_x = x + crop_x;
            int src_y = y + crop_y;
            
            if(src_x < 0 || src_x >= src_w || src_y < 0 || src_y >= src_h) {
                processed_image_bw[y * EPD_WIDTH + x] = 255;
                continue;
            }
            processed_image_bw[y * EPD_WIDTH + x] = cam_buf[src_y * src_w + src_x];
        }
    }
    
    // 2. 執行 Floyd-Steinberg 抖動
    for (int y = 0; y < EPD_HEIGHT; y++) {
        for (int x = 0; x < EPD_WIDTH; x++) {
            int old_pixel = processed_image_bw[y * EPD_WIDTH + x];
            int new_pixel = (old_pixel < 128) ? 0 : 255; 
            
            processed_image_bw[y * EPD_WIDTH + x] = new_pixel;
            int quant_error = old_pixel - new_pixel;
            
            if (x + 1 < EPD_WIDTH) {
                int p = processed_image_bw[y * EPD_WIDTH + (x + 1)];
                processed_image_bw[y * EPD_WIDTH + (x + 1)] = constrain(p + (quant_error * 7 / 16), 0, 255);
            }
            if (x - 1 >= 0 && y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + (x - 1)];
                processed_image_bw[(y + 1) * EPD_WIDTH + (x - 1)] = constrain(p + (quant_error * 3 / 16), 0, 255);
            }
            if (y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + x];
                processed_image_bw[(y + 1) * EPD_WIDTH + x] = constrain(p + (quant_error * 5 / 16), 0, 255);
            }
            if (x + 1 < EPD_WIDTH && y + 1 < EPD_HEIGHT) {
                int p = processed_image_bw[(y + 1) * EPD_WIDTH + (x + 1)];
                processed_image_bw[(y + 1) * EPD_WIDTH + (x + 1)] = constrain(p + (quant_error * 1 / 16), 0, 255);
            }
        }
    }

    // 3. 顯示到電子紙
    log_i("Refreshing E-Paper...");
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        for (int y = 0; y < EPD_HEIGHT; y++) {
            for (int x = 0; x < EPD_WIDTH; x++) {
                if (processed_image_bw[y * EPD_WIDTH + x] == 0) {
                    display.drawPixel(x, y, GxEPD_BLACK);
                }
            }
        }
    } while (display.nextPage());

    log_i("Update done. See the photo!");
}