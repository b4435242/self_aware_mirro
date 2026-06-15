#include "epd_manager.h"
#include <Fonts/FreeMonoBold9pt7b.h> 

// 🔹 建立電子紙物件 (2.13吋 3色 Z19c)
GxEPD2_3C<GxEPD2_213_Z19c, GxEPD2_213_Z19c::HEIGHT> display(GxEPD2_213_Z19c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

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
