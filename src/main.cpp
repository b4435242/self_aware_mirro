#include <SPI.h>
#include <GxEPD2_3C.h> // 注意！這裡改成引入 3C (三色) 函式庫
#include <Fonts/FreeMonoBold9pt7b.h>

// 安全腳位定義
const int EPD_CS   = 1;  
const int EPD_DC   = 2;  
const int EPD_RST  = 42; 
const int EPD_BUSY = 14; 
const int EPD_MOSI = 47; 
const int EPD_SCK  = 21; 

// 宣告電子紙型號：三色版 V3 通常對應 GDEY0213Z98 晶片
GxEPD2_3C<GxEPD2_213_Z19c, GxEPD2_213_Z19c::HEIGHT> display(GxEPD2_213_Z19c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 初始化 SPI
  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);

  // 初始化電子紙
  display.init(115200, true, 2, false); 
  
  display.setRotation(1); 
  display.setFont(&FreeMonoBold9pt7b);

  display.setFullWindow();
  display.firstPage();
  do {
    // 這裡可以玩紅色了！
    display.fillScreen(GxEPD_WHITE);
    
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 30);
    display.print("Tri-Color Display!");
    
    display.setTextColor(GxEPD_RED); // 設定為紅色字體！
    display.setCursor(10, 60);
    display.print("RED text works!");
    
  } while (display.nextPage()); 

  Serial.println("Update complete.");
}

void loop() {}