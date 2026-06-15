#pragma once
#include <Arduino.h>
#include <GxEPD2_3C.h>
#include <SPI.h>

#define EPD_CS   42
#define EPD_DC   41
#define EPD_RST  2
#define EPD_BUSY 1
#define EPD_SCK  5
#define EPD_MOSI 6

extern GxEPD2_3C<GxEPD2_213_Z19c, GxEPD2_213_Z19c::HEIGHT> display;

bool initEpd();
void testHelloEpd();