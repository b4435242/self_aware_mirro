#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <esp_camera.h>

// Initialize the SD card over SPI (Returns true if successful)
bool sd_init(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs);

// Reusable file system functions
void writeToFile(fs::FS &fs, const char * path, const char * message);
String readFromFile(fs::FS &fs, const char * path);
void printSDCardInfo();
void saveImageToSd(camera_fb_t *fb);