#pragma once
#include <Arduino.h>
#include "esp_camera.h"

// Initialize the OV2640 Camera
// Returns true if initialization is successful
bool camera_init();

// Take a test photo and print buffer info
void take_test_photo();