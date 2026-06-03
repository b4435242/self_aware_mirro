#include "sd_manager.h"

bool sd_init(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs) {
    log_i("Initializing SPI and SD Card...");

    // 1. Initialize the SPI bus with the given pins
    SPI.begin(sck, miso, mosi, cs);

    // 2. Mount the SD card using the specific CS pin and the SPI object
    if (!SD.begin(cs, SPI)) {
        log_e("SD Card Mount Failed! Check wiring, format, or power.");
        return false;
    }

    log_i("SD Card Mount SUCCESS!");
    return true;
}

void writeToFile(fs::FS &fs, const char * path, const char * message) {
    log_i("Writing file: %s", path);
    
    File file = fs.open(path, FILE_WRITE); 
    if (!file) {
        log_e("Failed to open file for writing.");
        return;
    }
    
    if (file.print(message)) {
        log_i("File written successfully.");
    } else {
        log_e("Write failed.");
    }
    file.close();
}

String readFromFile(fs::FS &fs, const char * path) {
    log_i("Reading file: %s", path);
    
    File file = fs.open(path);
    if (!file) {
        log_e("Failed to open file for reading.");
        return "";
    }
    
    String content = "";
    while (file.available()) {
        content += (char)file.read();
    }
    
    log_i("Read content: %s", content.c_str());
    file.close();
    
    return content;
}

void printSDCardInfo() {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        log_e("No SD card attached.");
        return;
    }

    if (cardType == CARD_MMC) log_i("Card Type: MMC");
    else if (cardType == CARD_SD) log_i("Card Type: SDSC");
    else if (cardType == CARD_SDHC) log_i("Card Type: SDHC/SDXC");
    else log_i("Card Type: UNKNOWN");

    // Standard SD library uses totalBytes()
    uint64_t cardSize = SD.totalBytes() / (1024 * 1024);
    log_i("SD Card Size: %llu MB", cardSize);
}