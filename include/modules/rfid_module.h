#ifndef RFID_MODULE_H
#define RFID_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// ==========================================
// RFID CONFIGURATION
// ==========================================

namespace RFIDConfig
{

    // SPI GPIO configuration
    constexpr uint8_t SS_PIN = 39;
    constexpr uint8_t SCK_PIN = 38;
    constexpr uint8_t MOSI_PIN = 37;
    constexpr uint8_t MISO_PIN = 36;

    // Additional GPIOs
    constexpr uint8_t IRQ_PIN = 35;
    constexpr uint8_t RST_PIN = 21;

    // Scanning configuration
    constexpr bool SCANNING_ENABLED_AT_BOOT = true;

    constexpr uint16_t SCAN_INTERVAL_MS = 50;

    constexpr uint16_t CARD_REMOVAL_TIMEOUT_MS = 500;

    // Deep sleep wake-up configuration
    // Disabled by default.
    constexpr bool WAKEUP_ENABLED = false;

    // GPIO35 is NOT RTC-capable on ESP32-S3.
    // This pin must be changed and physically rewired
    // before implementing deep sleep wake-up.
    constexpr uint8_t WAKEUP_PIN = IRQ_PIN;

}

// ==========================================
// RFID MODULE
// ==========================================

class RFIDModule
{

public:
    RFIDModule();

    // Initialization
    bool begin();

    // Scanning control
    void update();

    void setScanning(bool enabled);

    bool isScanning() const;

    // Tag information
    bool isCardPresent() const;

    String getLastUID() const;

    uint8_t getLastUIDSize() const;

    uint32_t getLastDetectionTime() const;

    // Wake-up configuration
    bool isWakeupEnabled() const;

    // Status
    void printStatus(Stream &output) const;

private:
    MFRC522 reader;

    bool initialized;
    bool scanning;
    bool cardPresent;

    String lastUID;

    uint8_t lastUIDSize;

    uint32_t lastDetectionTime;
    uint32_t lastScanTime;

    // Internal functions
    bool readCard();

    String formatUID();

    void handleCardDetected();

    void handleCardRemoved();
};

#endif