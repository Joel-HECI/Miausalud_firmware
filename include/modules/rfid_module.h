
#ifndef RFID_MODULE_H
#define RFID_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

namespace RFIDConfig
{

    static constexpr uint8_t SS_PIN = 39;
    static constexpr uint8_t SCK_PIN = 38;
    static constexpr uint8_t MOSI_PIN = 37;
    static constexpr uint8_t MISO_PIN = 36;

    // Dedicated RTC-capable external interrupt.
    static constexpr uint8_t IRQ_PIN = 1;

    static constexpr uint8_t RST_PIN = 21;

    static constexpr bool SCANNING_ENABLED_AT_BOOT = true;

    static constexpr uint32_t SCAN_INTERVAL_MS = 50;
    static constexpr uint32_t CARD_REMOVAL_TIMEOUT_MS = 500;

    // MFRC522 IRQ output is active LOW.
    static constexpr uint8_t IRQ_ACTIVE_LEVEL = LOW;

    // ComIEnReg: enable RxIRq only.
    // Bit 7 = 0 keeps the IRQ output active LOW.
    static constexpr uint8_t RX_INTERRUPT_MASK = 0x20;
}

class RFIDModule
{

public:
    RFIDModule();

    bool begin();
    void update();

    void setScanning(bool enabled);
    bool isScanning() const;

    bool isCardPresent() const;

    String getLastUID() const;
    uint8_t getLastUIDSize() const;
    uint32_t getLastDetectionTime() const;

    // Hardware interrupt monitoring.
    bool isIRQEnabled() const;
    uint32_t getIRQCount() const;
    bool isIRQActive() const;

    // Returns false for the MFRC522 because its receiver
    // cannot autonomously interrogate newly presented
    // passive cards throughout ESP32 deep sleep.
    bool prepareForSleepWake();

    // Compatibility with the previously defined interface.
    bool isWakeupEnabled() const;

    void printStatus(Stream &output) const;

private:
    MFRC522 reader;

    bool initialized;
    bool scanningEnabled;
    bool irqEnabled;
    bool cardPresent;

    String lastUID;
    uint8_t lastUIDSize;

    uint32_t lastDetectionTime;
    uint32_t lastScanTime;

    void enableReceiveIRQ();
    void disableReceiveIRQ();
    void clearReceiveIRQ();

    void scanCard();
    void handleCardRemoval();
    String formatUID();
};

#endif
