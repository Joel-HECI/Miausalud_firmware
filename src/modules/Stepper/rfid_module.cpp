#include "modules/rfid_module.h"

// ==========================================
// CONSTRUCTOR
// ==========================================

RFIDModule::RFIDModule()
    : reader(
          RFIDConfig::SS_PIN,
          RFIDConfig::RST_PIN),
      initialized(false),
      scanning(false),
      cardPresent(false),
      lastUID(""),
      lastUIDSize(0),
      lastDetectionTime(0),
      lastScanTime(0)
{
}

// ==========================================
// INITIALIZATION
// ==========================================

bool RFIDModule::begin()
{
    Serial.println("[RFID] Initializing module...");

    // Initialize SPI with custom GPIO configuration
    SPI.begin(
        RFIDConfig::SCK_PIN,
        RFIDConfig::MISO_PIN,
        RFIDConfig::MOSI_PIN,
        RFIDConfig::SS_PIN);

    // Configure IRQ input
    pinMode(RFIDConfig::IRQ_PIN, INPUT);

    // Initialize MFRC522
    reader.PCD_Init();

    delay(50);

    // Verify reader communication
    byte version = reader.PCD_ReadRegister(
        MFRC522::VersionReg);

    if (version == 0x00 || version == 0xFF)
    {
        Serial.println("[RFID] Reader communication FAILED");

        initialized = false;
        return false;
    }

    initialized = true;

    scanning = RFIDConfig::SCANNING_ENABLED_AT_BOOT;

    reader.PCD_AntennaOn();

    Serial.println("[RFID] Initialization successful");

    Serial.printf(
        "[RFID] MFRC522 version: 0x%02X\n",
        version);

    Serial.printf(
        "[RFID] SPI SS: GPIO%u\n",
        RFIDConfig::SS_PIN);

    Serial.printf(
        "[RFID] IRQ: GPIO%u\n",
        RFIDConfig::IRQ_PIN);

    Serial.printf(
        "[RFID] Scanning: %s\n",
        scanning ? "ON" : "OFF");

    Serial.printf(
        "[RFID] Deep sleep wake-up: %s\n",
        isWakeupEnabled() ? "ON" : "OFF");

    return true;
}

// ==========================================
// SCANNING CONTROL
// ==========================================

void RFIDModule::setScanning(bool enabled)
{
    if (!initialized)
        return;

    scanning = enabled;

    if (scanning)
    {
        reader.PCD_AntennaOn();

        lastScanTime = millis();

        Serial.println("[RFID] Scanning enabled");
    }
    else
    {
        reader.PCD_AntennaOff();

        cardPresent = false;

        lastUID = "";

        lastUIDSize = 0;

        Serial.println("[RFID] Scanning disabled");
    }
}

bool RFIDModule::isScanning() const
{
    return scanning;
}

// ==========================================
// UID FORMATTING
// ==========================================

String RFIDModule::formatUID()
{
    String uid = "";

    for (byte i = 0; i < reader.uid.size; i++)
    {
        if (reader.uid.uidByte[i] < 0x10)
            uid += "0";

        uid += String(
            reader.uid.uidByte[i],
            HEX);

        if (i < reader.uid.size - 1)
            uid += ":";
    }

    uid.toUpperCase();

    return uid;
}

// ==========================================
// CARD ACQUISITION
// ==========================================

bool RFIDModule::readCard()
{
    byte atqaBuffer[2];

    byte atqaSize = sizeof(atqaBuffer);

    // Wake up cards previously placed in HALT state.
    MFRC522::StatusCode wakeStatus =
        reader.PICC_WakeupA(
            atqaBuffer,
            &atqaSize);

    if (
        wakeStatus != MFRC522::STATUS_OK &&
        wakeStatus != MFRC522::STATUS_COLLISION)
    {
        return false;
    }

    // Read selected card UID
    if (!reader.PICC_ReadCardSerial())
    {
        return false;
    }

    // Release card for subsequent scans
    reader.PICC_HaltA();

    reader.PCD_StopCrypto1();

    return true;
}

// ==========================================
// CARD DETECTION
// ==========================================

void RFIDModule::handleCardDetected()
{
    String currentUID = formatUID();

    // Check whether this is a new card
    if (!cardPresent || currentUID != lastUID)
    {
        lastUID = currentUID;

        lastUIDSize = reader.uid.size;

        cardPresent = true;

        Serial.println("\n========== RFID EVENT ==========");

        Serial.println("[RFID] CARD DETECTED");

        Serial.printf(
            "UID: %s\n",
            lastUID.c_str());

        Serial.printf(
            "UID length: %u bytes\n",
            lastUIDSize);

        Serial.printf(
            "Card type: %s\n",
            reader.PICC_GetTypeName(
                reader.PICC_GetType(reader.uid.sak)));

        Serial.println("================================\n");
    }

    // Update timestamp even if the card is unchanged
    lastDetectionTime = millis();
}

// ==========================================
// CARD REMOVAL
// ==========================================

void RFIDModule::handleCardRemoved()
{
    if (!cardPresent)
        return;

    if (
        millis() - lastDetectionTime >=
        RFIDConfig::CARD_REMOVAL_TIMEOUT_MS)
    {
        Serial.println("\n========== RFID EVENT ==========");

        Serial.println("[RFID] CARD REMOVED");

        Serial.printf(
            "Previous UID: %s\n",
            lastUID.c_str());

        Serial.println("================================\n");

        cardPresent = false;

        lastUID = "";

        lastUIDSize = 0;
    }
}

// ==========================================
// MAIN SCANNING UPDATE
// ==========================================

void RFIDModule::update()
{
    if (!initialized || !scanning)
        return;

    if (
        millis() - lastScanTime <
        RFIDConfig::SCAN_INTERVAL_MS)
    {
        return;
    }

    lastScanTime = millis();

    if (readCard())
    {
        handleCardDetected();
    }
    else
    {
        handleCardRemoved();
    }
}

// ==========================================
// TAG INFORMATION
// ==========================================

bool RFIDModule::isCardPresent() const
{
    return cardPresent;
}

String RFIDModule::getLastUID() const
{
    return lastUID;
}

uint8_t RFIDModule::getLastUIDSize() const
{
    return lastUIDSize;
}

uint32_t RFIDModule::getLastDetectionTime() const
{
    return lastDetectionTime;
}

// ==========================================
// WAKE-UP CONFIGURATION
// ==========================================

bool RFIDModule::isWakeupEnabled() const
{
    return RFIDConfig::WAKEUP_ENABLED;
}

// ==========================================
// STATUS
// ==========================================

void RFIDModule::printStatus(Stream &output) const
{
    output.println("\n========== RFID MODULE ==========");

    output.printf(
        "Initialized: %s\n",
        initialized ? "YES" : "NO");

    output.printf(
        "Scanning: %s\n",
        scanning ? "ON" : "OFF");

    output.printf(
        "Card present: %s\n",
        cardPresent ? "YES" : "NO");

    output.printf(
        "Last UID: %s\n",
        cardPresent ? lastUID.c_str() : "NONE");

    output.printf(
        "UID length: %u bytes\n",
        lastUIDSize);

    output.printf(
        "Last detection: %lu ms\n",
        static_cast<unsigned long>(lastDetectionTime));

    output.printf(
        "Wake-up enabled: %s\n",
        isWakeupEnabled() ? "YES" : "NO");

    output.println("=================================");
}