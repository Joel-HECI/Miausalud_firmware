
#include "modules/rfid_module.h"

namespace
{

    volatile bool rfidIRQPending = false;
    volatile uint32_t rfidIRQCount = 0;

    void IRAM_ATTR onRFIDInterrupt()
    {
        rfidIRQPending = true;
        rfidIRQCount++;
    }
}

// ============================================================
// CONSTRUCTOR
// ============================================================

RFIDModule::RFIDModule()
    : reader(
          RFIDConfig::SS_PIN,
          RFIDConfig::RST_PIN),
      initialized(false),
      scanningEnabled(false),
      irqEnabled(false),
      cardPresent(false),
      lastUID(""),
      lastUIDSize(0),
      lastDetectionTime(0),
      lastScanTime(0)
{
}

// ============================================================
// INITIALIZATION
// ============================================================

bool RFIDModule::begin()
{
    initialized = false;
    scanningEnabled = false;
    irqEnabled = false;
    cardPresent = false;

    rfidIRQPending = false;
    rfidIRQCount = 0;

    // The scheduler restores GPIO1 after RTC EXT0 wake.
    pinMode(RFIDConfig::IRQ_PIN, INPUT_PULLUP);

    SPI.begin(
        RFIDConfig::SCK_PIN,
        RFIDConfig::MISO_PIN,
        RFIDConfig::MOSI_PIN,
        RFIDConfig::SS_PIN);

    reader.PCD_Init();

    delay(5);

    byte version = reader.PCD_ReadRegister(
        MFRC522::VersionReg);

    if (version == 0x00 || version == 0xFF)
    {
        Serial.println(
            "[RFID] MFRC522 not responding.");
        return false;
    }

    initialized = true;

    reader.PCD_AntennaOn();

    // Clear leftover reader interrupts.
    clearReceiveIRQ();

    // Route receive-complete interrupt to IRQ pin.
    enableReceiveIRQ();

    attachInterrupt(
        digitalPinToInterrupt(RFIDConfig::IRQ_PIN),
        onRFIDInterrupt,
        FALLING);

    irqEnabled = true;

    setScanning(
        RFIDConfig::SCANNING_ENABLED_AT_BOOT);

    Serial.printf(
        "[RFID] MFRC522 version: 0x%02X\n",
        version);

    Serial.printf(
        "[RFID] IRQ GPIO: %u\n",
        RFIDConfig::IRQ_PIN);

    Serial.println(
        "[RFID] Receive IRQ monitoring enabled.");

    return true;
}

// ============================================================
// CLEAR INTERRUPTS
// ============================================================

void RFIDModule::clearReceiveIRQ()
{
    if (!initialized)
    {
        return;
    }

    // Writing ones clears the corresponding IRQ flags.
    reader.PCD_WriteRegister(
        MFRC522::ComIrqReg,
        0x7F);

    rfidIRQPending = false;
}

// ============================================================
// ENABLE RECEIVE INTERRUPT
// ============================================================

void RFIDModule::enableReceiveIRQ()
{
    if (!initialized)
    {
        return;
    }

    clearReceiveIRQ();

    reader.PCD_WriteRegister(
        MFRC522::ComIEnReg,
        RFIDConfig::RX_INTERRUPT_MASK);
}

// ============================================================
// DISABLE RECEIVE INTERRUPT
// ============================================================

void RFIDModule::disableReceiveIRQ()
{
    if (!initialized)
    {
        return;
    }

    reader.PCD_WriteRegister(
        MFRC522::ComIEnReg,
        0x00);

    clearReceiveIRQ();
}

// ============================================================
// ENABLE / DISABLE SCANNING
// ============================================================

void RFIDModule::setScanning(bool enabled)
{
    if (!initialized)
    {
        return;
    }

    scanningEnabled = enabled;

    if (enabled)
    {

        reader.PCD_AntennaOn();
        enableReceiveIRQ();

        lastScanTime =
            millis() - RFIDConfig::SCAN_INTERVAL_MS;
    }
    else
    {

        disableReceiveIRQ();
        reader.PCD_AntennaOff();

        cardPresent = false;
    }
}

// ============================================================
// SCANNING STATUS
// ============================================================

bool RFIDModule::isScanning() const
{
    return scanningEnabled;
}

// ============================================================
// FORMAT CARD UID
// ============================================================

String RFIDModule::formatUID()
{
    String result;

    for (byte i = 0; i < reader.uid.size; i++)
    {

        if (i > 0)
        {
            result += ":";
        }

        if (reader.uid.uidByte[i] < 0x10)
        {
            result += "0";
        }

        String value = String(
            reader.uid.uidByte[i],
            HEX);

        value.toUpperCase();
        result += value;
    }

    return result;
}

// ============================================================
// SCAN A CARD
// ============================================================

void RFIDModule::scanCard()
{
    if (!initialized || !scanningEnabled)
    {
        return;
    }

    // PICC_WakeupA actively transmits WUPA.
    // This is required because passive tags do not
    // spontaneously send their UIDs.
    byte atqa[2];
    byte atqaSize = sizeof(atqa);

    MFRC522::StatusCode result =
        reader.PICC_WakeupA(atqa, &atqaSize);

    if (
        result != MFRC522::STATUS_OK &&
        result != MFRC522::STATUS_COLLISION)
    {
        return;
    }

    if (!reader.PICC_ReadCardSerial())
    {
        return;
    }

    String uid = formatUID();

    bool newCard =
        !cardPresent ||
        uid != lastUID;

    lastUID = uid;
    lastUIDSize = reader.uid.size;

    lastDetectionTime = millis();
    cardPresent = true;

    if (newCard)
    {

        Serial.print("[RFID EVENT] Card detected: ");
        Serial.println(lastUID);
    }

    reader.PICC_HaltA();
    reader.PCD_StopCrypto1();
}

// ============================================================
// CARD REMOVAL
// ============================================================

void RFIDModule::handleCardRemoval()
{
    if (!cardPresent)
    {
        return;
    }

    if (
        millis() - lastDetectionTime >=
        RFIDConfig::CARD_REMOVAL_TIMEOUT_MS)
    {
        cardPresent = false;

        Serial.println(
            "[RFID EVENT] Card removed.");
    }
}

// ============================================================
// UPDATE
// ============================================================

void RFIDModule::update()
{
    if (!initialized || !scanningEnabled)
    {
        return;
    }

    // The MFRC522 receive IRQ reports responses to
    // active reader transactions. It does not generate
    // interrogation commands by itself.
    //
    // Continue polling at the established scan interval
    // so cards can be detected after presentation.

    uint32_t now = millis();

    if (
        now - lastScanTime >=
        RFIDConfig::SCAN_INTERVAL_MS)
    {
        lastScanTime = now;

        scanCard();
    }

    // A receive IRQ may have been raised during
    // PICC_WakeupA / anticollision.
    //
    // The corresponding RFID data is processed through
    // scanCard() using the normal MFRC522 API.

    if (rfidIRQPending)
    {
        rfidIRQPending = false;
    }

    handleCardRemoval();
}

// ============================================================
// DEEP-SLEEP RFID WAKEUP CAPABILITY
// ============================================================

bool RFIDModule::prepareForSleepWake()
{
    // The MFRC522 has receive-complete interrupts,
    // but no autonomous low-power card-detection
    // engine that periodically sends REQA/WUPA.
    //
    // Simply enabling RxIRq before deep sleep would
    // not allow an arbitrary future tag presentation
    // to wake the ESP32-S3.
    //
    // Explicitly report unsupported rather than
    // giving the scheduler a false readiness signal.

    return false;
}

// ============================================================
// WAKEUP STATUS
// ============================================================

bool RFIDModule::isWakeupEnabled() const
{
    return false;
}

// ============================================================
// IRQ STATUS
// ============================================================

bool RFIDModule::isIRQEnabled() const
{
    return initialized && irqEnabled;
}

uint32_t RFIDModule::getIRQCount() const
{
    return rfidIRQCount;
}

bool RFIDModule::isIRQActive() const
{
    return digitalRead(RFIDConfig::IRQ_PIN) ==
           RFIDConfig::IRQ_ACTIVE_LEVEL;
}

// ============================================================
// CARD STATUS
// ============================================================

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

// ============================================================
// PRINT STATUS
// ============================================================

void RFIDModule::printStatus(Stream &output) const
{
    output.println();
    output.println("========== RFID ==========");

    output.print("Initialized: ");
    output.println(
        initialized ? "YES" : "NO");

    output.print("Scanning: ");
    output.println(
        scanningEnabled ? "ON" : "OFF");

    output.print("IRQ GPIO: ");
    output.println(RFIDConfig::IRQ_PIN);

    output.print("Receive IRQ: ");
    output.println(
        irqEnabled ? "ENABLED" : "DISABLED");

    output.print("IRQ count: ");
    output.println(rfidIRQCount);

    output.print("IRQ pin state: ");
    output.println(
        isIRQActive() ? "ACTIVE" : "INACTIVE");

    output.print("Card present: ");
    output.println(
        cardPresent ? "YES" : "NO");

    output.print("Last UID: ");
    output.println(
        lastUID.length() > 0 ? lastUID : "NONE");

    output.print("UID size: ");
    output.println(lastUIDSize);

    output.print("Last detection: ");
    output.print(lastDetectionTime);
    output.println(" ms");

    output.println(
        "Autonomous deep-sleep card wake: UNSUPPORTED");

    output.println("==========================");
}
