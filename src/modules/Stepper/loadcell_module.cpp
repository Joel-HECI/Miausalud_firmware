#include "modules/loadcell_module.h"

// ESP32 critical section for precise clock timing
static portMUX_TYPE hx711Mux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================
// CONSTRUCTOR
// ==========================================

LoadCellModule::LoadCellModule()
    : initialized(false),
      continuous(false),
      lastAcquisition(0)
{
}

// ==========================================
// INITIALIZATION
// ==========================================

void LoadCellModule::begin()
{
    pinMode(LoadCellConfig::HX1_DOUT_PIN, INPUT);
    pinMode(LoadCellConfig::HX2_DOUT_PIN, INPUT);

    pinMode(LoadCellConfig::SHARED_SCK_PIN, OUTPUT);

    digitalWrite(LoadCellConfig::SHARED_SCK_PIN, LOW);

    hx1 = SensorData();
    hx2 = SensorData();

    hx1.enabled = true;
    hx2.enabled = true;

    initialized = true;
    continuous = false;

    Serial.println("[LOADCELL] Module initialized");

    Serial.println("[LOADCELL] HX1 DOUT: GPIO42");
    Serial.println("[LOADCELL] HX2 DOUT: GPIO41");
    Serial.println("[LOADCELL] Shared SCK: GPIO40");

    Serial.println("[LOADCELL] Continuous mode: OFF");
}

// ==========================================
// SENSOR ACCESS
// ==========================================

LoadCellModule::SensorData &
LoadCellModule::getSensor(LoadCellID id)
{
    return (id == LoadCellID::HX1) ? hx1 : hx2;
}

const LoadCellModule::SensorData &
LoadCellModule::getSensor(LoadCellID id) const
{
    return (id == LoadCellID::HX1) ? hx1 : hx2;
}

// ==========================================
// ENABLE / DISABLE
// ==========================================

void LoadCellModule::enable(LoadCellID id)
{
    SensorData &sensor = getSensor(id);

    sensor.enabled = true;
    sensor.valid = false;

    Serial.printf(
        "[LOADCELL] HX%u enabled\n",
        static_cast<unsigned>(id));
}

void LoadCellModule::disable(LoadCellID id)
{
    SensorData &sensor = getSensor(id);

    sensor.enabled = false;
    sensor.valid = false;

    Serial.printf(
        "[LOADCELL] HX%u disabled\n",
        static_cast<unsigned>(id));
}

bool LoadCellModule::isEnabled(LoadCellID id) const
{
    return getSensor(id).enabled;
}

// ==========================================
// SYNCHRONIZED RAW ACQUISITION
// ==========================================

bool LoadCellModule::readRawPair(
    int32_t &raw1,
    int32_t &raw2,
    bool waitForReady,
    uint32_t timeout)
{
    if (!initialized)
        return false;

    if (!hx1.enabled && !hx2.enabled)
        return false;

    uint32_t startTime = millis();

    bool ready1 = false;
    bool ready2 = false;

    // Wait until all enabled ADCs are ready
    while (true)
    {
        ready1 = !hx1.enabled ||
                 (digitalRead(LoadCellConfig::HX1_DOUT_PIN) == LOW);

        ready2 = !hx2.enabled ||
                 (digitalRead(LoadCellConfig::HX2_DOUT_PIN) == LOW);

        if (ready1 && ready2)
            break;

        if (!waitForReady)
            return false;

        if (millis() - startTime >= timeout)
            return false;

        delay(1);
    }

    uint32_t data1 = 0;
    uint32_t data2 = 0;

    // ======================================
    // READ BOTH ADCs SIMULTANEOUSLY
    // ======================================

    portENTER_CRITICAL(&hx711Mux);

    for (uint8_t i = 0; i < 24; i++)
    {
        digitalWrite(LoadCellConfig::SHARED_SCK_PIN, HIGH);

        delayMicroseconds(1);

        data1 <<= 1;
        data2 <<= 1;

        if (hx1.enabled)
        {
            data1 |= digitalRead(
                LoadCellConfig::HX1_DOUT_PIN);
        }

        if (hx2.enabled)
        {
            data2 |= digitalRead(
                LoadCellConfig::HX2_DOUT_PIN);
        }

        digitalWrite(LoadCellConfig::SHARED_SCK_PIN, LOW);

        delayMicroseconds(1);
    }

    // 25th pulse: select Channel A, gain 128
    digitalWrite(LoadCellConfig::SHARED_SCK_PIN, HIGH);

    delayMicroseconds(1);

    digitalWrite(LoadCellConfig::SHARED_SCK_PIN, LOW);

    portEXIT_CRITICAL(&hx711Mux);

    // ======================================
    // SIGN EXTENSION (24-BIT TO 32-BIT)
    // ======================================

    if (data1 & 0x800000)
        data1 |= 0xFF000000;

    if (data2 & 0x800000)
        data2 |= 0xFF000000;

    raw1 = static_cast<int32_t>(data1);
    raw2 = static_cast<int32_t>(data2);

    return true;
}

// ==========================================
// AVERAGED ACQUISITION
// ==========================================

bool LoadCellModule::readAveraged(
    LoadCellID id,
    int32_t &result,
    uint8_t samples)
{
    if (samples == 0)
        return false;

    if (!isEnabled(id))
        return false;

    int64_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        int32_t raw1 = 0;
        int32_t raw2 = 0;

        if (!readRawPair(
                raw1,
                raw2,
                true,
                LoadCellConfig::READY_TIMEOUT_MS))
        {
            return false;
        }

        sum += (id == LoadCellID::HX1) ? raw1 : raw2;
    }

    result = static_cast<int32_t>(sum / samples);

    return true;
}

// ==========================================
// WEIGHT CALCULATION
// ==========================================

void LoadCellModule::updateWeight(SensorData &sensor)
{
    sensor.weight =
        static_cast<float>(sensor.raw - sensor.offset) / sensor.scale;

    sensor.valid = true;
    sensor.timestamp = millis();
}

// ==========================================
// ON-DEMAND ACQUISITION
// ==========================================

bool LoadCellModule::acquire(uint8_t samples)
{
    if (!initialized)
        return false;

    if (samples == 0)
        return false;

    int64_t sum1 = 0;
    int64_t sum2 = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        int32_t raw1 = 0;
        int32_t raw2 = 0;

        if (!readRawPair(
                raw1,
                raw2,
                true,
                LoadCellConfig::READY_TIMEOUT_MS))
        {
            Serial.println(
                "[LOADCELL] Acquisition timeout!");

            return false;
        }

        if (hx1.enabled)
            sum1 += raw1;

        if (hx2.enabled)
            sum2 += raw2;
    }

    if (hx1.enabled)
    {
        hx1.raw = static_cast<int32_t>(sum1 / samples);

        updateWeight(hx1);
    }

    if (hx2.enabled)
    {
        hx2.raw = static_cast<int32_t>(sum2 / samples);

        updateWeight(hx2);
    }

    lastAcquisition = millis();

    return true;
}

// ==========================================
// CONTINUOUS ACQUISITION
// ==========================================

void LoadCellModule::setContinuous(bool enabled)
{
    continuous = enabled;

    lastAcquisition = millis();

    Serial.printf(
        "[LOADCELL] Continuous acquisition: %s\n",
        continuous ? "ON" : "OFF");
}

bool LoadCellModule::isContinuous() const
{
    return continuous;
}

void LoadCellModule::update()
{
    if (!initialized || !continuous)
        return;

    if (millis() - lastAcquisition <
        LoadCellConfig::CONTINUOUS_INTERVAL_MS)
    {
        return;
    }

    // Non-blocking acquisition
    int32_t raw1 = 0;
    int32_t raw2 = 0;

    if (!readRawPair(raw1, raw2, false, 0))
        return;

    if (hx1.enabled)
    {
        hx1.raw = raw1;
        updateWeight(hx1);
    }

    if (hx2.enabled)
    {
        hx2.raw = raw2;
        updateWeight(hx2);
    }

    lastAcquisition = millis();

    // Print only enabled sensors
    Serial.print("[LOADCELL] ");

    if (hx1.enabled)
    {
        Serial.printf(
            "HX1: %.2f | ",
            hx1.weight);
    }

    if (hx2.enabled)
    {
        Serial.printf(
            "HX2: %.2f",
            hx2.weight);
    }

    Serial.println();
}

// ==========================================
// ZEROING / TARE
// ==========================================

bool LoadCellModule::tare(
    LoadCellID id,
    uint8_t samples)
{
    if (!isEnabled(id))
        return false;

    int32_t average = 0;

    Serial.printf(
        "[LOADCELL] Zeroing HX%u...\n",
        static_cast<unsigned>(id));

    if (!readAveraged(id, average, samples))
    {
        Serial.println("[LOADCELL] Tare failed!");
        return false;
    }

    SensorData &sensor = getSensor(id);

    sensor.offset = average;
    sensor.raw = average;
    sensor.weight = 0.0f;
    sensor.valid = true;
    sensor.timestamp = millis();

    Serial.printf(
        "[LOADCELL] HX%u zero offset: %ld\n",
        static_cast<unsigned>(id),
        static_cast<long>(average));

    return true;
}

// ==========================================
// CALIBRATION
// ==========================================

bool LoadCellModule::calibrate(
    LoadCellID id,
    float knownWeight,
    uint8_t samples)
{
    if (!isEnabled(id))
        return false;

    if (!isfinite(knownWeight) || knownWeight == 0.0f)
        return false;

    int32_t measuredRaw = 0;

    Serial.printf(
        "[LOADCELL] Calibrating HX%u...\n",
        static_cast<unsigned>(id));

    if (!readAveraged(id, measuredRaw, samples))
    {
        Serial.println("[LOADCELL] Calibration failed!");
        return false;
    }

    SensorData &sensor = getSensor(id);

    int32_t difference = measuredRaw - sensor.offset;

    if (difference == 0)
    {
        Serial.println(
            "[LOADCELL] Calibration error: zero difference");

        return false;
    }

    sensor.scale =
        static_cast<float>(difference) / knownWeight;

    sensor.raw = measuredRaw;

    updateWeight(sensor);

    Serial.printf(
        "[LOADCELL] HX%u calibration factor: %.6f\n",
        static_cast<unsigned>(id),
        sensor.scale);

    return true;
}

// ==========================================
// MANUAL SCALE CONFIGURATION
// ==========================================

bool LoadCellModule::setScale(
    LoadCellID id,
    float scale)
{
    if (!isfinite(scale) || scale == 0.0f)
        return false;

    SensorData &sensor = getSensor(id);

    sensor.scale = scale;

    if (sensor.valid)
        updateWeight(sensor);

    Serial.printf(
        "[LOADCELL] HX%u scale: %.6f\n",
        static_cast<unsigned>(id),
        scale);

    return true;
}

float LoadCellModule::getScale(LoadCellID id) const
{
    return getSensor(id).scale;
}

// ==========================================
// READING ACCESS
// ==========================================

LoadCellReading LoadCellModule::getReading(
    LoadCellID id) const
{
    const SensorData &sensor = getSensor(id);

    LoadCellReading reading;

    reading.raw = sensor.raw;
    reading.weight = sensor.weight;
    reading.valid = sensor.valid;
    reading.timestamp = sensor.timestamp;

    return reading;
}

// ==========================================
// STATUS
// ==========================================

void LoadCellModule::printStatus(Stream &output) const
{
    output.println("\n========== LOAD CELL MODULE ==========");

    output.printf(
        "Continuous mode: %s\n",
        continuous ? "ON" : "OFF");

    output.println("\n--- HX711 #1 ---");

    output.printf(
        "Enabled: %s\n",
        hx1.enabled ? "YES" : "NO");

    output.printf(
        "Valid reading: %s\n",
        hx1.valid ? "YES" : "NO");

    output.printf(
        "Raw: %ld\n",
        static_cast<long>(hx1.raw));

    output.printf(
        "Offset: %ld\n",
        static_cast<long>(hx1.offset));

    output.printf(
        "Scale: %.6f\n",
        hx1.scale);

    output.printf(
        "Weight: %.3f\n",
        hx1.weight);

    output.println("\n--- HX711 #2 ---");

    output.printf(
        "Enabled: %s\n",
        hx2.enabled ? "YES" : "NO");

    output.printf(
        "Valid reading: %s\n",
        hx2.valid ? "YES" : "NO");

    output.printf(
        "Raw: %ld\n",
        static_cast<long>(hx2.raw));

    output.printf(
        "Offset: %ld\n",
        static_cast<long>(hx2.offset));

    output.printf(
        "Scale: %.6f\n",
        hx2.scale);

    output.printf(
        "Weight: %.3f\n",
        hx2.weight);

    output.println("======================================");
}