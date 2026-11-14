#include "modules/temperature_module.h"

TemperatureModule::TemperatureModule(
    const TemperatureConfig &config)
    : config(config),
      initialized(false)
{

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        clusters[i].continuous = false;
        clusters[i].offsetC = 0.0f;
        clusters[i].gain = 1.0f;
        clusters[i].lastSampleTime = 0;

        clusters[i].reading.raw = 0;
        clusters[i].reading.millivolts = 0;
        clusters[i].reading.temperatureC = 0.0f;
        clusters[i].reading.valid = false;
        clusters[i].reading.connected = false;
        clusters[i].reading.timestamp = 0;
    }
}

// ============================================================
// INITIALIZATION
// ============================================================

bool TemperatureModule::begin()
{
    if (initialized)
    {
        return true;
    }

    if (config.MILLIVOLTS_PER_CELSIUS <= 0.0f ||
        config.DISCONNECTED_THRESHOLD_MV == 0 ||
        config.DEFAULT_SAMPLES == 0)
    {
        Serial.println("[TEMP] Invalid configuration.");
        return false;
    }

    analogReadResolution(12);

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        pinMode(config.pins[i], INPUT);

        analogSetPinAttenuation(
            config.pins[i],
            ADC_11db);

        clusters[i].continuous = false;
    }

    initialized = true;

    Serial.println("[TEMP] Temperature module initialized.");

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        Serial.printf(
            "[TEMP] Cluster %u initialized on GPIO%u\n",
            i + 1,
            config.pins[i]);
    }

    Serial.println("[TEMP] Continuous sampling is OFF by default.");

    return true;
}

// ============================================================
// CLUSTER INDEX
// ============================================================

int8_t TemperatureModule::getIndex(
    TemperatureCluster cluster) const
{
    uint8_t id = static_cast<uint8_t>(cluster);

    if (id < 1 || id > TemperatureConfig::CLUSTER_COUNT)
    {
        return -1;
    }

    return static_cast<int8_t>(id - 1);
}

// ============================================================
// TEMPERATURE CONVERSION
// ============================================================

float TemperatureModule::convertToCelsius(
    uint32_t millivolts,
    uint8_t index) const
{
    float temperature =
        (float)millivolts / config.MILLIVOLTS_PER_CELSIUS;

    temperature *= clusters[index].gain;
    temperature += clusters[index].offsetC;

    return temperature;
}

// ============================================================
// ACQUISITION
// ============================================================

bool TemperatureModule::acquire(
    uint8_t index,
    uint8_t samples)
{
    if (!initialized ||
        index >= TemperatureConfig::CLUSTER_COUNT ||
        samples == 0)
    {
        return false;
    }

    uint32_t millivoltSum = 0;
    uint32_t rawSum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        rawSum += analogRead(config.pins[index]);

        millivoltSum += analogReadMilliVolts(
            config.pins[index]);

        delayMicroseconds(500);
    }

    uint16_t averageRaw = rawSum / samples;
    uint32_t averageMillivolts = millivoltSum / samples;

    TemperatureReading &reading = clusters[index].reading;

    reading.raw = averageRaw;
    reading.millivolts = averageMillivolts;
    reading.timestamp = millis();

    clusters[index].lastSampleTime = reading.timestamp;

    // The external pull-up should drive a disconnected input
    // above the maximum expected LM35 output voltage.
    if (averageMillivolts >= config.DISCONNECTED_THRESHOLD_MV)
    {
        reading.connected = false;
        reading.valid = false;
        reading.temperatureC = NAN;

        return false;
    }

    reading.connected = true;
    reading.valid = true;

    reading.temperatureC = convertToCelsius(
        averageMillivolts,
        index);

    return true;
}

bool TemperatureModule::sample(
    TemperatureCluster cluster,
    uint8_t samples)
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return false;
    }

    return acquire((uint8_t)index, samples);
}

void TemperatureModule::sampleAll(uint8_t samples)
{
    if (!initialized)
    {
        return;
    }

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        acquire(i, samples);
    }
}

// ============================================================
// CONTINUOUS ACQUISITION
// ============================================================

void TemperatureModule::update()
{
    if (!initialized)
    {
        return;
    }

    uint32_t now = millis();

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        if (!clusters[i].continuous)
        {
            continue;
        }

        if ((uint32_t)(now - clusters[i].lastSampleTime) >=
            config.SAMPLE_INTERVAL_MS)
        {

            acquire(i, config.DEFAULT_SAMPLES);
        }
    }
}

void TemperatureModule::setContinuous(
    TemperatureCluster cluster,
    bool enabled)
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return;
    }

    clusters[index].continuous = enabled;

    if (enabled)
    {
        // Force the first acquisition on the next update().
        clusters[index].lastSampleTime =
            millis() - config.SAMPLE_INTERVAL_MS;
    }

    Serial.printf(
        "[TEMP] Cluster %u continuous sampling %s.\n",
        (uint8_t)cluster,
        enabled ? "ENABLED" : "DISABLED");
}

bool TemperatureModule::isContinuous(
    TemperatureCluster cluster) const
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return false;
    }

    return clusters[index].continuous;
}

// ============================================================
// CALIBRATION
// ============================================================

bool TemperatureModule::calibrate(
    TemperatureCluster cluster,
    float knownTemperatureC,
    uint8_t samples)
{
    int8_t index = getIndex(cluster);

    if (index < 0 || !initialized || samples == 0)
    {
        return false;
    }

    if (!acquire((uint8_t)index, samples))
    {
        Serial.printf(
            "[TEMP] Cannot calibrate cluster %u: sensor disconnected.\n",
            (uint8_t)cluster);

        return false;
    }

    float measuredTemperature =
        clusters[index].reading.temperatureC;

    clusters[index].offsetC +=
        knownTemperatureC - measuredTemperature;

    // Update the current reading using the new calibration.
    clusters[index].reading.temperatureC = knownTemperatureC;

    Serial.printf(
        "[TEMP] Cluster %u calibrated.\n",
        (uint8_t)cluster);

    Serial.printf(
        "[TEMP] Reference: %.2f C\n",
        knownTemperatureC);

    Serial.printf(
        "[TEMP] New offset: %.3f C\n",
        clusters[index].offsetC);

    return true;
}

void TemperatureModule::setCalibration(
    TemperatureCluster cluster,
    float offsetC,
    float gain)
{
    int8_t index = getIndex(cluster);

    if (index < 0 || gain <= 0.0f)
    {
        return;
    }

    clusters[index].offsetC = offsetC;
    clusters[index].gain = gain;

    Serial.printf(
        "[TEMP] Cluster %u calibration updated. Offset: %.3f C, gain: %.5f\n",
        (uint8_t)cluster,
        offsetC,
        gain);
}

float TemperatureModule::getOffset(
    TemperatureCluster cluster) const
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return 0.0f;
    }

    return clusters[index].offsetC;
}

float TemperatureModule::getGain(
    TemperatureCluster cluster) const
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return 1.0f;
    }

    return clusters[index].gain;
}

// ============================================================
// READING ACCESS
// ============================================================

TemperatureReading TemperatureModule::getReading(
    TemperatureCluster cluster) const
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        TemperatureReading invalid = {};
        invalid.temperatureC = NAN;
        invalid.valid = false;
        invalid.connected = false;

        return invalid;
    }

    return clusters[index].reading;
}

bool TemperatureModule::isConnected(
    TemperatureCluster cluster) const
{
    int8_t index = getIndex(cluster);

    if (index < 0)
    {
        return false;
    }

    return clusters[index].reading.connected;
}

// ============================================================
// STATUS
// ============================================================

void TemperatureModule::printStatus(Stream &output) const
{
    output.println("\n========== TEMPERATURE STATUS ==========");

    output.printf(
        "Initialized: %s\n",
        initialized ? "YES" : "NO");

    output.printf(
        "Sampling interval: %u ms\n",
        config.SAMPLE_INTERVAL_MS);

    for (uint8_t i = 0; i < TemperatureConfig::CLUSTER_COUNT; i++)
    {
        const ClusterState &state = clusters[i];
        const TemperatureReading &reading = state.reading;

        output.printf("\nCluster %u\n", i + 1);
        output.printf("GPIO: %u\n", config.pins[i]);

        output.printf(
            "Continuous: %s\n",
            state.continuous ? "ON" : "OFF");

        output.printf(
            "Connection: %s\n",
            reading.connected ? "CONNECTED" : "DISCONNECTED / UNKNOWN");

        if (reading.valid)
        {
            output.printf("Raw ADC: %u\n", reading.raw);
            output.printf("Voltage: %lu mV\n",
                          (unsigned long)reading.millivolts);
            output.printf("Temperature: %.2f C\n",
                          reading.temperatureC);
        }
        else
        {
            output.println("Temperature: INVALID");
        }

        output.printf("Calibration offset: %.3f C\n", state.offsetC);
        output.printf("Calibration gain: %.5f\n", state.gain);
        output.printf("Last sample: %lu ms\n",
                      (unsigned long)reading.timestamp);
    }

    output.println("========================================");
}