#pragma once

#include <Arduino.h>

enum TemperatureCluster : uint8_t
{
    TEMP1 = 1,
    TEMP2 = 2,
    TEMP3 = 3
};

struct TemperatureConfig
{
    static constexpr uint8_t CLUSTER_COUNT = 3;

    uint8_t pins[CLUSTER_COUNT] = {
        10, 11, 12};

    // LM35 nominal output: 10 mV per degree Celsius.
    float MILLIVOLTS_PER_CELSIUS = 10.0f;

    // Inputs above this voltage are considered disconnected.
    // Requires an external 10k pull-up to 3.3V on each signal.
    uint16_t DISCONNECTED_THRESHOLD_MV = 2200;

    uint16_t SAMPLE_INTERVAL_MS = 500;
    uint8_t DEFAULT_SAMPLES = 4;
};

struct TemperatureReading
{
    uint16_t raw;
    uint32_t millivolts;
    float temperatureC;

    bool valid;
    bool connected;

    uint32_t timestamp;
};

class TemperatureModule
{
public:
    explicit TemperatureModule(
        const TemperatureConfig &config = TemperatureConfig());

    bool begin();

    // Call continuously from loop().
    void update();

    // On-demand sampling of one cluster.
    bool sample(
        TemperatureCluster cluster,
        uint8_t samples = 1);

    // On-demand sampling of all clusters.
    void sampleAll(uint8_t samples = 1);

    // Independent continuous acquisition control.
    void setContinuous(
        TemperatureCluster cluster,
        bool enabled);

    bool isContinuous(TemperatureCluster cluster) const;

    // Calibration using a known reference temperature.
    bool calibrate(
        TemperatureCluster cluster,
        float knownTemperatureC,
        uint8_t samples = 8);

    // Manually configure calibration parameters.
    void setCalibration(
        TemperatureCluster cluster,
        float offsetC,
        float gain);

    float getOffset(TemperatureCluster cluster) const;
    float getGain(TemperatureCluster cluster) const;

    // Retrieve the most recent reading.
    TemperatureReading getReading(
        TemperatureCluster cluster) const;

    bool isConnected(TemperatureCluster cluster) const;

    void printStatus(Stream &output) const;

private:
    struct ClusterState
    {
        bool continuous;
        float offsetC;
        float gain;

        uint32_t lastSampleTime;

        TemperatureReading reading;
    };

    TemperatureConfig config;
    ClusterState clusters[TemperatureConfig::CLUSTER_COUNT];

    bool initialized;

    int8_t getIndex(TemperatureCluster cluster) const;

    bool acquire(
        uint8_t index,
        uint8_t samples);

    float convertToCelsius(uint32_t millivolts,
                           uint8_t index) const;
};