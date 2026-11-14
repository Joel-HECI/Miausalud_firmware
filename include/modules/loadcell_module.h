#ifndef LOADCELL_MODULE_H
#define LOADCELL_MODULE_H

#include <Arduino.h>

namespace LoadCellConfig
{

    // GPIO configuration
    constexpr uint8_t HX1_DOUT_PIN = 42;
    constexpr uint8_t HX2_DOUT_PIN = 41;
    constexpr uint8_t SHARED_SCK_PIN = 40;

    // HX711 configuration
    constexpr uint8_t HX711_GAIN = 128;

    // Acquisition configuration
    constexpr uint16_t READY_TIMEOUT_MS = 1500;
    constexpr uint16_t CONTINUOUS_INTERVAL_MS = 150;

    // Default averaging
    constexpr uint8_t DEFAULT_SAMPLES = 5;

}

enum class LoadCellID : uint8_t
{
    HX1 = 1,
    HX2 = 2
};

struct LoadCellReading
{

    int32_t raw = 0;

    float weight = 0.0f;

    bool valid = false;

    uint32_t timestamp = 0;
};

class LoadCellModule
{

public:
    LoadCellModule();

    // Initialization
    void begin();

    // Sensor availability
    void enable(LoadCellID id);
    void disable(LoadCellID id);

    bool isEnabled(LoadCellID id) const;

    // Acquisition
    bool acquire(uint8_t samples = 1);
    void update();

    // Continuous acquisition
    void setContinuous(bool enabled);
    bool isContinuous() const;

    // Calibration
    bool tare(LoadCellID id, uint8_t samples = 10);

    bool calibrate(
        LoadCellID id,
        float knownWeight,
        uint8_t samples = 5);

    bool setScale(LoadCellID id, float scale);
    float getScale(LoadCellID id) const;

    // Reading access
    LoadCellReading getReading(LoadCellID id) const;

    // Status
    void printStatus(Stream &output) const;

private:
    struct SensorData
    {

        bool enabled = true;

        int32_t raw = 0;

        int32_t offset = 0;

        float scale = 1.0f;

        float weight = 0.0f;

        bool valid = false;

        uint32_t timestamp = 0;
    };

    SensorData hx1;
    SensorData hx2;

    bool initialized;
    bool continuous;

    uint32_t lastAcquisition;

    // Hardware acquisition
    bool readRawPair(
        int32_t &raw1,
        int32_t &raw2,
        bool waitForReady,
        uint32_t timeout);

    bool readAveraged(
        LoadCellID id,
        int32_t &result,
        uint8_t samples);

    // Data processing
    void updateWeight(SensorData &sensor);

    SensorData &getSensor(LoadCellID id);
    const SensorData &getSensor(LoadCellID id) const;
};

#endif