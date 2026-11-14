#ifndef COOLING_STACK_H
#define COOLING_STACK_H

#include <Arduino.h>

namespace CoolingConfig
{

    // GPIO configuration
    constexpr uint8_t PELTIER_PIN = 17;
    constexpr uint8_t FAN_PIN = 18;
    constexpr uint8_t PUMP_PIN = 8;

    // PWM configuration
    constexpr uint32_t PELTIER_FREQ = 1000;
    constexpr uint32_t FAN_FREQ = 1000;
    constexpr uint32_t PUMP_FREQ = 1000;

    constexpr uint8_t PWM_RESOLUTION = 12;

    constexpr uint16_t PWM_MAX =
        (1U << PWM_RESOLUTION) - 1;

}

class CoolingStack
{

public:
    CoolingStack();

    // Initialization
    bool begin();

    // Individual output control (0-100%)
    void setPeltier(float duty);
    void setFan(float duty);
    void setPump(float duty);

    // Master control
    void allOff();

    // Individual shutdown
    void peltierOff();
    void fanOff();
    void pumpOff();

    // Status
    float getPeltierDuty() const;
    float getFanDuty() const;
    float getPumpDuty() const;

    bool isPeltierOn() const;
    bool isFanOn() const;
    bool isPumpOn() const;

    void printStatus(Stream &output) const;

private:
    float peltierDuty;
    float fanDuty;
    float pumpDuty;

    bool initialized;

    void writePWM(uint8_t pin, float duty);

    float clampDuty(float duty);
};

#endif