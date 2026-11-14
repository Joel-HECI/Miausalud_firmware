#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

struct ServoConfig
{
    uint8_t PWM_PIN = 47;

    // Standard continuous-rotation servo pulse calibration.
    uint16_t MIN_PULSE_US = 1000;
    uint16_t NEUTRAL_PULSE_US = 1500;
    uint16_t MAX_PULSE_US = 2000;

    uint16_t FREQUENCY_HZ = 50;
};

class ServoModule
{
public:
    explicit ServoModule(const ServoConfig &config = ServoConfig());

    bool begin();

    // Call continuously from loop().
    void update();

    // Continuous rotation. Speed range: -100 to +100.
    // Negative = reverse, positive = forward, zero = neutral.
    void setSpeed(float speed);

    // Rotate at the requested speed for durationMs, then stop.
    void rotateFor(float speed, uint32_t durationMs);

    // Send the neutral pulse and cancel any timed rotation.
    void stop();

    bool isInitialized() const;
    bool isRunningTimed() const;
    float getSpeed() const;
    uint32_t getRemainingTime() const;

    void printStatus(Stream &output) const;

private:
    ServoConfig config;
    Servo servo;

    bool initialized;
    bool timedMode;

    float currentSpeed;

    uint32_t timedStart;
    uint32_t timedDuration;

    float clampSpeed(float speed) const;
    uint16_t speedToPulse(float speed) const;
    void applySpeed(float speed);
};