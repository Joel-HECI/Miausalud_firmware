#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>
#include <AccelStepper.h>

namespace StepperConfig
{

    // GPIO configuration
    constexpr uint8_t STEP_PIN = 15;
    constexpr uint8_t DIR_PIN = 7;
    constexpr uint8_t SLEEP_PIN = 16;

    // Motor configuration
    constexpr uint16_t FULL_STEPS_PER_REV = 200;
    constexpr uint8_t MICROSTEP_FACTOR = 4;

    constexpr uint16_t STEPS_PER_REV =
        FULL_STEPS_PER_REV * MICROSTEP_FACTOR;

    // Default motion parameters
    constexpr float MAX_SPEED = 1000.0;
    constexpr float ACCELERATION = 500.0;

    // Driver wake-up delay
    constexpr uint16_t WAKEUP_DELAY_MS = 2;
}

class StepperModule
{

public:
    StepperModule();

    void begin();

    // Driver control
    void wake();
    void sleep();
    bool isAwake() const;

    // Motion control
    bool moveRelative(long steps);
    bool moveAbsolute(long position);

    void stop();
    void run();

    // Motion configuration
    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);

    // Position information
    long getCurrentPosition();
    long getTargetPosition();
    long getDistanceToGo();

    bool isMoving();

private:
    AccelStepper driver;

    bool awake;
};

#endif