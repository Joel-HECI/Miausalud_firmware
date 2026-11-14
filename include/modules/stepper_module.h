
#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>
#include <AccelStepper.h>

namespace StepperConfig
{

    static constexpr uint8_t STEP_PIN = 15;
    static constexpr uint8_t DIR_PIN = 7;
    static constexpr uint8_t SLEEP_PIN = 16;

    // A4988: SLEEP is active LOW.
    static constexpr uint8_t SLEEP_ACTIVE_LEVEL = LOW;

    // A4988 wake-up delay, with margin.
    static constexpr uint32_t WAKE_DELAY_US = 1000;

    static constexpr float DEFAULT_MAX_SPEED = 1000.0f;
    static constexpr float DEFAULT_ACCELERATION = 500.0f;

    // Steps per revolution depend on microstepping.
    static constexpr long FULL_STEPS_PER_REV = 200;
    static constexpr uint8_t MICROSTEP_DIVIDER = 4;
    static constexpr long STEPS_PER_REV =
        FULL_STEPS_PER_REV * MICROSTEP_DIVIDER;
}

enum StepperMode : uint8_t
{
    STEPPER_IDLE = 0,
    STEPPER_POSITION,
    STEPPER_CONTINUOUS
};

class StepperModule
{

public:
    StepperModule();

    void begin();
    void run();

    // Driver power control
    void wake();
    void sleep();
    bool isAwake();

    // Position-based movement
    void moveRelative(long steps);
    void moveAbsolute(long position);

    // Continuous rotation
    void rotateContinuous(float speed);
    void stopContinuous();
    bool isContinuous();

    // Stop operations
    void stop();
    void emergencyStop();

    // Motion configuration
    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);

    float getMaxSpeed();
    float getAcceleration();
    float getContinuousSpeed();

    // Position and status
    long getCurrentPosition();
    long getTargetPosition();
    long getDistanceToGo();

    bool isMoving();
    StepperMode getMode();

    void setCurrentPosition(long position);
    void printStatus(Stream &output);

private:
    AccelStepper motor;

    bool driverAwake;
    StepperMode mode;

    float maxSpeed;
    float acceleration;
    float continuousSpeed;

    void prepareMovement();
    void finishMovement();
};

#endif
