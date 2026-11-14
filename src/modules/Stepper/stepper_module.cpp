#include "modules/stepper_module.h"

StepperModule::StepperModule()
    : driver(
          AccelStepper::DRIVER,
          StepperConfig::STEP_PIN,
          StepperConfig::DIR_PIN),
      awake(false)
{
}

// ==========================================
// INITIALIZATION
// ==========================================

void StepperModule::begin()
{
    pinMode(StepperConfig::SLEEP_PIN, OUTPUT);

    // Wake driver during initialization
    digitalWrite(StepperConfig::SLEEP_PIN, HIGH);

    delay(StepperConfig::WAKEUP_DELAY_MS);

    awake = true;

    driver.setMaxSpeed(StepperConfig::MAX_SPEED);
    driver.setAcceleration(StepperConfig::ACCELERATION);

    driver.setCurrentPosition(0);

    StepperModule::sleep();

    Serial.println("[STEPPER] Initialized");
    Serial.printf(
        "[STEPPER] Steps per revolution: %u\n",
        StepperConfig::STEPS_PER_REV);
}

// ==========================================
// DRIVER CONTROL
// ==========================================

void StepperModule::wake()
{
    if (awake)
        return;

    digitalWrite(StepperConfig::SLEEP_PIN, HIGH);

    delay(StepperConfig::WAKEUP_DELAY_MS);

    awake = true;

    Serial.println("[STEPPER] Driver awake");
}

void StepperModule::sleep()
{
    if (!awake)
        return;

    // Discard pending movement and stop issuing pulses
    driver.setCurrentPosition(driver.currentPosition());

    digitalWrite(StepperConfig::SLEEP_PIN, LOW);

    awake = false;

    Serial.println("[STEPPER] Driver sleeping");
}

bool StepperModule::isAwake() const
{
    return awake;
}

// ==========================================
// MOVEMENT
// ==========================================

bool StepperModule::moveRelative(long steps)
{
    if (!awake)
        StepperModule::wake();

    driver.move(steps);
    while (driver.distanceToGo() != 0)
    {
        driver.run();
    }

    StepperModule::sleep();

    return true;
}

bool StepperModule::moveAbsolute(long position)
{
    if (!awake)
        StepperModule::wake();

    driver.moveTo(position);
    while (driver.distanceToGo() != 0)
    {
        driver.run();
    }

    StepperModule::sleep();

    return true;
}

void StepperModule::stop()
{
    if (!awake)
        return;

    // Controlled deceleration
    driver.stop();

    StepperModule::sleep();
}

void StepperModule::run()
{
    if (!awake)
        StepperModule::wake();

    driver.run();
}

// ==========================================
// MOTION CONFIGURATION
// ==========================================

void StepperModule::setMaxSpeed(float speed)
{
    if (speed <= 0)
        return;

    driver.setMaxSpeed(speed);
}

void StepperModule::setAcceleration(float acceleration)
{
    if (acceleration <= 0)
        return;

    driver.setAcceleration(acceleration);
}

// ==========================================
// POSITION INFORMATION
// ==========================================

long StepperModule::getCurrentPosition()
{
    return driver.currentPosition();
}

long StepperModule::getTargetPosition()
{
    return driver.targetPosition();
}

long StepperModule::getDistanceToGo()
{
    return driver.distanceToGo();
}

bool StepperModule::isMoving()
{
    return driver.isRunning();
}