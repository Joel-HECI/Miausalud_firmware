
#include "modules/stepper_module.h"

#include <math.h>

// ============================================================
// CONSTRUCTOR
// ============================================================

StepperModule::StepperModule()
    : motor(
          AccelStepper::DRIVER,
          StepperConfig::STEP_PIN,
          StepperConfig::DIR_PIN),
      driverAwake(false),
      mode(STEPPER_IDLE),
      maxSpeed(StepperConfig::DEFAULT_MAX_SPEED),
      acceleration(StepperConfig::DEFAULT_ACCELERATION),
      continuousSpeed(0.0f)
{
}

// ============================================================
// INITIALIZATION
// ============================================================

void StepperModule::begin()
{
    // Set SLEEP inactive before configuring the pin.
    digitalWrite(
        StepperConfig::SLEEP_PIN,
        StepperConfig::SLEEP_ACTIVE_LEVEL);

    pinMode(StepperConfig::SLEEP_PIN, OUTPUT);

    motor.setMaxSpeed(maxSpeed);
    motor.setAcceleration(acceleration);

    motor.setMinPulseWidth(2);
    motor.setCurrentPosition(0);

    driverAwake = false;
    mode = STEPPER_IDLE;
    continuousSpeed = 0.0f;
}

// ============================================================
// DRIVER WAKE
// ============================================================

void StepperModule::wake()
{
    if (driverAwake)
    {
        return;
    }

    digitalWrite(
        StepperConfig::SLEEP_PIN,
        HIGH);

    delayMicroseconds(
        StepperConfig::WAKE_DELAY_US);

    driverAwake = true;
}

// ============================================================
// DRIVER SLEEP
// ============================================================

void StepperModule::sleep()
{
    // Cancel all pending motion.
    motor.setCurrentPosition(
        motor.currentPosition());

    continuousSpeed = 0.0f;
    mode = STEPPER_IDLE;

    digitalWrite(
        StepperConfig::SLEEP_PIN,
        StepperConfig::SLEEP_ACTIVE_LEVEL);

    driverAwake = false;
}

// ============================================================
// DRIVER STATE
// ============================================================

bool StepperModule::isAwake()
{
    return driverAwake;
}

// ============================================================
// PREPARE MOVEMENT
// ============================================================

void StepperModule::prepareMovement()
{
    wake();
}

// ============================================================
// FINISH MOVEMENT
// ============================================================

void StepperModule::finishMovement()
{
    if (mode == STEPPER_CONTINUOUS)
    {
        return;
    }

    if (motor.distanceToGo() != 0)
    {
        return;
    }

    mode = STEPPER_IDLE;
    continuousSpeed = 0.0f;

    sleep();
}

// ============================================================
// RELATIVE MOVEMENT
// ============================================================

void StepperModule::moveRelative(long steps)
{
    if (steps == 0)
    {
        return;
    }

    // Cancel continuous operation before changing mode.
    if (mode == STEPPER_CONTINUOUS)
    {
        motor.setCurrentPosition(
            motor.currentPosition());
        continuousSpeed = 0.0f;
    }

    prepareMovement();

    mode = STEPPER_POSITION;

    motor.move(steps);
}

// ============================================================
// ABSOLUTE MOVEMENT
// ============================================================

void StepperModule::moveAbsolute(long position)
{
    if (
        mode != STEPPER_CONTINUOUS &&
        position == motor.currentPosition())
    {
        return;
    }

    if (mode == STEPPER_CONTINUOUS)
    {
        motor.setCurrentPosition(
            motor.currentPosition());
        continuousSpeed = 0.0f;
    }

    prepareMovement();

    mode = STEPPER_POSITION;

    motor.moveTo(position);
}

// ============================================================
// CONTINUOUS ROTATION
// ============================================================

void StepperModule::rotateContinuous(float speed)
{
    if (!isfinite(speed))
    {
        return;
    }

    if (speed == 0.0f)
    {
        stopContinuous();
        return;
    }

    // Limit speed to configured maximum.
    speed = constrain(
        speed,
        -maxSpeed,
        maxSpeed);

    prepareMovement();

    // Cancel any position target.
    motor.setCurrentPosition(
        motor.currentPosition());

    continuousSpeed = speed;

    motor.setSpeed(continuousSpeed);

    mode = STEPPER_CONTINUOUS;
}

// ============================================================
// STOP CONTINUOUS ROTATION
// ============================================================

void StepperModule::stopContinuous()
{
    if (mode != STEPPER_CONTINUOUS)
    {
        return;
    }

    // runSpeed() uses a constant velocity.
    // Stop pulses immediately.
    continuousSpeed = 0.0f;

    motor.setSpeed(0.0f);

    mode = STEPPER_IDLE;

    sleep();
}

// ============================================================
// CONTINUOUS STATE
// ============================================================

bool StepperModule::isContinuous()
{
    return mode == STEPPER_CONTINUOUS;
}

// ============================================================
// NORMAL STOP
// ============================================================

void StepperModule::stop()
{
    if (mode == STEPPER_CONTINUOUS)
    {
        stopContinuous();
        return;
    }

    if (mode == STEPPER_POSITION)
    {

        // AccelStepper decelerates toward a stop.
        motor.stop();

        if (motor.distanceToGo() == 0)
        {
            finishMovement();
        }

        return;
    }

    sleep();
}

// ============================================================
// EMERGENCY STOP
// ============================================================

void StepperModule::emergencyStop()
{
    // Immediately cancel all motion commands.
    motor.setCurrentPosition(
        motor.currentPosition());

    continuousSpeed = 0.0f;
    mode = STEPPER_IDLE;

    sleep();
}

// ============================================================
// MAXIMUM SPEED
// ============================================================

void StepperModule::setMaxSpeed(float speed)
{
    if (!isfinite(speed) || speed <= 0.0f)
    {
        return;
    }

    maxSpeed = speed;

    motor.setMaxSpeed(maxSpeed);

    if (mode == STEPPER_CONTINUOUS)
    {

        continuousSpeed = constrain(
            continuousSpeed,
            -maxSpeed,
            maxSpeed);

        motor.setSpeed(continuousSpeed);
    }
}

// ============================================================
// ACCELERATION
// ============================================================

void StepperModule::setAcceleration(float value)
{
    if (!isfinite(value) || value <= 0.0f)
    {
        return;
    }

    acceleration = value;

    motor.setAcceleration(acceleration);
}

// ============================================================
// SPEED GETTERS
// ============================================================

float StepperModule::getMaxSpeed()
{
    return maxSpeed;
}

float StepperModule::getAcceleration()
{
    return acceleration;
}

float StepperModule::getContinuousSpeed()
{
    return continuousSpeed;
}

// ============================================================
// POSITION GETTERS
// ============================================================

long StepperModule::getCurrentPosition()
{
    return motor.currentPosition();
}

long StepperModule::getTargetPosition()
{
    return motor.targetPosition();
}

long StepperModule::getDistanceToGo()
{
    return motor.distanceToGo();
}

// ============================================================
// MOVEMENT STATUS
// ============================================================

bool StepperModule::isMoving()
{
    if (mode == STEPPER_CONTINUOUS)
    {
        return continuousSpeed != 0.0f;
    }

    if (mode == STEPPER_POSITION)
    {
        return motor.distanceToGo() != 0;
    }

    return false;
}

// ============================================================
// MODE GETTER
// ============================================================

StepperMode StepperModule::getMode()
{
    return mode;
}

// ============================================================
// SET CURRENT POSITION
// ============================================================

void StepperModule::setCurrentPosition(long position)
{
    // Re-zeroing while moving is not allowed.
    if (isMoving())
    {
        return;
    }

    motor.setCurrentPosition(position);

    // Position update does not require energizing the driver.
    mode = STEPPER_IDLE;
}

// ============================================================
// MAIN MOTOR UPDATE
// ============================================================

void StepperModule::run()
{
    if (!driverAwake)
    {
        return;
    }

    switch (mode)
    {

    case STEPPER_POSITION:

        motor.run();

        if (motor.distanceToGo() == 0)
        {
            finishMovement();
        }

        break;

    case STEPPER_CONTINUOUS:

        // Constant-speed, non-blocking stepping.
        motor.runSpeed();

        break;

    case STEPPER_IDLE:
    default:

        sleep();
        break;
    }
}

// ============================================================
// STATUS
// ============================================================

void StepperModule::printStatus(Stream &output)
{
    output.println();
    output.println("========== STEPPER ==========");

    output.print("Driver: ");
    output.println(
        driverAwake ? "AWAKE" : "SLEEP");

    output.print("Mode: ");

    switch (mode)
    {

    case STEPPER_POSITION:
        output.println("POSITION");
        break;

    case STEPPER_CONTINUOUS:
        output.println("CONTINUOUS");
        break;

    default:
        output.println("IDLE");
        break;
    }

    output.print("Moving: ");
    output.println(isMoving() ? "YES" : "NO");

    output.print("Current position: ");
    output.println(getCurrentPosition());

    output.print("Target position: ");
    output.println(getTargetPosition());

    output.print("Distance to go: ");
    output.println(getDistanceToGo());

    output.print("Maximum speed: ");
    output.println(maxSpeed);

    output.print("Acceleration: ");
    output.println(acceleration);

    output.print("Continuous speed: ");
    output.println(continuousSpeed);

    output.println("=============================");
}
