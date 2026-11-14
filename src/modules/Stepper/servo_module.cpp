#include "modules/servo_module.h"

ServoModule::ServoModule(const ServoConfig &config)
    : config(config),
      initialized(false),
      timedMode(false),
      currentSpeed(0.0f),
      timedStart(0),
      timedDuration(0)
{
}

bool ServoModule::begin()
{
    if (initialized)
    {
        return true;
    }

    if (config.MIN_PULSE_US >= config.NEUTRAL_PULSE_US ||
        config.NEUTRAL_PULSE_US >= config.MAX_PULSE_US)
    {
        Serial.println("[SERVO] Invalid pulse configuration.");
        return false;
    }

    servo.setPeriodHertz(config.FREQUENCY_HZ);

    int channel = servo.attach(
        config.PWM_PIN,
        config.MIN_PULSE_US,
        config.MAX_PULSE_US);

    if (channel < 0)
    {
        Serial.println("[SERVO] Failed to attach servo.");
        return false;
    }

    initialized = true;

    stop();

    Serial.println("[SERVO] Initialized.");
    Serial.printf("[SERVO] PWM GPIO: %u\n", config.PWM_PIN);
    Serial.printf("[SERVO] Frequency: %u Hz\n", config.FREQUENCY_HZ);
    Serial.printf(
        "[SERVO] Pulse range: %u - %u us, neutral: %u us\n",
        config.MIN_PULSE_US,
        config.MAX_PULSE_US,
        config.NEUTRAL_PULSE_US);

    return true;
}

void ServoModule::update()
{
    if (!initialized || !timedMode)
    {
        return;
    }

    // Unsigned subtraction handles millis() rollover.
    if ((uint32_t)(millis() - timedStart) >= timedDuration)
    {
        stop();
        Serial.println("[SERVO] Timed rotation completed.");
    }
}

void ServoModule::setSpeed(float speed)
{
    if (!initialized)
    {
        return;
    }

    // A direct speed command cancels timed operation.
    timedMode = false;
    timedDuration = 0;

    applySpeed(speed);
}

void ServoModule::rotateFor(float speed, uint32_t durationMs)
{
    if (!initialized)
    {
        return;
    }

    if (durationMs == 0 || speed == 0.0f)
    {
        stop();
        return;
    }

    timedMode = true;
    timedStart = millis();
    timedDuration = durationMs;

    applySpeed(speed);

    Serial.printf(
        "[SERVO] Timed rotation started: %.1f%% for %lu ms\n",
        currentSpeed,
        (unsigned long)timedDuration);
}

void ServoModule::stop()
{
    if (!initialized)
    {
        return;
    }

    timedMode = false;
    timedDuration = 0;
    currentSpeed = 0.0f;

    servo.writeMicroseconds(config.NEUTRAL_PULSE_US);
}

bool ServoModule::isInitialized() const
{
    return initialized;
}

bool ServoModule::isRunningTimed() const
{
    return timedMode;
}

float ServoModule::getSpeed() const
{
    return currentSpeed;
}

uint32_t ServoModule::getRemainingTime() const
{
    if (!timedMode)
    {
        return 0;
    }

    uint32_t elapsed = (uint32_t)(millis() - timedStart);

    if (elapsed >= timedDuration)
    {
        return 0;
    }

    return timedDuration - elapsed;
}

float ServoModule::clampSpeed(float speed) const
{
    if (speed > 100.0f)
    {
        return 100.0f;
    }

    if (speed < -100.0f)
    {
        return -100.0f;
    }

    return speed;
}

uint16_t ServoModule::speedToPulse(float speed) const
{
    speed = clampSpeed(speed);

    if (speed > 0.0f)
    {
        float range =
            config.MAX_PULSE_US - config.NEUTRAL_PULSE_US;

        return config.NEUTRAL_PULSE_US +
               (uint16_t)(range * speed / 100.0f);
    }

    if (speed < 0.0f)
    {
        float range =
            config.NEUTRAL_PULSE_US - config.MIN_PULSE_US;

        return config.NEUTRAL_PULSE_US -
               (uint16_t)(range * (-speed) / 100.0f);
    }

    return config.NEUTRAL_PULSE_US;
}

void ServoModule::applySpeed(float speed)
{
    currentSpeed = clampSpeed(speed);

    uint16_t pulse = speedToPulse(currentSpeed);

    servo.writeMicroseconds(pulse);
}

void ServoModule::printStatus(Stream &output) const
{
    output.println("\n========== SERVO STATUS ==========");

    output.printf(
        "Initialized: %s\n",
        initialized ? "YES" : "NO");

    output.printf(
        "PWM GPIO: %u\n",
        config.PWM_PIN);

    output.printf(
        "Current speed: %.1f%%\n",
        currentSpeed);

    output.printf(
        "Direction: %s\n",
        currentSpeed > 0.0f ? "FORWARD" : currentSpeed < 0.0f ? "REVERSE"
                                                              : "STOPPED");

    output.printf(
        "Timed mode: %s\n",
        timedMode ? "ACTIVE" : "INACTIVE");

    if (timedMode)
    {
        output.printf(
            "Remaining time: %lu ms\n",
            (unsigned long)getRemainingTime());
    }

    output.println("==================================");
}