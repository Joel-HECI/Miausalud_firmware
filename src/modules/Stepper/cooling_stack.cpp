#include "modules/cooling_stack.h"

// ==========================================
// CONSTRUCTOR
// ==========================================

CoolingStack::CoolingStack()
    : peltierDuty(0.0f),
      fanDuty(0.0f),
      pumpDuty(0.0f),
      initialized(false)
{
}

// ==========================================
// INITIALIZATION
// ==========================================

bool CoolingStack::begin()
{
    // Initialize PWM peripherals
    bool peltierOK = ledcAttach(
        CoolingConfig::PELTIER_PIN,
        CoolingConfig::PELTIER_FREQ,
        CoolingConfig::PWM_RESOLUTION);

    bool fanOK = ledcAttach(
        CoolingConfig::FAN_PIN,
        CoolingConfig::FAN_FREQ,
        CoolingConfig::PWM_RESOLUTION);

    bool pumpOK = ledcAttach(
        CoolingConfig::PUMP_PIN,
        CoolingConfig::PUMP_FREQ,
        CoolingConfig::PWM_RESOLUTION);

    initialized = peltierOK && fanOK && pumpOK;

    // Always attempt to initialize outputs to OFF
    if (peltierOK)
        ledcWrite(CoolingConfig::PELTIER_PIN, 0);

    if (fanOK)
        ledcWrite(CoolingConfig::FAN_PIN, 0);

    if (pumpOK)
        ledcWrite(CoolingConfig::PUMP_PIN, 0);

    if (!initialized)
    {
        Serial.println("[COOLING] PWM initialization FAILED!");
        return false;
    }

    Serial.println("[COOLING] Stack initialized");
    Serial.println("[COOLING] Peltier: GPIO17");
    Serial.println("[COOLING] Fan: GPIO18");
    Serial.println("[COOLING] Pump: GPIO8");

    allOff();

    return true;
}

// ==========================================
// PWM UTILITIES
// ==========================================

float CoolingStack::clampDuty(float duty)
{
    if (!isfinite(duty))
        return 0.0f;

    return constrain(duty, 0.0f, 100.0f);
}

void CoolingStack::writePWM(uint8_t pin, float duty)
{
    if (!initialized)
        return;

    duty = clampDuty(duty);

    uint16_t pwmValue = static_cast<uint16_t>(
        (duty / 100.0f) * CoolingConfig::PWM_MAX + 0.5f);

    ledcWrite(pin, pwmValue);
}

// ==========================================
// PELTIER CONTROL
// ==========================================

void CoolingStack::setPeltier(float duty)
{
    peltierDuty = clampDuty(duty);

    writePWM(
        CoolingConfig::PELTIER_PIN,
        peltierDuty);
}

void CoolingStack::peltierOff()
{
    setPeltier(0);
}

// ==========================================
// FAN CONTROL
// ==========================================

void CoolingStack::setFan(float duty)
{
    fanDuty = clampDuty(duty);

    writePWM(
        CoolingConfig::FAN_PIN,
        fanDuty);
}

void CoolingStack::fanOff()
{
    setFan(0);
}

// ==========================================
// WATER PUMP CONTROL
// ==========================================

void CoolingStack::setPump(float duty)
{
    pumpDuty = clampDuty(duty);

    writePWM(
        CoolingConfig::PUMP_PIN,
        pumpDuty);
}

void CoolingStack::pumpOff()
{
    setPump(0);
}

// ==========================================
// MASTER CONTROL
// ==========================================

void CoolingStack::allOff()
{
    peltierOff();
    fanOff();
    pumpOff();

    Serial.println("[COOLING] All outputs OFF");
}

// ==========================================
// STATUS GETTERS
// ==========================================

float CoolingStack::getPeltierDuty() const
{
    return peltierDuty;
}

float CoolingStack::getFanDuty() const
{
    return fanDuty;
}

float CoolingStack::getPumpDuty() const
{
    return pumpDuty;
}

bool CoolingStack::isPeltierOn() const
{
    return peltierDuty > 0.0f;
}

bool CoolingStack::isFanOn() const
{
    return fanDuty > 0.0f;
}

bool CoolingStack::isPumpOn() const
{
    return pumpDuty > 0.0f;
}

// ==========================================
// STATUS REPORT
// ==========================================

void CoolingStack::printStatus(Stream &output) const
{
    output.println("\n========== COOLING STACK ==========");

    output.printf(
        "Peltier GPIO %u: %.1f%% [%s]\n",
        CoolingConfig::PELTIER_PIN,
        peltierDuty,
        isPeltierOn() ? "ON" : "OFF");

    output.printf(
        "Fan     GPIO %u: %.1f%% [%s]\n",
        CoolingConfig::FAN_PIN,
        fanDuty,
        isFanOn() ? "ON" : "OFF");

    output.printf(
        "Pump    GPIO %u: %.1f%% [%s]\n",
        CoolingConfig::PUMP_PIN,
        pumpDuty,
        isPumpOn() ? "ON" : "OFF");

    output.printf(
        "Module initialized: %s\n",
        initialized ? "YES" : "NO");

    output.println("===================================");
}