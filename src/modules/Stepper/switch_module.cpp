#include "modules/switch_module.h"

SwitchModule::SwitchModule(const SwitchConfig &config)
    : config(config),
      callback(nullptr),
      initialized(false)
{

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        states[i].rawActive = false;
        states[i].stableActive = false;
        states[i].lastRawChange = 0;
    }
}

// ============================================================
// INITIALIZATION
// ============================================================

bool SwitchModule::begin()
{
    if (initialized)
    {
        return true;
    }

    if (config.debounceMs == 0)
    {
        Serial.println("[SWITCH] Invalid debounce configuration.");
        return false;
    }

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        // Active LOW switches use internal pull-ups.
        // Active HIGH switches use internal pull-downs.
        if (config.activeLevels[i] == LOW)
        {
            pinMode(config.pins[i], INPUT_PULLUP);
        }
        else
        {
            pinMode(config.pins[i], INPUT_PULLDOWN);
        }
    }

    // Allow input levels to settle.
    delay(5);

    uint32_t now = millis();

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        bool initialState = readActiveState(i);

        states[i].rawActive = initialState;
        states[i].stableActive = initialState;
        states[i].lastRawChange = now;

        Serial.printf(
            "[SWITCH] %s initialized on GPIO%u: %s\n",
            config.names[i],
            config.pins[i],
            initialState ? "ACTIVE" : "INACTIVE");
    }

    initialized = true;

    Serial.printf(
        "[SWITCH] Module initialized. Debounce: %u ms\n",
        config.debounceMs);

    return true;
}

// ============================================================
// INDEX AND INPUT HELPERS
// ============================================================

int8_t SwitchModule::getIndex(SwitchID id) const
{
    uint8_t value = static_cast<uint8_t>(id);

    if (value < 1 || value > SWITCH_COUNT)
    {
        return -1;
    }

    return static_cast<int8_t>(value - 1);
}

bool SwitchModule::readActiveState(uint8_t index) const
{
    int level = digitalRead(config.pins[index]);

    return level == config.activeLevels[index];
}

// ============================================================
// CALLBACK
// ============================================================

void SwitchModule::setCallback(SwitchEventCallback callbackFunction)
{
    callback = callbackFunction;
}

// ============================================================
// DEBOUNCING
// ============================================================

void SwitchModule::processSwitch(uint8_t index, uint32_t now)
{
    bool currentRawState = readActiveState(index);

    // Detect a raw electrical transition.
    if (currentRawState != states[index].rawActive)
    {
        states[index].rawActive = currentRawState;
        states[index].lastRawChange = now;
    }

    // Wait until the raw input has remained stable.
    if ((uint32_t)(now - states[index].lastRawChange) <
        config.debounceMs)
    {
        return;
    }

    // No confirmed state change.
    if (states[index].stableActive == states[index].rawActive)
    {
        return;
    }

    // Confirm the debounced state.
    states[index].stableActive = states[index].rawActive;

    SwitchEvent event;

    event.id = static_cast<SwitchID>(index + 1);
    event.pin = config.pins[index];
    event.name = config.names[index];
    event.active = states[index].stableActive;
    event.timestamp = now;

    // Broadcast the event to the application.
    if (callback != nullptr)
    {
        callback(event);
    }
}

// ============================================================
// UPDATE
// ============================================================

void SwitchModule::update()
{
    if (!initialized)
    {
        return;
    }

    uint32_t now = millis();

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        processSwitch(i, now);
    }
}

// ============================================================
// STATE ACCESS
// ============================================================

bool SwitchModule::isActive(SwitchID id) const
{
    int8_t index = getIndex(id);

    if (index < 0 || !initialized)
    {
        return false;
    }

    return states[index].stableActive;
}

uint8_t SwitchModule::getPin(SwitchID id) const
{
    int8_t index = getIndex(id);

    if (index < 0)
    {
        return 255;
    }

    return config.pins[index];
}

// ============================================================
// STATUS
// ============================================================

void SwitchModule::printStatus(Stream &output) const
{
    output.println("\n========== SWITCH STATUS ==========");

    output.printf(
        "Initialized: %s\n",
        initialized ? "YES" : "NO");

    output.printf(
        "Debounce: %u ms\n",
        config.debounceMs);

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        output.printf(
            "%s | GPIO%u | %s | Active level: %s\n",
            config.names[i],
            config.pins[i],
            states[i].stableActive ? "ACTIVE" : "INACTIVE",
            config.activeLevels[i] == HIGH ? "HIGH" : "LOW");
    }

    output.println("===================================");
}