#pragma once

#include <Arduino.h>

static constexpr uint8_t SWITCH_COUNT = 6;

enum SwitchID : uint8_t
{
    SWITCH_1 = 1,
    SWITCH_2 = 2,
    SWITCH_3 = 3,
    SWITCH_4 = 4,
    SWITCH_5 = 5,
    SWITCH_6 = 6
};

struct SwitchConfig
{
    uint8_t pins[SWITCH_COUNT] = {
        9, 10, 11, 12, 13, 14};

    // Electrical level that represents an activated switch.
    // GPIO3 uses HIGH; all other switches use LOW.
    uint8_t activeLevels[SWITCH_COUNT] = {
        LOW, LOW, LOW, LOW, LOW, HIGH};

    const char *names[SWITCH_COUNT] = {
        "SW1", "SW2", "SW3", "SW4", "SW5", "SW6"};

    uint16_t debounceMs = 30;
};

struct SwitchEvent
{
    SwitchID id;
    uint8_t pin;
    const char *name;
    bool active;
    uint32_t timestamp;
};

using SwitchEventCallback = void (*)(const SwitchEvent &event);

class SwitchModule
{
public:
    explicit SwitchModule(
        const SwitchConfig &config = SwitchConfig());

    bool begin();

    // Call continuously from loop().
    void update();

    // Register a callback for confirmed state changes.
    void setCallback(SwitchEventCallback callback);

    // Read the current debounced state.
    bool isActive(SwitchID id) const;

    // Retrieve the GPIO assigned to a switch.
    uint8_t getPin(SwitchID id) const;

    // Print the state of all six switches.
    void printStatus(Stream &output) const;

private:
    struct SwitchState
    {
        bool rawActive;
        bool stableActive;
        uint32_t lastRawChange;
    };

    SwitchConfig config;
    SwitchState states[SWITCH_COUNT];

    SwitchEventCallback callback;
    bool initialized;

    int8_t getIndex(SwitchID id) const;

    bool readActiveState(uint8_t index) const;

    void processSwitch(uint8_t index, uint32_t now);
};