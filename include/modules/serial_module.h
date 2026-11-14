
#pragma once

#include <Arduino.h>

#include "modules/stepper_module.h"
#include "modules/cooling_stack.h"
#include "modules/loadcell_module.h"
#include "modules/rfid_module.h"
#include "modules/servo_module.h"
#include "modules/temperature_module.h"
#include "modules/switch_module.h"
// #include "modules/schedule_module.h"

struct SerialConfig
{
    uint32_t baudRate = 115200;
    uint16_t bufferLimit = 160;
    bool echoCommands = true;
    bool showReadyMessage = true;
};

class SerialModule
{
public:
    explicit SerialModule(
        StepperModule &stepper,
        CoolingStack &cooling,
        LoadCellModule &loadcells,
        RFIDModule &rfid,
        ServoModule &servo,
        TemperatureModule &temperature,
        SwitchModule &switches,
        // ScheduleModule &schedule,
        const SerialConfig &config = SerialConfig());

    void begin(HardwareSerial &serial);
    void update();

    bool isInitialized() const;
    void printStatus(Stream &output) const;

private:
    HardwareSerial *serial_ = nullptr;

    StepperModule &stepper_;
    CoolingStack &cooling_;
    LoadCellModule &loadcells_;
    RFIDModule &rfid_;
    ServoModule &servo_;
    TemperatureModule &temperature_;
    SwitchModule &switches_;
    // ScheduleModule &schedule_;

    SerialConfig config_;

    String inputBuffer_;
    bool initialized_ = false;

    static SerialModule *instance_;

    // Serial input management
    void readSerial();
    void processLine(String command);
    void printPrompt();

    // Parsing helpers
    static String getToken(const String &input, uint8_t index);
    static bool hasToken(const String &input, uint8_t index);
    static bool parseLong(const String &value, long &result);
    static bool parseFloat(const String &value, float &result);
    static bool parseUnsigned(const String &value, uint32_t &result);
    static bool parseOnOff(const String &value, bool &result);

    // Command dispatch
    void executeCommand(const String &command);
    void executeStepperCommand(const String &command);
    void executeCoolingCommand(const String &command);
    void executeLoadCellCommand(const String &command);
    void executeRFIDCommand(const String &command);
    void executeServoCommand(const String &command);
    void executeTemperatureCommand(const String &command);
    void executeSwitchCommand(const String &command);
    void executeScheduleCommand(const String &command);

    // Global commands
    void commandHelp();
    void commandStatus();
    void commandOff();

    // Module output helpers
    void printTemperatureReading(TemperatureCluster id);
    void printLoadCellReading(LoadCellID id);

    // Switch event reporting
    static void switchEventCallback(const SwitchEvent &event);
};
