
#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>

#include "modules/stepper_module.h"
#include "modules/cooling_stack.h"
#include "modules/loadcell_module.h"
#include "modules/rfid_module.h"
#include "modules/servo_module.h"
#include "modules/temperature_module.h"
#include "modules/switch_module.h"

// ============================================================
// MODULE INSTANCES
// ============================================================

StepperModule stepper;
CoolingStack cooling;
LoadCellModule loadcells;
RFIDModule rfid;
ServoModule servo;
TemperatureModule temperature;
SwitchModule switches;

// ============================================================
// SERIAL CONFIGURATION
// ============================================================

String serialBuffer;

static constexpr size_t SERIAL_BUFFER_LIMIT = 128;

// ============================================================
// SERIAL TOKENIZER
// ============================================================

String getToken(const String &input, uint8_t index)
{
    uint8_t current = 0;
    int start = 0;
    int length = input.length();

    while (start < length)
    {

        while (
            start < length &&
            input[start] == ' ')
        {
            start++;
        }

        if (start >= length)
        {
            break;
        }

        int end = input.indexOf(' ', start);

        if (end < 0)
        {
            end = length;
        }

        if (current == index)
        {
            return input.substring(start, end);
        }

        current++;
        start = end + 1;
    }

    return "";
}

// ============================================================
// PARSING HELPERS
// ============================================================

bool parseLong(const String &input, long &result)
{
    if (input.length() == 0)
    {
        return false;
    }

    errno = 0;

    char *end = nullptr;

    long value = strtol(
        input.c_str(),
        &end,
        10);

    if (
        errno == ERANGE ||
        end == input.c_str() ||
        *end != '\0')
    {
        return false;
    }

    result = value;
    return true;
}

bool parseFloat(const String &input, float &result)
{
    if (input.length() == 0)
    {
        return false;
    }

    char *end = nullptr;

    float value = strtof(
        input.c_str(),
        &end);

    if (
        end == input.c_str() ||
        *end != '\0' ||
        !isfinite(value))
    {
        return false;
    }

    result = value;
    return true;
}

bool parseUnsignedByte(
    const String &input,
    uint8_t &result)
{
    long value;

    if (!parseLong(input, value))
    {
        return false;
    }

    if (value < 0 || value > 255)
    {
        return false;
    }

    result = static_cast<uint8_t>(value);
    return true;
}

bool parseOnOff(
    const String &input,
    bool &result)
{
    if (input == "on" || input == "1")
    {
        result = true;
        return true;
    }

    if (input == "off" || input == "0")
    {
        result = false;
        return true;
    }

    return false;
}

// ============================================================
// MODULE IDENTIFIERS
// ============================================================

bool parseLoadCellID(
    const String &token,
    LoadCellID &id)
{
    if (token == "1")
    {
        id = LoadCellID::HX1;
        return true;
    }

    if (token == "2")
    {
        id = LoadCellID::HX2;
        return true;
    }

    return false;
}

bool parseTemperatureID(
    const String &token,
    TemperatureCluster &id)
{
    if (token == "1")
    {
        id = TEMP1;
        return true;
    }

    if (token == "2")
    {
        id = TEMP2;
        return true;
    }

    if (token == "3")
    {
        id = TEMP3;
        return true;
    }

    return false;
}

// ============================================================
// SWITCH EVENT CALLBACK
// ============================================================

void onSwitchChanged(const SwitchEvent &event)
{
    Serial.printf(
        "[SWITCH EVENT] ID=%u | %s | GPIO%u | %s | TIME=%lu ms\n",
        static_cast<unsigned>(event.id),
        event.name,
        static_cast<unsigned>(event.pin),
        event.active ? "ACTIVE" : "INACTIVE",
        static_cast<unsigned long>(event.timestamp));
}

// ============================================================
// SYSTEM STATUS
// ============================================================

void systemStatus()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("          SYSTEM STATUS         ");
    Serial.println("================================");

    stepper.printStatus(Serial);
    cooling.printStatus(Serial);
    loadcells.printStatus(Serial);
    rfid.printStatus(Serial);
    servo.printStatus(Serial);
    temperature.printStatus(Serial);
    switches.printStatus(Serial);

    Serial.println("================================");
}

// ============================================================
// GLOBAL OFF
// ============================================================

void systemOff()
{
    // Immediate cancellation of stepper movement.
    stepper.emergencyStop();

    cooling.allOff();

    loadcells.setContinuous(false);

    rfid.setScanning(false);

    servo.stop();

    temperature.setContinuous(TEMP1, false);
    temperature.setContinuous(TEMP2, false);
    temperature.setContinuous(TEMP3, false);

    // Switch monitoring stays enabled.
    Serial.println("[SYSTEM] All actuators stopped.");
    Serial.println("[SYSTEM] Stepper driver sleeping.");
}

// ============================================================
// HELP
// ============================================================

void printHelp()
{
    Serial.println();
    Serial.println("========== COMMANDS ==========");

    Serial.println();
    Serial.println("SYSTEM:");
    Serial.println("  status");
    Serial.println("  off");
    Serial.println("  help");

    Serial.println();
    Serial.println("STEPPER:");
    Serial.println("  step status");
    Serial.println("  step move <steps>");
    Serial.println("  step relative <steps>");
    Serial.println("  step absolute <position>");
    Serial.println("  step continuous <speed>");
    Serial.println("  step rotate <speed>");
    Serial.println("  step stop");
    Serial.println("  step emergency");
    Serial.println("  step sleep");
    Serial.println("  step wake");
    Serial.println("  step speed <steps/s>");
    Serial.println("  step accel <steps/s2>");
    Serial.println("  step zero");
    Serial.println("  step position");

    Serial.println();
    Serial.println("COOLING:");
    Serial.println("  cool status");
    Serial.println("  cool peltier <0-100>");
    Serial.println("  cool fan <0-100>");
    Serial.println("  cool pump <0-100>");
    Serial.println("  cool off");

    Serial.println();
    Serial.println("LOAD CELLS:");
    Serial.println("  load status");
    Serial.println("  load read [samples]");
    Serial.println("  load enable <1|2>");
    Serial.println("  load disable <1|2>");
    Serial.println("  load continuous on|off");
    Serial.println("  load tare <1|2> [samples]");
    Serial.println("  load calibrate <1|2> <weight>");
    Serial.println("  load scale <1|2> <factor>");

    Serial.println();
    Serial.println("RFID:");
    Serial.println("  rfid status");
    Serial.println("  rfid on");
    Serial.println("  rfid off");
    Serial.println("  rfid uid");

    Serial.println();
    Serial.println("SERVO:");
    Serial.println("  servo status");
    Serial.println("  servo speed <-100..100>");
    Serial.println("  servo rotate <speed> <duration_ms>");
    Serial.println("  servo stop");

    Serial.println();
    Serial.println("TEMPERATURE:");
    Serial.println("  temp status");
    Serial.println("  temp read <1|2|3|all> [samples]");
    Serial.println("  temp continuous <1|2|3|all> on|off");
    Serial.println("  temp calibrate <1|2|3> <temp_C>");
    Serial.println("  temp adjust <1|2|3> <offset> <gain>");

    Serial.println();
    Serial.println("SWITCHES:");
    Serial.println("  switch status");
    Serial.println("  switch read");

    Serial.println("==============================");
}

// ============================================================
// STEPPER COMMAND HANDLER
// ============================================================

void handleStepperCommand(const String &command)
{
    String action = getToken(command, 1);
    String arg1 = getToken(command, 2);

    if (action == "status")
    {
        stepper.printStatus(Serial);
        return;
    }

    if (
        action == "move" ||
        action == "relative")
    {
        long steps;

        if (!parseLong(arg1, steps))
        {
            Serial.println("Usage: step move <steps>");
            return;
        }

        stepper.moveRelative(steps);

        Serial.printf(
            "[STEPPER] Relative move: %ld steps\n",
            steps);

        return;
    }

    if (action == "absolute")
    {

        long position;

        if (!parseLong(arg1, position))
        {
            Serial.println(
                "Usage: step absolute <position>");
            return;
        }

        stepper.moveAbsolute(position);

        Serial.printf(
            "[STEPPER] Absolute target: %ld\n",
            position);

        return;
    }

    // --------------------------------------------------------
    // CONTINUOUS ROTATION
    // --------------------------------------------------------

    if (
        action == "continuous" ||
        action == "rotate")
    {
        float speed;

        if (!parseFloat(arg1, speed))
        {
            Serial.println(
                "Usage: step continuous <steps_per_second>");
            return;
        }

        if (speed == 0.0f)
        {

            stepper.stopContinuous();

            Serial.println(
                "[STEPPER] Continuous rotation stopped.");

            return;
        }

        stepper.rotateContinuous(speed);

        Serial.printf(
            "[STEPPER] Continuous rotation: %.2f steps/s\n",
            stepper.getContinuousSpeed());

        return;
    }

    // --------------------------------------------------------
    // NORMAL STOP
    // --------------------------------------------------------

    if (action == "stop")
    {

        stepper.stop();

        Serial.println(
            "[STEPPER] Stop requested.");

        return;
    }

    // --------------------------------------------------------
    // EMERGENCY STOP
    // --------------------------------------------------------

    if (
        action == "emergency" ||
        action == "estop")
    {
        stepper.emergencyStop();

        Serial.println(
            "[STEPPER] EMERGENCY STOP.");

        return;
    }

    // --------------------------------------------------------
    // DRIVER POWER
    // --------------------------------------------------------

    if (action == "sleep")
    {

        stepper.sleep();

        Serial.println(
            "[STEPPER] Driver sleeping.");

        return;
    }

    if (action == "wake")
    {

        stepper.wake();

        Serial.println(
            "[STEPPER] Driver awake.");

        return;
    }

    // --------------------------------------------------------
    // SPEED
    // --------------------------------------------------------

    if (action == "speed")
    {

        float speed;

        if (
            !parseFloat(arg1, speed) ||
            speed <= 0.0f)
        {
            Serial.println(
                "Usage: step speed <positive_steps_per_second>");
            return;
        }

        stepper.setMaxSpeed(speed);

        Serial.printf(
            "[STEPPER] Maximum speed: %.2f steps/s\n",
            stepper.getMaxSpeed());

        return;
    }

    // --------------------------------------------------------
    // ACCELERATION
    // --------------------------------------------------------

    if (
        action == "accel" ||
        action == "acceleration")
    {
        float acceleration;

        if (
            !parseFloat(arg1, acceleration) ||
            acceleration <= 0.0f)
        {
            Serial.println(
                "Usage: step accel <positive_steps_per_second_squared>");
            return;
        }

        stepper.setAcceleration(acceleration);

        Serial.printf(
            "[STEPPER] Acceleration: %.2f steps/s2\n",
            stepper.getAcceleration());

        return;
    }

    // --------------------------------------------------------
    // POSITION
    // --------------------------------------------------------

    if (action == "position")
    {

        Serial.printf(
            "[STEPPER] Current: %ld | Target: %ld | Remaining: %ld\n",
            stepper.getCurrentPosition(),
            stepper.getTargetPosition(),
            stepper.getDistanceToGo());

        return;
    }

    if (action == "zero")
    {

        if (stepper.isMoving())
        {
            Serial.println(
                "[STEPPER] Stop movement before zeroing.");
            return;
        }

        stepper.setCurrentPosition(0);

        Serial.println(
            "[STEPPER] Position set to zero.");

        return;
    }

    Serial.println("[STEPPER] Unknown command.");
}

// ============================================================
// COOLING COMMAND HANDLER
// ============================================================

void handleCoolingCommand(const String &command)
{
    String action = getToken(command, 1);
    String arg1 = getToken(command, 2);

    if (action == "status")
    {
        cooling.printStatus(Serial);
        return;
    }

    if (action == "off")
    {
        cooling.allOff();
        Serial.println("[COOLING] All outputs OFF.");
        return;
    }

    float duty;

    if (!parseFloat(arg1, duty))
    {
        Serial.println(
            "Usage: cool <peltier|fan|pump> <0-100>");
        return;
    }

    duty = constrain(duty, 0.0f, 100.0f);

    if (action == "peltier")
    {
        cooling.setPeltier(duty);
    }
    else if (action == "fan")
    {
        cooling.setFan(duty);
    }
    else if (action == "pump")
    {
        cooling.setPump(duty);
    }
    else
    {
        Serial.println("[COOLING] Unknown command.");
        return;
    }

    Serial.printf(
        "[COOLING] %s = %.1f%%\n",
        action.c_str(),
        duty);
}

// ============================================================
// LOAD CELL COMMAND HANDLER
// ============================================================

void handleLoadCommand(const String &command)
{
    String action = getToken(command, 1);
    String arg1 = getToken(command, 2);
    String arg2 = getToken(command, 3);

    if (action == "status")
    {
        loadcells.printStatus(Serial);
        return;
    }

    if (action == "continuous")
    {

        bool enabled;

        if (!parseOnOff(arg1, enabled))
        {
            Serial.println(
                "Usage: load continuous on|off");
            return;
        }

        loadcells.setContinuous(enabled);

        Serial.println(
            enabled ? "[LOAD] Continuous ON." : "[LOAD] Continuous OFF.");

        return;
    }

    if (action == "read")
    {

        uint8_t samples = 1;

        if (
            arg1.length() > 0 &&
            (!parseUnsignedByte(arg1, samples) ||
             samples == 0))
        {
            Serial.println(
                "Usage: load read [samples]");
            return;
        }

        loadcells.acquire(samples);
        loadcells.printStatus(Serial);

        return;
    }

    LoadCellID id;

    if (!parseLoadCellID(arg1, id))
    {
        Serial.println(
            "Usage: load <enable|disable|tare|calibrate|scale> <1|2> ...");
        return;
    }

    if (action == "enable")
    {

        loadcells.enable(id);

        Serial.println("[LOAD] Enabled.");
        return;
    }

    if (action == "disable")
    {

        loadcells.disable(id);

        Serial.println("[LOAD] Disabled.");
        return;
    }

    if (action == "tare")
    {

        uint8_t samples = 10;

        if (
            arg2.length() > 0 &&
            (!parseUnsignedByte(arg2, samples) ||
             samples == 0))
        {
            Serial.println(
                "Usage: load tare <1|2> [samples]");
            return;
        }

        loadcells.tare(id, samples);

        Serial.println("[LOAD] Tare completed.");
        return;
    }

    if (action == "calibrate")
    {

        float knownWeight;

        if (!parseFloat(arg2, knownWeight))
        {
            Serial.println(
                "Usage: load calibrate <1|2> <weight>");
            return;
        }

        loadcells.calibrate(id, knownWeight);

        Serial.println(
            "[LOAD] Calibration requested.");

        return;
    }

    if (action == "scale")
    {

        float scale;

        if (
            !parseFloat(arg2, scale) ||
            scale == 0.0f)
        {
            Serial.println(
                "Usage: load scale <1|2> <nonzero_factor>");
            return;
        }

        loadcells.setScale(id, scale);

        Serial.println("[LOAD] Scale updated.");
        return;
    }

    Serial.println("[LOAD] Unknown command.");
}

// ============================================================
// RFID COMMAND HANDLER
// ============================================================

void handleRFIDCommand(const String &command)
{
    String action = getToken(command, 1);

    if (action == "status")
    {

        rfid.printStatus(Serial);
        return;
    }

    if (action == "on")
    {

        rfid.setScanning(true);

        Serial.println("[RFID] Scanning enabled.");
        return;
    }

    if (action == "off")
    {

        rfid.setScanning(false);

        Serial.println("[RFID] Scanning disabled.");
        return;
    }

    if (action == "uid")
    {

        Serial.print("[RFID] Last UID: ");
        Serial.println(rfid.getLastUID());

        return;
    }

    Serial.println("[RFID] Unknown command.");
}

// ============================================================
// SERVO COMMAND HANDLER
// ============================================================

void handleServoCommand(const String &command)
{
    String action = getToken(command, 1);
    String arg1 = getToken(command, 2);
    String arg2 = getToken(command, 3);

    if (action == "status")
    {

        servo.printStatus(Serial);
        return;
    }

    if (action == "stop")
    {

        servo.stop();

        Serial.println("[SERVO] Stopped.");
        return;
    }

    if (action == "speed")
    {

        float speed;

        if (!parseFloat(arg1, speed))
        {
            Serial.println(
                "Usage: servo speed <-100..100>");
            return;
        }

        servo.setSpeed(
            constrain(speed, -100.0f, 100.0f));

        Serial.println("[SERVO] Speed updated.");
        return;
    }

    if (action == "rotate")
    {

        float speed;
        long duration;

        if (
            !parseFloat(arg1, speed) ||
            !parseLong(arg2, duration) ||
            duration <= 0)
        {
            Serial.println(
                "Usage: servo rotate <speed> <duration_ms>");
            return;
        }

        servo.rotateFor(
            constrain(speed, -100.0f, 100.0f),
            static_cast<uint32_t>(duration));

        Serial.println(
            "[SERVO] Timed rotation started.");

        return;
    }

    Serial.println("[SERVO] Unknown command.");
}

// ============================================================
// TEMPERATURE COMMAND HANDLER
// ============================================================

void handleTemperatureCommand(const String &command)
{
    String action = getToken(command, 1);
    String arg1 = getToken(command, 2);
    String arg2 = getToken(command, 3);
    String arg3 = getToken(command, 4);

    if (action == "status")
    {

        temperature.printStatus(Serial);
        return;
    }

    if (action == "read")
    {

        uint8_t samples = 1;

        if (
            arg2.length() > 0 &&
            (!parseUnsignedByte(arg2, samples) ||
             samples == 0))
        {
            Serial.println(
                "Usage: temp read <1|2|3|all> [samples]");
            return;
        }

        if (arg1 == "all")
        {

            temperature.sampleAll(samples);
            temperature.printStatus(Serial);

            return;
        }

        TemperatureCluster id;

        if (!parseTemperatureID(arg1, id))
        {
            Serial.println(
                "Usage: temp read <1|2|3|all> [samples]");
            return;
        }

        temperature.sample(id, samples);
        temperature.printStatus(Serial);

        return;
    }

    if (action == "continuous")
    {

        bool enabled;

        if (!parseOnOff(arg2, enabled))
        {
            Serial.println(
                "Usage: temp continuous <1|2|3|all> on|off");
            return;
        }

        if (arg1 == "all")
        {

            temperature.setContinuous(TEMP1, enabled);
            temperature.setContinuous(TEMP2, enabled);
            temperature.setContinuous(TEMP3, enabled);
        }
        else
        {

            TemperatureCluster id;

            if (!parseTemperatureID(arg1, id))
            {
                Serial.println(
                    "Invalid temperature cluster.");
                return;
            }

            temperature.setContinuous(id, enabled);
        }

        Serial.println(
            enabled ? "[TEMP] Continuous enabled." : "[TEMP] Continuous disabled.");

        return;
    }

    TemperatureCluster id;

    if (!parseTemperatureID(arg1, id))
    {
        Serial.println(
            "Invalid temperature cluster.");
        return;
    }

    if (action == "calibrate")
    {

        float knownTemperature;

        if (!parseFloat(arg2, knownTemperature))
        {
            Serial.println(
                "Usage: temp calibrate <1|2|3> <temp_C>");
            return;
        }

        temperature.calibrate(
            id,
            knownTemperature);

        Serial.println(
            "[TEMP] Calibration requested.");

        return;
    }

    if (action == "adjust")
    {

        float offset;
        float gain;

        if (
            !parseFloat(arg2, offset) ||
            !parseFloat(arg3, gain) ||
            gain <= 0.0f)
        {
            Serial.println(
                "Usage: temp adjust <1|2|3> <offset> <positive_gain>");
            return;
        }

        temperature.setCalibration(
            id,
            offset,
            gain);

        Serial.println(
            "[TEMP] Calibration adjusted.");

        return;
    }

    Serial.println("[TEMP] Unknown command.");
}

// ============================================================
// SWITCH COMMAND HANDLER
// ============================================================

void handleSwitchCommand(const String &command)
{
    String action = getToken(command, 1);

    if (
        action == "status" ||
        action == "read")
    {
        switches.printStatus(Serial);
        return;
    }

    Serial.println("[SWITCH] Unknown command.");
}

// ============================================================
// MAIN COMMAND ROUTER
// ============================================================

void processCommand(String command)
{
    command.trim();
    command.toLowerCase();

    if (command.length() == 0)
    {
        return;
    }

    String module = getToken(command, 0);

    if (module == "help")
    {

        printHelp();
        return;
    }

    if (module == "status")
    {

        systemStatus();
        return;
    }

    if (module == "off")
    {

        systemOff();
        return;
    }

    if (module == "step")
    {

        handleStepperCommand(command);
        return;
    }

    if (module == "cool")
    {

        handleCoolingCommand(command);
        return;
    }

    if (module == "load")
    {

        handleLoadCommand(command);
        return;
    }

    if (module == "rfid")
    {

        handleRFIDCommand(command);
        return;
    }

    if (module == "servo")
    {

        handleServoCommand(command);
        return;
    }

    if (module == "temp")
    {

        handleTemperatureCommand(command);
        return;
    }

    if (module == "switch")
    {

        handleSwitchCommand(command);
        return;
    }

    Serial.print("[SYSTEM] Unknown command: ");
    Serial.println(command);
}

// ============================================================
// SERIAL INPUT PROCESSING
// ============================================================

void processSerial()
{
    while (Serial.available() > 0)
    {

        char character = static_cast<char>(
            Serial.read());

        if (
            character == '\n' ||
            character == '\r')
        {
            if (serialBuffer.length() > 0)
            {

                processCommand(serialBuffer);

                serialBuffer = "";
            }

            continue;
        }

        if (serialBuffer.length() >= SERIAL_BUFFER_LIMIT)
        {

            serialBuffer = "";

            Serial.println(
                "[SERIAL] Command buffer overflow.");

            continue;
        }

        serialBuffer += character;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("================================");
    Serial.println("      ESP32-S3 CONTROLLER       ");
    Serial.println("================================");

    // --------------------------------------------------------
    // STEPPER
    // --------------------------------------------------------

    stepper.begin();

    Serial.println("[INIT] Stepper initialized.");

    // --------------------------------------------------------
    // COOLING
    // --------------------------------------------------------

    cooling.begin();

    Serial.println("[INIT] Cooling initialized.");

    // --------------------------------------------------------
    // LOAD CELLS
    // --------------------------------------------------------

    loadcells.begin();

    Serial.println("[INIT] Load cells initialized.");

    // --------------------------------------------------------
    // RFID
    // --------------------------------------------------------

    if (rfid.begin())
    {

        Serial.println("[INIT] RFID initialized.");
    }
    else
    {

        Serial.println("[INIT] RFID initialization failed.");
    }

    // --------------------------------------------------------
    // SERVO
    // --------------------------------------------------------

    servo.begin();

    Serial.println("[INIT] Servo initialized.");

    // --------------------------------------------------------
    // TEMPERATURE
    // --------------------------------------------------------

    temperature.begin();

    Serial.println("[INIT] Temperature initialized.");

    // --------------------------------------------------------
    // SWITCHES
    // --------------------------------------------------------

    switches.setCallback(onSwitchChanged);

    switches.begin();

    Serial.println("[INIT] Switches initialized.");

    // --------------------------------------------------------
    // READY
    // --------------------------------------------------------

    Serial.println();
    Serial.println("[SYSTEM] Ready.");
    Serial.println("[SYSTEM] Type 'help' for commands.");

    printHelp();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Motor stepping must be serviced frequently.
    stepper.run();

    // Remaining modules
    loadcells.update();
    rfid.update();
    servo.update();
    temperature.update();
    switches.update();

    // Non-blocking serial processing
    processSerial();
}
