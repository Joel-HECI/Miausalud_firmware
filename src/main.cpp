#include <Arduino.h>

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
// SERIAL INPUT
// ============================================================

String serialBuffer;
const size_t SERIAL_BUFFER_LIMIT = 128;

// ============================================================
// HELPERS
// ============================================================

String getToken(const String &input, uint8_t index)
{
    String token;
    uint8_t currentToken = 0;
    bool insideToken = false;

    for (size_t i = 0; i < input.length(); i++)
    {
        char c = input.charAt(i);

        if (c == ' ' || c == '\t')
        {
            if (insideToken)
            {
                if (currentToken == index)
                {
                    return token;
                }

                currentToken++;
                token = "";
                insideToken = false;
            }
        }
        else
        {
            token += c;
            insideToken = true;
        }
    }

    if (insideToken && currentToken == index)
    {
        return token;
    }

    return "";
}

LoadCellID getLoadCellID(int id)
{
    return (id == 1) ? LoadCellID::HX1 : LoadCellID::HX2;
}

TemperatureCluster getTemperatureCluster(int id)
{
    switch (id)
    {
    case 1:
        return TEMP1;
    case 2:
        return TEMP2;
    case 3:
        return TEMP3;
    default:
        return TEMP1;
    }
}

bool isValidTemperatureCluster(int id)
{
    return id >= 1 && id <= 3;
}

bool isValidLoadCell(int id)
{
    return id == 1 || id == 2;
}

// ============================================================
// SWITCH EVENT CALLBACK
// ============================================================

void onSwitchChanged(const SwitchEvent &event)
{
    Serial.printf(
        "[SWITCH EVENT] ID=%u | %s | GPIO%u | %s | TIME=%lu ms\n",
        (uint8_t)event.id,
        event.name,
        event.pin,
        event.active ? "ACTIVE" : "INACTIVE",
        (unsigned long)event.timestamp);
}

// ============================================================
// READING DISPLAY HELPERS
// ============================================================

void printLoadCellReading(LoadCellID id, const char *name)
{
    LoadCellReading reading = loadcells.getReading(id);

    Serial.printf("\n%s\n", name);

    if (!reading.valid)
    {
        Serial.println("No valid reading available.");
        return;
    }

    Serial.printf("Raw: %ld\n", (long)reading.raw);
    Serial.printf("Weight: %.3f\n", reading.weight);
    Serial.printf(
        "Timestamp: %lu ms\n",
        (unsigned long)reading.timestamp);
}

void printTemperatureReading(
    TemperatureCluster cluster,
    const char *name)
{
    TemperatureReading reading = temperature.getReading(cluster);

    Serial.printf("\n%s\n", name);

    if (!reading.connected)
    {
        Serial.println("Sensor disconnected or input voltage out of range.");
        return;
    }

    if (!reading.valid)
    {
        Serial.println("No valid reading available.");
        return;
    }

    Serial.printf("Raw ADC: %u\n", reading.raw);
    Serial.printf(
        "Voltage: %lu mV\n",
        (unsigned long)reading.millivolts);
    Serial.printf("Temperature: %.2f C\n", reading.temperatureC);
    Serial.printf(
        "Timestamp: %lu ms\n",
        (unsigned long)reading.timestamp);
}

// ============================================================
// SYSTEM STATUS
// ============================================================

void systemStatus()
{
    Serial.println("\n========== SYSTEM STATUS ==========");

    Serial.println("\n[STEPPER]");
    Serial.printf("Awake: %s\n",
                  stepper.isAwake() ? "YES" : "NO");
    Serial.printf("Moving: %s\n",
                  stepper.isMoving() ? "YES" : "NO");
    Serial.printf("Current position: %ld\n",
                  stepper.getCurrentPosition());
    Serial.printf("Target position: %ld\n",
                  stepper.getTargetPosition());
    Serial.printf("Distance to go: %ld\n",
                  stepper.getDistanceToGo());

    Serial.println("\n[COOLING STACK]");
    cooling.printStatus(Serial);

    Serial.println("\n[LOAD CELLS]");
    loadcells.printStatus(Serial);

    Serial.println("\n[RFID]");
    rfid.printStatus(Serial);

    Serial.println("\n[SERVO]");
    servo.printStatus(Serial);

    Serial.println("\n[TEMPERATURE]");
    temperature.printStatus(Serial);

    Serial.println("\n[LIMIT SWITCHES]");
    switches.printStatus(Serial);

    Serial.println("\n===================================");
}

// ============================================================
// STEPPER COMMANDS
// ============================================================

void processStepperCommand(const String &command)
{
    String action = getToken(command, 1);
    String argument = getToken(command, 2);

    if (action == "wake")
    {
        stepper.wake();
        Serial.println("[STEPPER] Wake command sent.");
    }

    else if (action == "sleep")
    {
        stepper.sleep();
        Serial.println("[STEPPER] Sleep command sent.");
    }

    else if (action == "move")
    {
        if (argument.length() == 0)
        {
            Serial.println("Usage: step move <steps>");
            return;
        }

        long steps = argument.toInt();
        stepper.moveRelative(steps);

        Serial.printf(
            "[STEPPER] Moving relative %ld steps.\n",
            steps);
    }

    else if (action == "absolute")
    {
        if (argument.length() == 0)
        {
            Serial.println("Usage: step absolute <position>");
            return;
        }

        long position = argument.toInt();
        stepper.moveAbsolute(position);

        Serial.printf(
            "[STEPPER] Moving to position %ld.\n",
            position);
    }

    else if (action == "speed")
    {
        float speed = argument.toFloat();
        stepper.setMaxSpeed(speed);

        Serial.printf(
            "[STEPPER] Maximum speed: %.2f steps/s\n",
            speed);
    }

    else if (action == "accel")
    {
        float acceleration = argument.toFloat();
        stepper.setAcceleration(acceleration);

        Serial.printf(
            "[STEPPER] Acceleration: %.2f steps/s^2\n",
            acceleration);
    }

    else if (action == "stop")
    {
        stepper.stop();
        Serial.println("[STEPPER] Stop command sent.");
    }

    else
    {
        Serial.println("Unknown stepper command.");
        Serial.println(
            "Commands: wake, sleep, move, absolute, speed, accel, stop");
    }
}

// ============================================================
// COOLING COMMANDS
// ============================================================

void processCoolingCommand(const String &command)
{
    String action = getToken(command, 1);
    String argument = getToken(command, 2);

    if (action == "peltier")
    {
        float duty = argument.toFloat();
        cooling.setPeltier(duty);

        Serial.printf("[COOLING] Peltier: %.1f%%\n", duty);
    }

    else if (action == "fan")
    {
        float duty = argument.toFloat();
        cooling.setFan(duty);

        Serial.printf("[COOLING] Fan: %.1f%%\n", duty);
    }

    else if (action == "pump")
    {
        float duty = argument.toFloat();
        cooling.setPump(duty);

        Serial.printf("[COOLING] Pump: %.1f%%\n", duty);
    }

    else if (action == "off")
    {
        cooling.allOff();
        Serial.println("[COOLING] All outputs disabled.");
    }

    else
    {
        Serial.println("Unknown cooling command.");
        Serial.println(
            "Commands: peltier <0-100>, fan <0-100>, pump <0-100>, off");
    }
}

// ============================================================
// LOAD CELL COMMANDS
// ============================================================

void processLoadCommand(const String &command)
{
    String action = getToken(command, 1);
    String argument = getToken(command, 2);
    String thirdArgument = getToken(command, 3);

    if (action == "read")
    {
        uint8_t samples = 1;

        if (argument.length() > 0)
        {
            int requestedSamples = argument.toInt();

            if (requestedSamples > 0 && requestedSamples <= 100)
            {
                samples = (uint8_t)requestedSamples;
            }
        }

        Serial.printf(
            "[LOAD] Acquiring %u sample(s)...\n",
            samples);

        loadcells.acquire(samples);

        printLoadCellReading(LoadCellID::HX1, "HX1");
        printLoadCellReading(LoadCellID::HX2, "HX2");
    }

    else if (action == "continuous")
    {
        if (argument == "on")
        {
            loadcells.setContinuous(true);
            Serial.println("[LOAD] Continuous acquisition enabled.");
        }

        else if (argument == "off")
        {
            loadcells.setContinuous(false);
            Serial.println("[LOAD] Continuous acquisition disabled.");
        }

        else
        {
            Serial.println("Usage: load continuous on|off");
        }
    }

    else if (action == "enable" || action == "disable")
    {
        int id = argument.toInt();

        if (!isValidLoadCell(id))
        {
            Serial.println("Usage: load enable|disable <1|2>");
            return;
        }

        LoadCellID loadID = getLoadCellID(id);

        if (action == "enable")
        {
            loadcells.enable(loadID);
            Serial.printf("[LOAD] HX%d enabled.\n", id);
        }
        else
        {
            loadcells.disable(loadID);
            Serial.printf("[LOAD] HX%d disabled.\n", id);
        }
    }

    else if (action == "tare")
    {
        int id = argument.toInt();

        if (id == 0)
        {
            loadcells.tare(LoadCellID::HX1);
            loadcells.tare(LoadCellID::HX2);

            Serial.println("[LOAD] Both load cells tared.");
        }

        else if (isValidLoadCell(id))
        {
            loadcells.tare(getLoadCellID(id));
            Serial.printf("[LOAD] HX%d tared.\n", id);
        }

        else
        {
            Serial.println("Usage: load tare <0|1|2>");
        }
    }

    else if (action == "calibrate")
    {
        int id = argument.toInt();
        float knownWeight = thirdArgument.toFloat();

        if (!isValidLoadCell(id) ||
            thirdArgument.length() == 0 ||
            knownWeight <= 0.0f)
        {
            Serial.println(
                "Usage: load calibrate <1|2> <known_weight>");
            return;
        }

        loadcells.calibrate(getLoadCellID(id), knownWeight);

        Serial.printf(
            "[LOAD] Calibration requested for HX%d using %.3f.\n",
            id,
            knownWeight);
    }

    else if (action == "scale")
    {
        int id = argument.toInt();
        float scale = thirdArgument.toFloat();

        if (!isValidLoadCell(id) ||
            thirdArgument.length() == 0 ||
            scale == 0.0f)
        {
            Serial.println("Usage: load scale <1|2> <factor>");
            return;
        }

        loadcells.setScale(getLoadCellID(id), scale);

        Serial.printf(
            "[LOAD] HX%d scale set to %.5f.\n",
            id,
            scale);
    }

    else if (action == "status")
    {
        loadcells.printStatus(Serial);
    }

    else
    {
        Serial.println("Load cell commands:");
        Serial.println("  load read [samples]");
        Serial.println("  load continuous on|off");
        Serial.println("  load enable <1|2>");
        Serial.println("  load disable <1|2>");
        Serial.println("  load tare <0|1|2>");
        Serial.println("  load calibrate <1|2> <weight>");
        Serial.println("  load scale <1|2> <factor>");
        Serial.println("  load status");
    }
}

// ============================================================
// RFID COMMANDS
// ============================================================

void processRFIDCommand(const String &command)
{
    String action = getToken(command, 1);
    String argument = getToken(command, 2);

    if (action == "scan")
    {
        if (argument == "on")
        {
            rfid.setScanning(true);
            Serial.println("[RFID] Scanning enabled.");
        }

        else if (argument == "off")
        {
            rfid.setScanning(false);
            Serial.println("[RFID] Scanning disabled.");
        }

        else
        {
            Serial.println("Usage: rfid scan on|off");
        }
    }

    else if (action == "status")
    {
        rfid.printStatus(Serial);
    }

    else
    {
        Serial.println("RFID commands:");
        Serial.println("  rfid scan on|off");
        Serial.println("  rfid status");
    }
}

// ============================================================
// SERVO COMMANDS
// ============================================================

void processServoCommand(const String &command)
{
    String action = getToken(command, 1);
    String speedArgument = getToken(command, 2);
    String durationArgument = getToken(command, 3);

    if (action == "continuous")
    {
        if (speedArgument.length() == 0)
        {
            Serial.println("Usage: servo continuous <speed>");
            return;
        }

        float speed = speedArgument.toFloat();

        servo.setSpeed(speed);

        Serial.printf(
            "[SERVO] Continuous rotation set to %.1f%%.\n",
            servo.getSpeed());
    }

    else if (action == "timed")
    {
        if (speedArgument.length() == 0 ||
            durationArgument.length() == 0)
        {
            Serial.println("Usage: servo timed <speed> <duration_ms>");
            return;
        }

        float speed = speedArgument.toFloat();
        uint32_t duration = (uint32_t)durationArgument.toInt();

        if (duration == 0)
        {
            Serial.println("[SERVO] Duration must be greater than zero.");
            return;
        }

        servo.rotateFor(speed, duration);
    }

    else if (action == "stop")
    {
        servo.stop();
        Serial.println("[SERVO] Stopped.");
    }

    else if (action == "status")
    {
        servo.printStatus(Serial);
    }

    else
    {
        Serial.println("Servo commands:");
        Serial.println("  servo continuous <speed>");
        Serial.println("  servo timed <speed> <duration_ms>");
        Serial.println("  servo stop");
        Serial.println("  servo status");
    }
}

// ============================================================
// TEMPERATURE COMMANDS
// ============================================================

void processTemperatureCommand(const String &command)
{
    String action = getToken(command, 1);
    String argument = getToken(command, 2);
    String thirdArgument = getToken(command, 3);
    String fourthArgument = getToken(command, 4);

    if (action == "read")
    {
        uint8_t samples = 1;

        if (argument == "all")
        {
            if (thirdArgument.length() > 0)
            {
                int requestedSamples = thirdArgument.toInt();

                if (requestedSamples > 0 && requestedSamples <= 64)
                {
                    samples = (uint8_t)requestedSamples;
                }
            }

            Serial.printf(
                "[TEMP] Acquiring all clusters using %u samples...\n",
                samples);

            temperature.sampleAll(samples);

            printTemperatureReading(TEMP1, "CLUSTER 1");
            printTemperatureReading(TEMP2, "CLUSTER 2");
            printTemperatureReading(TEMP3, "CLUSTER 3");

            return;
        }

        int id = argument.toInt();

        if (!isValidTemperatureCluster(id))
        {
            Serial.println("Usage: temp read <1|2|3|all> [samples]");
            return;
        }

        if (thirdArgument.length() > 0)
        {
            int requestedSamples = thirdArgument.toInt();

            if (requestedSamples > 0 && requestedSamples <= 64)
            {
                samples = (uint8_t)requestedSamples;
            }
        }

        TemperatureCluster cluster = getTemperatureCluster(id);

        Serial.printf(
            "[TEMP] Acquiring cluster %d using %u samples...\n",
            id,
            samples);

        temperature.sample(cluster, samples);

        if (id == 1)
        {
            printTemperatureReading(TEMP1, "CLUSTER 1");
        }
        else if (id == 2)
        {
            printTemperatureReading(TEMP2, "CLUSTER 2");
        }
        else
        {
            printTemperatureReading(TEMP3, "CLUSTER 3");
        }
    }

    else if (action == "continuous")
    {
        bool enabled;

        if (thirdArgument == "on")
        {
            enabled = true;
        }

        else if (thirdArgument == "off")
        {
            enabled = false;
        }

        else
        {
            Serial.println(
                "Usage: temp continuous <1|2|3|all> on|off");
            return;
        }

        if (argument == "all")
        {
            temperature.setContinuous(TEMP1, enabled);
            temperature.setContinuous(TEMP2, enabled);
            temperature.setContinuous(TEMP3, enabled);

            Serial.printf(
                "[TEMP] Continuous sampling for all clusters %s.\n",
                enabled ? "ENABLED" : "DISABLED");

            return;
        }

        int id = argument.toInt();

        if (!isValidTemperatureCluster(id))
        {
            Serial.println(
                "Usage: temp continuous <1|2|3|all> on|off");
            return;
        }

        temperature.setContinuous(
            getTemperatureCluster(id),
            enabled);
    }

    else if (action == "calibrate")
    {
        int id = argument.toInt();
        float knownTemperature = thirdArgument.toFloat();

        if (!isValidTemperatureCluster(id) ||
            thirdArgument.length() == 0)
        {
            Serial.println(
                "Usage: temp calibrate <1|2|3> <known_temp_C>");
            return;
        }

        if (!temperature.calibrate(
                getTemperatureCluster(id),
                knownTemperature,
                8))
        {
            Serial.println("[TEMP] Calibration failed.");
        }
    }

    else if (action == "adjust")
    {
        int id = argument.toInt();
        float offset = thirdArgument.toFloat();
        float gain = fourthArgument.toFloat();

        if (!isValidTemperatureCluster(id) ||
            thirdArgument.length() == 0 ||
            fourthArgument.length() == 0 ||
            gain <= 0.0f)
        {
            Serial.println(
                "Usage: temp adjust <1|2|3> <offset_C> <gain>");
            return;
        }

        temperature.setCalibration(
            getTemperatureCluster(id),
            offset,
            gain);
    }

    else if (action == "status")
    {
        temperature.printStatus(Serial);
    }

    else
    {
        Serial.println("Temperature commands:");
        Serial.println("  temp read <1|2|3|all> [samples]");
        Serial.println("  temp continuous <1|2|3|all> on|off");
        Serial.println("  temp calibrate <1|2|3> <known_temp_C>");
        Serial.println("  temp adjust <1|2|3> <offset_C> <gain>");
        Serial.println("  temp status");
    }
}

// ============================================================
// LIMIT SWITCH COMMANDS
// ============================================================

void processSwitchCommand(const String &command)
{
    String action = getToken(command, 1);

    if (action == "status" || action == "read")
    {
        switches.printStatus(Serial);
    }

    else
    {
        Serial.println("Limit switch commands:");
        Serial.println("  switch status");
        Serial.println("  switch read");
        Serial.println(
            "Switch transitions are broadcast automatically.");
    }
}

// ============================================================
// HELP
// ============================================================

void printHelp()
{
    Serial.println("\n========== AVAILABLE COMMANDS ==========");

    Serial.println("\nSYSTEM");
    Serial.println("  status");
    Serial.println("  off");
    Serial.println("  help");

    Serial.println("\nSTEPPER");
    Serial.println("  step wake");
    Serial.println("  step sleep");
    Serial.println("  step move <steps>");
    Serial.println("  step absolute <position>");
    Serial.println("  step speed <steps_per_second>");
    Serial.println("  step accel <steps_per_second_squared>");
    Serial.println("  step stop");

    Serial.println("\nCOOLING");
    Serial.println("  cool peltier <0-100>");
    Serial.println("  cool fan <0-100>");
    Serial.println("  cool pump <0-100>");
    Serial.println("  cool off");

    Serial.println("\nLOAD CELLS");
    Serial.println("  load read [samples]");
    Serial.println("  load continuous on|off");
    Serial.println("  load enable <1|2>");
    Serial.println("  load disable <1|2>");
    Serial.println("  load tare <0|1|2>");
    Serial.println("  load calibrate <1|2> <weight>");
    Serial.println("  load scale <1|2> <factor>");
    Serial.println("  load status");

    Serial.println("\nRFID");
    Serial.println("  rfid scan on|off");
    Serial.println("  rfid status");

    Serial.println("\nSERVO");
    Serial.println("  servo continuous <speed>");
    Serial.println("  servo timed <speed> <duration_ms>");
    Serial.println("  servo stop");
    Serial.println("  servo status");

    Serial.println("\nTEMPERATURE");
    Serial.println("  temp read <1|2|3|all> [samples]");
    Serial.println("  temp continuous <1|2|3|all> on|off");
    Serial.println("  temp calibrate <1|2|3> <known_temp_C>");
    Serial.println("  temp adjust <1|2|3> <offset_C> <gain>");
    Serial.println("  temp status");

    Serial.println("\nLIMIT SWITCHES");
    Serial.println("  switch status");
    Serial.println("  switch read");
    Serial.println("  Switch transitions are broadcast automatically.");

    Serial.println("\n========================================");
}

// ============================================================
// COMMAND DISPATCHER
// ============================================================

void processCommand(String command)
{
    command.trim();

    if (command.length() == 0)
    {
        return;
    }

    Serial.printf("\n> %s\n", command.c_str());

    if (command == "status")
    {
        systemStatus();
    }

    else if (command == "off")
    {
        stepper.sleep();
        cooling.allOff();
        loadcells.setContinuous(false);
        rfid.setScanning(false);
        servo.stop();

        temperature.setContinuous(TEMP1, false);
        temperature.setContinuous(TEMP2, false);
        temperature.setContinuous(TEMP3, false);

        // Limit switch monitoring remains active.
        Serial.println(
            "[SYSTEM] Outputs disabled. Limit switch monitoring remains active.");
    }

    else if (command == "help")
    {
        printHelp();
    }

    else if (command.startsWith("step "))
    {
        processStepperCommand(command);
    }

    else if (command.startsWith("cool "))
    {
        processCoolingCommand(command);
    }

    else if (command.startsWith("load "))
    {
        processLoadCommand(command);
    }

    else if (command.startsWith("rfid "))
    {
        processRFIDCommand(command);
    }

    else if (command.startsWith("servo "))
    {
        processServoCommand(command);
    }

    else if (command.startsWith("temp "))
    {
        processTemperatureCommand(command);
    }

    else if (command.startsWith("switch "))
    {
        processSwitchCommand(command);
    }

    else
    {
        Serial.println("[SYSTEM] Unknown command. Type 'help'.");
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
    Serial.println("========================================");
    Serial.println("       ESP32-S3 MODULAR CONTROLLER");
    Serial.println("========================================");

    Serial.println("\n[INIT] Initializing stepper...");
    stepper.begin();

    Serial.println("\n[INIT] Initializing cooling stack...");
    cooling.begin();

    Serial.println("\n[INIT] Initializing load cells...");
    loadcells.begin();

    Serial.println("\n[INIT] Initializing RFID...");
    if (!rfid.begin())
    {
        Serial.println("[INIT] RFID initialization failed.");
    }

    Serial.println("\n[INIT] Initializing servo...");
    if (!servo.begin())
    {
        Serial.println("[INIT] Servo initialization failed.");
    }

    Serial.println("\n[INIT] Initializing temperature module...");
    if (!temperature.begin())
    {
        Serial.println("[INIT] Temperature module initialization failed.");
    }

    Serial.println("\n[INIT] Initializing limit switches...");

    switches.setCallback(onSwitchChanged);

    if (!switches.begin())
    {
        Serial.println("[INIT] Switch module initialization failed.");
    }

    Serial.println("\n[INIT] Initialization sequence completed.");

    printHelp();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Update all non-blocking modules.
    stepper.run();
    loadcells.update();
    rfid.update();
    servo.update();
    temperature.update();
    switches.update();

    // Non-blocking serial command reception.
    while (Serial.available() > 0)
    {
        char incoming = (char)Serial.read();

        if (incoming == '\n' || incoming == '\r')
        {
            if (serialBuffer.length() > 0)
            {
                processCommand(serialBuffer);
                serialBuffer = "";
            }
        }

        else if (serialBuffer.length() < SERIAL_BUFFER_LIMIT)
        {
            serialBuffer += incoming;
        }

        else
        {
            serialBuffer = "";
            Serial.println(
                "[SERIAL] Command too long. Buffer cleared.");
        }
    }
}