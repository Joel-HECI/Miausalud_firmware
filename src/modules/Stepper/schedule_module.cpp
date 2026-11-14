
#include "modules/schedule_module.h"

#include <esp_attr.h>
#include <esp_err.h>
#include <driver/rtc_io.h>

// ============================================================
// RTC PERSISTENT STORAGE
// ============================================================

// Retained across deep sleep.
// Not guaranteed across power loss or a complete RTC reset.

namespace
{

    static constexpr uint32_t SCHEDULE_MAGIC =
        0x53434831;

    struct PersistentSchedule
    {

        uint32_t magic;

        uint32_t timerSeconds;

        uint32_t bootCount;

        uint32_t timerWakeCount;
        uint32_t rfidWakeCount;

        uint8_t timerEnabled;
        uint8_t recurring;
        uint8_t rfidEnabled;
        uint8_t reserved;
    };

    RTC_DATA_ATTR PersistentSchedule rtcSchedule;

    void initializePersistentSchedule()
    {
        if (rtcSchedule.magic == SCHEDULE_MAGIC)
        {
            return;
        }

        rtcSchedule.magic = SCHEDULE_MAGIC;

        rtcSchedule.timerSeconds = 0;

        rtcSchedule.bootCount = 0;
        rtcSchedule.timerWakeCount = 0;
        rtcSchedule.rfidWakeCount = 0;

        rtcSchedule.timerEnabled = 0;
        rtcSchedule.recurring = 0;

        // Disabled until the RFID IRQ is configured.
        rtcSchedule.rfidEnabled = 0;

        rtcSchedule.reserved = 0;
    }
}

// ============================================================
// CONSTRUCTOR
// ============================================================

ScheduleModule::ScheduleModule()
    : wakeReason(SCHEDULE_WAKE_COLD_BOOT),
      wakeCallback(nullptr),
      sleepCallback(nullptr),
      eventDispatched(false),
      pendingSleep(false),
      sleepStartMs(0),
      sleepDelayMs(0)
{
}

// ============================================================
// INITIALIZATION
// ============================================================

void ScheduleModule::begin()
{
    initializePersistentSchedule();

    rtcSchedule.bootCount++;

    esp_sleep_wakeup_cause_t cause =
        esp_sleep_get_wakeup_cause();

    switch (cause)
    {

    case ESP_SLEEP_WAKEUP_TIMER:

        wakeReason = SCHEDULE_WAKE_TIMER;

        rtcSchedule.timerWakeCount++;

        // A one-shot timer is consumed only when
        // the RTC timer actually wakes the chip.
        if (
            rtcSchedule.timerEnabled &&
            !rtcSchedule.recurring)
        {
            rtcSchedule.timerEnabled = 0;
        }

        break;

    case ESP_SLEEP_WAKEUP_EXT0:

        wakeReason = SCHEDULE_WAKE_RFID;

        rtcSchedule.rfidWakeCount++;

        break;

    case ESP_SLEEP_WAKEUP_UNDEFINED:

        wakeReason = SCHEDULE_WAKE_COLD_BOOT;
        break;

    default:

        wakeReason = SCHEDULE_WAKE_OTHER;
        break;
    }

    // EXT0 wakeup leaves GPIO1 configured as RTC IO.
    // Restore ordinary GPIO operation.
    rtc_gpio_deinit(ScheduleConfig::RFID_WAKE_PIN);

    pinMode(
        static_cast<uint8_t>(
            ScheduleConfig::RFID_WAKE_PIN),
        INPUT_PULLUP);

    eventDispatched = false;

    pendingSleep = false;
    sleepStartMs = 0;
    sleepDelayMs = 0;
}

// ============================================================
// ONE-SHOT TIMER
// ============================================================

bool ScheduleModule::scheduleOnce(uint32_t seconds)
{
    if (
        seconds < ScheduleConfig::MIN_TIMER_SECONDS ||
        seconds > ScheduleConfig::MAX_TIMER_SECONDS)
    {
        return false;
    }

    rtcSchedule.timerSeconds = seconds;

    rtcSchedule.timerEnabled = 1;
    rtcSchedule.recurring = 0;

    return true;
}

// ============================================================
// RECURRING TIMER
// ============================================================

bool ScheduleModule::scheduleEvery(uint32_t seconds)
{
    if (
        seconds < ScheduleConfig::MIN_TIMER_SECONDS ||
        seconds > ScheduleConfig::MAX_TIMER_SECONDS)
    {
        return false;
    }

    rtcSchedule.timerSeconds = seconds;

    rtcSchedule.timerEnabled = 1;
    rtcSchedule.recurring = 1;

    return true;
}

// ============================================================
// CANCEL TIMER
// ============================================================

void ScheduleModule::cancelTimer()
{
    rtcSchedule.timerEnabled = 0;
    rtcSchedule.recurring = 0;
    rtcSchedule.timerSeconds = 0;
}

// ============================================================
// TIMER STATUS
// ============================================================

bool ScheduleModule::isTimerEnabled()
{
    return rtcSchedule.timerEnabled != 0;
}

bool ScheduleModule::isRecurring()
{
    return rtcSchedule.recurring != 0;
}

uint32_t ScheduleModule::getTimerSeconds()
{
    return rtcSchedule.timerSeconds;
}

// ============================================================
// RFID WAKEUP CONFIGURATION
// ============================================================

void ScheduleModule::setRFIDWakeup(bool enabled)
{
    rtcSchedule.rfidEnabled = enabled ? 1 : 0;
}

bool ScheduleModule::isRFIDWakeupEnabled()
{
    return rtcSchedule.rfidEnabled != 0;
}

// ============================================================
// CONFIGURE WAKE SOURCES
// ============================================================

bool ScheduleModule::configureWakeSources()
{
    // Remove any previously configured wake sources.
    esp_err_t result =
        esp_sleep_disable_wakeup_source(
            ESP_SLEEP_WAKEUP_ALL);

    if (result != ESP_OK)
    {
        Serial.printf(
            "[SCHEDULE] Failed to clear wake sources: %s\n",
            esp_err_to_name(result));

        return false;
    }

    bool hasWakeSource = false;

    // --------------------------------------------------------
    // RTC TIMER
    // --------------------------------------------------------

    if (rtcSchedule.timerEnabled)
    {

        if (
            rtcSchedule.timerSeconds <
                ScheduleConfig::MIN_TIMER_SECONDS ||
            rtcSchedule.timerSeconds >
                ScheduleConfig::MAX_TIMER_SECONDS)
        {
            return false;
        }

        uint64_t durationUs =
            static_cast<uint64_t>(
                rtcSchedule.timerSeconds) *
            1000000ULL;

        result = esp_sleep_enable_timer_wakeup(
            durationUs);

        if (result != ESP_OK)
        {
            Serial.printf(
                "[SCHEDULE] Timer configuration error: %s\n",
                esp_err_to_name(result));

            return false;
        }

        hasWakeSource = true;
    }

    // --------------------------------------------------------
    // RFID GPIO WAKEUP
    // --------------------------------------------------------

    if (rtcSchedule.rfidEnabled)
    {

        // EXT0 is level-sensitive, not edge-triggered.
        // The IRQ must be inactive before going to sleep.
        if (
            digitalRead(
                static_cast<uint8_t>(
                    ScheduleConfig::RFID_WAKE_PIN)) == ScheduleConfig::RFID_WAKE_LEVEL)
        {
            Serial.println(
                "[SCHEDULE] GPIO1 is already active.");

            Serial.println(
                "[SCHEDULE] Sleep rejected to avoid immediate wakeup.");

            return false;
        }

        result = esp_sleep_enable_ext0_wakeup(
            ScheduleConfig::RFID_WAKE_PIN,
            ScheduleConfig::RFID_WAKE_LEVEL);

        if (result != ESP_OK)
        {
            Serial.printf(
                "[SCHEDULE] RFID wake configuration error: %s\n",
                esp_err_to_name(result));

            return false;
        }

        // Configure RTC GPIO pull-up for active-LOW IRQ.
        // An external pull-up is still recommended.
        result = rtc_gpio_init(
            ScheduleConfig::RFID_WAKE_PIN);

        if (result != ESP_OK)
        {
            return false;
        }

        result = rtc_gpio_set_direction(
            ScheduleConfig::RFID_WAKE_PIN,
            RTC_GPIO_MODE_INPUT_ONLY);

        if (result != ESP_OK)
        {
            return false;
        }

        result = rtc_gpio_pullup_en(
            ScheduleConfig::RFID_WAKE_PIN);

        if (result != ESP_OK)
        {
            return false;
        }

        result = rtc_gpio_pulldown_dis(
            ScheduleConfig::RFID_WAKE_PIN);

        if (result != ESP_OK)
        {
            return false;
        }

        hasWakeSource = true;
    }

    if (!hasWakeSource)
    {
        Serial.println(
            "[SCHEDULE] No wake source configured.");

        return false;
    }

    return true;
}

// ============================================================
// ENTER DEEP SLEEP
// ============================================================

bool ScheduleModule::sleepNow()
{
    pendingSleep = false;

    if (!configureWakeSources())
    {

        Serial.println(
            "[SCHEDULE] Deep sleep aborted.");

        return false;
    }

    // Allow other modules to shut down their outputs.
    if (sleepCallback != nullptr)
    {
        sleepCallback();
    }

    Serial.println();
    Serial.println("================================");
    Serial.println("       ENTERING DEEP SLEEP      ");
    Serial.println("================================");

    if (rtcSchedule.timerEnabled)
    {

        Serial.printf(
            "[SCHEDULE] RTC wake in %lu seconds\n",
            static_cast<unsigned long>(
                rtcSchedule.timerSeconds));
    }

    if (rtcSchedule.rfidEnabled)
    {

        Serial.println(
            "[SCHEDULE] GPIO1 RFID wake enabled.");
    }

    Serial.flush();

    // Execution normally never returns from this call.
    esp_deep_sleep_start();

    return true;
}

// ============================================================
// DELAYED SLEEP
// ============================================================

bool ScheduleModule::sleepAfter(uint32_t seconds)
{
    if (
        seconds == 0 ||
        seconds >
            ScheduleConfig::MAX_SLEEP_DELAY_SECONDS)
    {
        return false;
    }

    pendingSleep = true;

    sleepStartMs = millis();

    sleepDelayMs = seconds * 1000UL;

    return true;
}

// ============================================================
// CANCEL DELAYED SLEEP
// ============================================================

void ScheduleModule::cancelPendingSleep()
{
    pendingSleep = false;

    sleepStartMs = 0;
    sleepDelayMs = 0;
}

// ============================================================
// DELAYED SLEEP STATUS
// ============================================================

bool ScheduleModule::isSleepPending()
{
    return pendingSleep;
}

uint32_t ScheduleModule::getSleepCountdown()
{
    if (!pendingSleep)
    {
        return 0;
    }

    uint32_t elapsed = millis() - sleepStartMs;

    if (elapsed >= sleepDelayMs)
    {
        return 0;
    }

    uint32_t remainingMs =
        sleepDelayMs - elapsed;

    return (remainingMs + 999UL) / 1000UL;
}

// ============================================================
// MAIN UPDATE
// ============================================================

void ScheduleModule::update()
{
    if (!pendingSleep)
    {
        return;
    }

    uint32_t elapsed = millis() - sleepStartMs;

    if (elapsed >= sleepDelayMs)
    {

        pendingSleep = false;

        sleepNow();
    }
}

// ============================================================
// WAKE REASON
// ============================================================

ScheduleWakeReason ScheduleModule::getWakeReason()
{
    return wakeReason;
}

bool ScheduleModule::wasDeepSleepWake()
{
    return (
        wakeReason == SCHEDULE_WAKE_TIMER ||
        wakeReason == SCHEDULE_WAKE_RFID ||
        wakeReason == SCHEDULE_WAKE_OTHER);
}

// ============================================================
// CALLBACK REGISTRATION
// ============================================================

void ScheduleModule::setWakeCallback(
    ScheduleWakeCallback callback)
{
    wakeCallback = callback;
}

void ScheduleModule::setSleepCallback(
    ScheduleSleepCallback callback)
{
    sleepCallback = callback;
}

// ============================================================
// WAKE EVENT DISPATCH
// ============================================================

void ScheduleModule::dispatchWakeEvent()
{
    if (eventDispatched)
    {
        return;
    }

    eventDispatched = true;

    if (wakeCallback != nullptr)
    {
        wakeCallback(wakeReason);
    }
}

// ============================================================
// WAKE REASON STRING
// ============================================================

const char *ScheduleModule::reasonToString(
    ScheduleWakeReason reason)
{
    switch (reason)
    {

    case SCHEDULE_WAKE_TIMER:
        return "RTC_TIMER";

    case SCHEDULE_WAKE_RFID:
        return "RFID_GPIO1";

    case SCHEDULE_WAKE_OTHER:
        return "OTHER_WAKEUP";

    case SCHEDULE_WAKE_COLD_BOOT:
    default:
        return "COLD_BOOT";
    }
}

// ============================================================
// STATUS
// ============================================================

void ScheduleModule::printStatus(Stream &output)
{
    output.println();
    output.println("========== SCHEDULE ==========");

    output.print("Wake reason: ");
    output.println(reasonToString(wakeReason));

    output.print("Boot count: ");
    output.println(rtcSchedule.bootCount);

    output.print("Timer wake count: ");
    output.println(rtcSchedule.timerWakeCount);

    output.print("RFID wake count: ");
    output.println(rtcSchedule.rfidWakeCount);

    output.print("RTC timer: ");
    output.println(
        rtcSchedule.timerEnabled ? "ENABLED" : "DISABLED");

    if (rtcSchedule.timerEnabled)
    {

        output.print("Timer interval: ");
        output.print(rtcSchedule.timerSeconds);
        output.println(" seconds");

        output.print("Timer mode: ");
        output.println(
            rtcSchedule.recurring ? "RECURRING" : "ONE-SHOT");
    }

    output.print("RFID wake GPIO: ");
    output.println(
        static_cast<int>(
            ScheduleConfig::RFID_WAKE_PIN));

    output.print("RFID wake: ");
    output.println(
        rtcSchedule.rfidEnabled ? "ENABLED" : "DISABLED");

    output.print("Delayed sleep: ");
    output.println(
        pendingSleep ? "PENDING" : "NOT PENDING");

    if (pendingSleep)
    {

        output.print("Sleep countdown: ");
        output.print(getSleepCountdown());
        output.println(" seconds");
    }

    output.println("==============================");
}
