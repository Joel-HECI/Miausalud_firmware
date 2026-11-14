
#ifndef SCHEDULE_MODULE_H
#define SCHEDULE_MODULE_H

#include <Arduino.h>
#include <esp_sleep.h>

// ============================================================
// CONFIGURATION
// ============================================================

namespace ScheduleConfig
{

    // Reserved for the future RFID interrupt.
    static constexpr gpio_num_t RFID_WAKE_PIN =
        GPIO_NUM_1;

    // MFRC522 IRQ is normally active LOW.
    static constexpr int RFID_WAKE_LEVEL = LOW;

    static constexpr uint32_t MIN_TIMER_SECONDS = 1;

    static constexpr uint32_t MAX_TIMER_SECONDS =
        7UL * 24UL * 60UL * 60UL;

    // Maximum awake delay before automatically sleeping.
    static constexpr uint32_t MAX_SLEEP_DELAY_SECONDS =
        24UL * 60UL * 60UL;
}

// ============================================================
// WAKE REASONS
// ============================================================

enum ScheduleWakeReason : uint8_t
{
    SCHEDULE_WAKE_COLD_BOOT = 0,
    SCHEDULE_WAKE_TIMER,
    SCHEDULE_WAKE_RFID,
    SCHEDULE_WAKE_OTHER
};

// ============================================================
// CALLBACK TYPES
// ============================================================

// Called once after modules have been initialized.
using ScheduleWakeCallback =
    void (*)(ScheduleWakeReason reason);

// Called immediately before deep sleep.
using ScheduleSleepCallback =
    void (*)();

// ============================================================
// MODULE
// ============================================================

class ScheduleModule
{

public:
    ScheduleModule();

    // Initialize and identify the reset/wakeup reason.
    void begin();

    // Check delayed sleep requests.
    void update();

    // --------------------------------------------------------
    // TIMER SCHEDULE
    // --------------------------------------------------------

    // Wake once after the given sleep interval.
    bool scheduleOnce(uint32_t seconds);

    // Wake repeatedly, rearming on each sleep.
    bool scheduleEvery(uint32_t seconds);

    // Disable timer-based wakeup.
    void cancelTimer();

    bool isTimerEnabled();
    bool isRecurring();
    uint32_t getTimerSeconds();

    // --------------------------------------------------------
    // RFID WAKEUP
    // --------------------------------------------------------

    void setRFIDWakeup(bool enabled);
    bool isRFIDWakeupEnabled();

    // --------------------------------------------------------
    // SLEEP CONTROL
    // --------------------------------------------------------

    // Enter deep sleep immediately.
    // Returns false if configuration is invalid.
    bool sleepNow();

    // Schedule entry into deep sleep while awake.
    bool sleepAfter(uint32_t seconds);

    // Cancel a pending delayed sleep.
    void cancelPendingSleep();

    bool isSleepPending();
    uint32_t getSleepCountdown();

    // --------------------------------------------------------
    // WAKE EVENT HANDLING
    // --------------------------------------------------------

    ScheduleWakeReason getWakeReason();

    // True for any supported deep-sleep wakeup.
    bool wasDeepSleepWake();

    // Register future task dispatcher.
    void setWakeCallback(ScheduleWakeCallback callback);

    // Register pre-sleep shutdown handler.
    void setSleepCallback(ScheduleSleepCallback callback);

    // Deliver the wake event once after setup.
    void dispatchWakeEvent();

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    void printStatus(Stream &output);

    static const char *reasonToString(
        ScheduleWakeReason reason);

private:
    ScheduleWakeReason wakeReason;

    ScheduleWakeCallback wakeCallback;
    ScheduleSleepCallback sleepCallback;

    bool eventDispatched;

    bool pendingSleep;
    uint32_t sleepStartMs;
    uint32_t sleepDelayMs;

    bool configureWakeSources();
};

#endif
