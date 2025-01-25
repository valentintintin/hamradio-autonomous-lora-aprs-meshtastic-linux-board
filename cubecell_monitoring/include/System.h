#ifndef CUBECELL_MONITORING_SYSTEM_H
#define CUBECELL_MONITORING_SYSTEM_H

#include <cstdint>
#include <CubeCell_NeoPixel.h>
#include <DS3231.h>

#include "Communication.h"
#include "Timer.h"
#include "Config.h"
#include "Command.h"
#include "MpptMonitor.h"
#include "WeatherSensors.h"
#include "Gpio.h"

class System {
public:
    explicit System(CubeCell_NeoPixel *pixels, TimerEvent_t *wakeUpEvent);

    bool begin(RadioEvents_t *radioEvents);
    void update();
    bool isBoxOpened() const;
    void setFunctionAllowed(byte function, bool allowed, bool save = true);
    void sleep(uint64_t time);
    void resetTimerJson();

    inline bool isFunctionAllowed(byte function) const {
        return functionsAllowed[function];
    }

    void turnOnRGB(uint32_t color);
    void turnOffRGB();
    void serialError(const char* content, bool addError = true);
    void addError();
    void removeOneError();

    static void delayWdt(uint32_t milliseconds);
    static void setTimeFromRTcToInternalRtc(uint64_t epoch);
    static DateTime nowToString(char *result);

    Communication *communication{};
    Command command;
    MpptMonitor mpptMonitor;
    WeatherSensors weatherSensors;
    Gpio gpio;
    DS3231 RTC;

    bool forceSendPosition = false;
    bool forceSendTelemetry = false;
    uint8_t nbError = 0;
private:
    uint32_t ledColor = 0;

    bool functionsAllowed[8] = {
            true, // EEPROM_ADDRESS_MPPT_WATCHDOG
            true, // EEPROM_ADDRESS_APRS_DIGIPEATER
            true, // EEPROM_ADDRESS_APRS_TELEMETRY
            true, // EEPROM_ADDRESS_APRS_POSITION
            false, // EEPROM_ADDRESS_SLEEP
            false, // EEPROM_ADDRESS_RESET_ON_ERROR
            false, // EEPROM_ADDRESS_WATCHDOG
            true // EEPROM_ADDRESS_WATCHDOG_LORA_TX
    };

    Timer timerStatus = Timer(INTERVAL_STATUS_APRS, false);
    Timer timerPosition = Timer(INTERVAL_POSITION_AND_WEATHER_APRS, true);
    Timer timerTelemetry = Timer(INTERVAL_TELEMETRY_APRS, true);
    Timer timerState = Timer(INTERVAL_STATE, true);
    Timer timerBoxOpened = Timer(INTERVAL_ALARM_BOX_OPENED_APRS, true);
    Timer timerBlinker = Timer(INTERVAL_BLINKER, true);

    CubeCell_NeoPixel *pixels;
    TimerEvent_t *wakeUpEvent;

    void printJsonSystem(const char *state);
};

#endif //CUBECELL_MONITORING_SYSTEM_H
