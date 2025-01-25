#ifndef RP2040_LORA_APRS_SYSTEM_H
#define RP2040_LORA_APRS_SYSTEM_H

#include <cstdint>
#include <ThreadController.h>
#include <DS3231.h>
#include <JsonWriter.h>

#include "Communication.h"
#include "Timer.h"
#include "config.h"
#include "Command.h"
#include "GpioPin.h"
#include "Threads/EnergyThread.h"
#include "Threads/WeatherThread.h"
#include "Threads/Watchdog/WatchdogSlaveMpptChgThread.h"
#include "Threads/Watchdog/WatchdogSlaveLoraTxThread.h"
#include "Threads/Watchdog/WatchdogMasterPinThread.h"
#include "Threads/Send/SendPositionThread.h"
#include "Threads/Send/SendStatusThread.h"
#include "Threads/Send/SendTelemetriesThread.h"

#include "Settings.h"
#include "Threads/LdrBoxOpenedThread.h"
#include "Threads/Send/MeshtasticSendAprsThread.h"
#include "Threads/Send/LinuxSendAprsThread.h"

class System {
public:
    explicit System();

    bool begin();
    void loop();

    void setSlowClock();
    void setTimeToInternalRtc(uint32_t unixtime);
    bool resetSettings();
    bool saveSettings();
    void addAprsFrameReceivedToHistory(const AprsPacketLite *packet, float snr, float rssi);
    void planReboot();
    void planDfu();
    void printSettings();
    void printJson(bool onUsb);
    void sendToSerial(const uint8_t* data, size_t size);

    GpioPin* getGpio(uint8_t pin);
    DateTime getDateTime() const;

    inline bool hasError() const {
        return communication.hasError() || weatherThread->hasError() || energyThread->hasError();
    }

    inline bool isInDebugMode() const {
        return !isSlowClock;
    }

    Settings settings{};
    SettingsAprsCallsignHeard *lastAprsHeard = nullptr;

    LdrBoxOpenedThread *ldrBoxOpenedThread{};
    EnergyThread *energyThread{};
    WeatherThread *weatherThread{};

    WatchdogSlaveLoraTxThread *watchdogSlaveLoraTxThread{};
    SendPositionThread *sendPositionThread{};
    SendStatusThread *sendStatusThread{};
    SendTelemetriesThread *sendTelemetriesThread{};
    MeshtasticSendAprsThread *sendMeshtasticAprsThread{};
    LinuxSendAprsThread *sendLinuxAprsThread{};

    Communication communication;
    Command command;
    GpioPin gpioLed = GpioPin(LED_BUILTIN, OUTPUT_2MA);
    GpioPin *gpiosPin[MAX_GPIO_USED]{};

    mpptChg mpptChgCharger;

    WatchdogSlaveMpptChgThread *watchdogSlaveMpptChgThread{};

    WatchdogMasterPinThread *watchdogMeshtastic{};
    WatchdogMasterPinThread *watchdogLinux{};

    DS3231 rtc;
private:
    bool isSlowClock = false;
    ThreadController threadController;
    Timer timerDfu = Timer(TIME_BEFORE_REBOOT);
    Timer timerReboot = Timer(TIME_BEFORE_REBOOT);
    Timer timerPrintJson = Timer(INTERVAL_PRINT_JSON_USB, true);
    JsonWriter serialJsonWriter = JsonWriter(&Serial);
    JsonWriter serialLinuxJsonWriter = JsonWriter(&Serial1);
    JsonWriter serialLowPowerJsonWriter = JsonWriter(&Serial2);

    bool loadSettings();
    void setDefaultSettings();
};

#endif //RP2040_LORA_APRS_SYSTEM_H
