#ifndef CUBECELL_MONITORING_COMMUNICATION_H
#define CUBECELL_MONITORING_COMMUNICATION_H

#include <radio/radio.h>
#include "Aprs.h"
#include "Config.h"
#include "Timer.h"

class System;

class Communication {
public:
    explicit Communication(System *system);

    bool begin(RadioEvents_t *radioEvents);
    void update(bool sendTelemetry, bool sendPosition, bool sendStatus);
    void sent();
    void received(uint8_t * payload, uint16_t size, int16_t rssi, int8_t snr);

    bool sendMessage(const char* destination, const char* message, const char* ackToConfirm = nullptr);
    bool sendPosition(const char* comment);
    bool sendStatus(const char* comment);
    bool sendTelemetry();
    bool sendTelemetryParams();

    inline unsigned long getWatchdogLoraTxTimeLeft() const {
        return timerLoraTxWatchdog.getTimeLeft();
    }

    inline bool hasWatchdogLoraTxExpired() const {
        return timerLoraTxWatchdog.hasExpired();
    }

    void resetWatchdogLoraTx();

    bool shouldSendTelemetryParams = false;
private:
    System* system;

    uint8_t buffer[TRX_BUFFER]{};
    AprsPacket aprsPacketTx;
    AprsPacketLite aprsPacketRx;
    uint16_t telemetrySequenceNumber = 0;
    Timer timerLoraTxWatchdog = Timer(TIME_LORA_TX_WATCHDOG);

    bool send();
};

#endif //CUBECELL_MONITORING_COMMUNICATION_H
