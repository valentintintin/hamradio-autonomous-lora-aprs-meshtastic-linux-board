#ifndef RP2040_LORA_APRS_COMMUNICATION_H
#define RP2040_LORA_APRS_COMMUNICATION_H

#include <RadioLib.h>
#include "Aprs.h"
#include "config.h"

class System;

class Communication {
public:
    explicit Communication(System *system);

    bool begin();
    void update();
    void received(uint8_t * payload, uint16_t size, float rssi, float snr);

    bool sendMessage(const char* destination, const char* message, const char* ackToConfirm = nullptr);
    bool sendPosition(const char* comment);
    bool sendStatus(const char* comment);
    bool sendTelemetry();
    bool sendTelemetryParams();
    bool sendItem(const char* name, char symbol, char symbolTable, const char* comment, double latitude, double longitude, uint16_t altitude, bool alive = true);

    bool sendRaw(const uint8_t* payload, size_t size);
    bool changeLoRaSettings(float frequency, uint16_t bandwidth, uint8_t spreadingFactor, uint8_t codingRate, uint8_t outputPower);

    bool shouldSendTelemetryParams = false;

    inline bool hasError() const {
        return _hasError;
    }
private:
    static volatile bool hasInterrupt;

    static void setHasInterrupt();

    System* system;

    uint8_t buffer[TRX_BUFFER]{};
    AprsPacket aprsPacketTx{};
    AprsPacketLite aprsPacketRx{};
    SX1262 lora = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY, SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    bool _hasError = false;

    bool startReceive();
    bool sendAprsFrame();
    bool send(size_t size);
    void sent();
    bool isChannelActive();
    void prepareTelemetry();
};

#endif //RP2040_LORA_APRS_COMMUNICATION_H
