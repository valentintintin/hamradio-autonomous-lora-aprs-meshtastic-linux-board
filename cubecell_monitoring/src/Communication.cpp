#include "Communication.h"
#include "ArduinoLog.h"
#include "System.h"

Communication::Communication(System *system) : system(system) {
    system->communication = this;

    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    aprsPacketTx.position.symbol = APRS_SYMBOL;
    aprsPacketTx.position.overlay = APRS_SYMBOL_TABLE;
    aprsPacketTx.position.latitude = APRS_LATITUDE;
    aprsPacketTx.position.longitude = APRS_LONGITUDE;
    aprsPacketTx.position.altitudeFeet = APRS_ALTITUDE * 3.28;
    aprsPacketTx.position.altitudeInComment = false;
    aprsPacketTx.position.withWeather = true;
    aprsPacketTx.weather.useHumidity = true;
    aprsPacketTx.weather.useTemperature = true;
    aprsPacketTx.weather.usePressure = true;

//    strcpy_P(aprsPacketTx.weather.device, PSTR("4HVV"));
}

bool Communication::begin(RadioEvents_t *radioEvents) {
    if (Radio.Init(radioEvents)) {
        system->serialError(PSTR("[RADIO] Init error"));

        return false;
    }

    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, false, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, false, 0, LORA_IQ_INVERSION_ON, 5000);

    Radio.RxBoosted(0);

    return true;
}

void Communication::update(bool sendTelemetry, bool sendPosition, bool sendStatus) {
    Radio.IrqProcess();

    if (sendStatus && system->isFunctionAllowed(EEPROM_ADDRESS_APRS_POSITION)) {
        this->sendStatus(PSTR(APRS_STATUS));
    }

    if (sendPosition && system->isFunctionAllowed(EEPROM_ADDRESS_APRS_POSITION)) {
        this->sendPosition(PSTR(APRS_COMMENT));
    }

    if (sendTelemetry && system->isFunctionAllowed(EEPROM_ADDRESS_APRS_TELEMETRY)) {
        this->sendTelemetry();
    }
}

bool Communication::send() {
    System::delayWdt(100);
    Radio.IrqProcess();

    Log.traceln(F("[LORA_TX] Radio status : %d"), Radio.GetStatus());

    system->turnOnRGB(COLOR_YELLOW);

    uint8_t rxTry = 0;
    while (rxTry++ < 5 && USE_RF && !(Radio.GetStatus() == RF_RX_RUNNING || Radio.GetStatus() == RF_IDLE)) {
        Log.warningln(F("[LORA_TX] Locked. Radio Status : %d. Try : %d"), Radio.GetStatus(), rxTry);
        System::delayWdt(2500);
        Radio.IrqProcess();
    }

    if (rxTry == 5) {
        system->serialError(PSTR("[LORA_TX] Radio busy after retries, abort TX"));
        return false;
    }

    Log.traceln(F("[LORA_TX] Radio ready, status : %d. Try : %d"), Radio.GetStatus(), rxTry - 1);

    system->turnOnRGB(COLOR_RED);

    uint8_t size = Aprs::encode(&aprsPacketTx, bufferText);

    if (!size) {
        system->serialError(PSTR("[APRS] Error during string encode"), false);
        return false;
    }

    buffer[0] = '<';
    buffer[1]= 0xFF;
    buffer[2] = 0x01;

    for (uint8_t i = 0; i < size; i++) {
        buffer[i + 3] = bufferText[i];
        Log.verboseln(F("[LORA_TX] Payload[%d]=%X %c"), i, buffer[i], buffer[i]);
    }

    if (USE_RF) {
        Radio.Send(buffer, size + 3);
        Radio.IrqProcess();
    }

    serialJsonWriter
            .beginObject()
            .property(F("type"), PSTR("lora"))
            .property(F("state"), PSTR("tx"))
            .property(F("payload"), bufferText)
            .endObject(); SerialPi->println();

    Log.infoln(F("[LORA_TX] Start send : %s"), bufferText);

    if (!USE_RF) {
        System::delayWdt(1000);
        sent();
    }

    return true;
}

bool Communication::sendMessage(const char* destination, const char* message, const char* ackToConfirm) {
    Aprs::reset(&aprsPacketTx);

    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    strcpy(aprsPacketTx.message.destination, destination);
    strcpy(aprsPacketTx.message.message, message);

    if (strlen(ackToConfirm) > 0) {
        strcpy(aprsPacketTx.message.ackToConfirm, ackToConfirm);
    }

    aprsPacketTx.type = Message;

    return send();
}

bool Communication::sendTelemetry() {
    Aprs::reset(&aprsPacketTx);

    bool result = false;

    if (shouldSendTelemetryParams) {
        result = sendTelemetryParams();
        shouldSendTelemetryParams = !result;
    }

    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    sprintf_P(aprsPacketTx.comment, PSTR("Chg:%s Up:%ld"), mpptChg::getStatusAsString(system->mpptMonitor.getStatus()), millis() / 1000);
    aprsPacketTx.telemetries.telemetrySequenceNumber = telemetrySequenceNumber++;
    aprsPacketTx.telemetries.telemetriesAnalog[0].value = system->mpptMonitor.getVoltageBattery();
    aprsPacketTx.telemetries.telemetriesAnalog[1].value = system->mpptMonitor.getCurrentBattery();
    aprsPacketTx.telemetries.telemetriesAnalog[2].value = system->mpptMonitor.getVoltageSolar();
    aprsPacketTx.telemetries.telemetriesAnalog[3].value = system->mpptMonitor.getCurrentSolar();
    aprsPacketTx.telemetries.telemetriesAnalog[4].value = system->mpptMonitor.isWatchdogEnabled() ? system->mpptMonitor.getWatchdogCounter() : 0;
    aprsPacketTx.telemetries.telemetriesBoolean[0].value = system->mpptMonitor.isNight();
    aprsPacketTx.telemetries.telemetriesBoolean[1].value = system->mpptMonitor.isAlert();
    aprsPacketTx.telemetries.telemetriesBoolean[2].value = system->mpptMonitor.isWatchdogEnabled();
    aprsPacketTx.telemetries.telemetriesBoolean[3].value = system->mpptMonitor.isPowerEnabled();
    aprsPacketTx.telemetries.telemetriesBoolean[4].value = system->isBoxOpened();
    aprsPacketTx.telemetries.telemetriesBoolean[5].value = system->gpio.isRelay1Enabled();
    aprsPacketTx.telemetries.telemetriesBoolean[6].value = system->gpio.isRelay2Enabled();
    aprsPacketTx.telemetries.telemetriesBoolean[7].value = system->nbError > 0;

    aprsPacketTx.type = Telemetry;
    result |= send();

    return result;
}

bool Communication::sendTelemetryParams() {
    Aprs::reset(&aprsPacketTx);

    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    // Voltage battery between 0 and 15000mV
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[0].name, PSTR("VBat")); // 7
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[0].unit, PSTR("V"));
    aprsPacketTx.telemetries.telemetriesAnalog[0].equation.b = 0.001;

    // Current charge between 0mA and 2000mA
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[1].name, PSTR("IBat")); // 6
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[1].unit, PSTR("mA"));

    // Voltage battery between 0 and 30000mV
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[2].name, PSTR("VSol")); // 5
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[2].unit, PSTR("V"));
    aprsPacketTx.telemetries.telemetriesAnalog[2].equation.b = 0.001;

    // Current charge between 0mA and 2000mA
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[3].name, PSTR("ISol")); // 5
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[3].unit, PSTR("mA"));

    // Watchdog poweroff in seconds
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[4].name, PSTR("Slep")); // 4
    strcpy_P(aprsPacketTx.telemetries.telemetriesAnalog[4].unit, PSTR("min"));
    aprsPacketTx.telemetries.telemetriesAnalog[4].equation.b = 0.0166;

    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[0].name, PSTR("Night"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[1].name, PSTR("Alrt"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[2].name, PSTR("WDog"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[3].name, PSTR("5V"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[4].name, PSTR("Box"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[5].name, PSTR("R1"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[6].name, PSTR("R2"));
    strcpy_P(aprsPacketTx.telemetries.telemetriesBoolean[7].name, PSTR("Err"));

    aprsPacketTx.type = TelemetryLabel;
    bool result = send();

    aprsPacketTx.type = TelemetryUnit;
    result |= send();

    aprsPacketTx.type = TelemetryEquation;
    result |= send();

    return result;
}

bool Communication::sendPosition(const char* comment) {
    Aprs::reset(&aprsPacketTx);

    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    aprsPacketTx.position.symbol = APRS_SYMBOL;
    aprsPacketTx.position.overlay = APRS_SYMBOL_TABLE;
    aprsPacketTx.position.latitude = APRS_LATITUDE;
    aprsPacketTx.position.longitude = APRS_LONGITUDE;
    aprsPacketTx.position.altitudeFeet = APRS_ALTITUDE * 3.28;
    aprsPacketTx.position.altitudeInComment = false;

    aprsPacketTx.weather.useHumidity = true;
    aprsPacketTx.weather.useTemperature = true;
    aprsPacketTx.weather.usePressure = true;

    aprsPacketTx.telemetries.telemetrySequenceNumber = telemetrySequenceNumber++;
    aprsPacketTx.telemetries.telemetriesAnalog[0].value = system->mpptMonitor.getVoltageBattery();
    aprsPacketTx.telemetries.telemetriesAnalog[1].value = system->mpptMonitor.getCurrentBattery();
    aprsPacketTx.telemetries.telemetriesAnalog[2].value = system->mpptMonitor.getVoltageSolar();
    aprsPacketTx.telemetries.telemetriesAnalog[3].value = system->mpptMonitor.getCurrentSolar();
    aprsPacketTx.telemetries.telemetriesAnalog[4].value = system->mpptMonitor.isWatchdogEnabled() ? system->mpptMonitor.getWatchdogCounter() : 0;
    aprsPacketTx.telemetries.telemetriesBoolean[0].value = system->mpptMonitor.isNight();
    aprsPacketTx.telemetries.telemetriesBoolean[1].value = system->mpptMonitor.isAlert();
    aprsPacketTx.telemetries.telemetriesBoolean[2].value = system->mpptMonitor.isWatchdogEnabled();
    aprsPacketTx.telemetries.telemetriesBoolean[3].value = system->mpptMonitor.isPowerEnabled();
    aprsPacketTx.telemetries.telemetriesBoolean[4].value = system->isBoxOpened();
    aprsPacketTx.telemetries.telemetriesBoolean[5].value = system->gpio.isRelay1Enabled();
    aprsPacketTx.telemetries.telemetriesBoolean[6].value = system->gpio.isRelay2Enabled();
    aprsPacketTx.telemetries.telemetriesBoolean[7].value = system->nbError > 0;

    aprsPacketTx.position.withWeather = !system->weatherSensors.hasError();

    if (APRS_TELEMETRY_IN_POSITION || !aprsPacketTx.position.withWeather) {
        aprsPacketTx.position.withTelemetry = true;
        sprintf_P(aprsPacketTx.comment, PSTR("Up:%ld"), millis() / 1000);
        aprsPacketTx.telemetries.telemetrySequenceNumber = telemetrySequenceNumber++;
    }

    aprsPacketTx.type = Position;
    aprsPacketTx.position.withWeather = system->weatherSensors.getPressure() > 0;

    aprsPacketTx.weather.temperatureFahrenheit = system->weatherSensors.getTemperature() * 9 / 5 + 32;
    aprsPacketTx.weather.humidity = system->weatherSensors.getHumidity();
    aprsPacketTx.weather.pressure = system->weatherSensors.getPressure();

    return send();
}

bool Communication::sendStatus(const char* comment) {
    Aprs::reset(&aprsPacketTx);

    strcpy_P(aprsPacketTx.path, PSTR(APRS_PATH));
    strcpy_P(aprsPacketTx.source, PSTR(APRS_CALLSIGN));
    strcpy_P(aprsPacketTx.destination, PSTR(APRS_DESTINATION));

    strcpy(aprsPacketTx.comment, comment);

    aprsPacketTx.type = Status;

    return send();
}

void Communication::sent() {
    Radio.IrqProcess();

    system->turnOffRGB();

    Log.infoln(F("[LORA_TX] End"));

    timerLoraTxWatchdog.restart();

    Radio.RxBoosted(0);
}

void Communication::received(uint8_t * payload, uint16_t size, int16_t rssi, int8_t snr) {
    Log.traceln(F("[LORA_RX] Payload of size %d, RSSI : %d and SNR : %d"), size, rssi, snr);
    Log.infoln(F("[LORA_RX] %s"), payload);

    serialJsonWriter
            .beginObject()
            .property(F("type"), PSTR("lora"))
            .property(F("state"), PSTR("rx"))
            .property(F("payload"), payload)
            .endObject(); SerialPi->println();

    for (uint16_t i = 0; i < size; i++) {
        Log.verboseln(F("[LORA_RX] Payload[%d]=%X %c"), i, payload[i], payload[i]);
    }

    system->turnOnRGB(COLOR_GREEN);
    system->turnOffRGB();

    bool shouldTx = false;

    if (!Aprs::decode(reinterpret_cast<const char *>(payload + sizeof(uint8_t) * 3), &aprsPacketRx)) {
        system->serialError(PSTR("[APRS] Error during decode"), false);
    } else {
        Log.traceln(F("[APRS] Decoded from %s to %s via %s"), aprsPacketRx.source, aprsPacketRx.destination, aprsPacketRx.path);

        if (strstr_P(aprsPacketRx.message.destination, PSTR(APRS_CALLSIGN)) != nullptr) {
            Log.traceln(F("[APRS] Message for me : %s"), aprsPacketRx.content);

            if (strlen(aprsPacketRx.message.message) > 0) {
                shouldTx = sendMessage(aprsPacketRx.source, PSTR(""), aprsPacketRx.message.ackToConfirm);

                bool processCommandResult = system->command.processCommand(aprsPacketRx.message.message);

                shouldTx |= sendMessage(aprsPacketRx.source, processCommandResult ? PSTR("OK") : PSTR("KO"));
            }
        } else if (system->isFunctionAllowed(EEPROM_ADDRESS_APRS_DIGIPEATER)) {
            shouldTx = Aprs::canBeDigipeated(aprsPacketRx.path, APRS_CALLSIGN);

            Log.traceln(F("[APRS] Message should TX : %T"), shouldTx);

            if (shouldTx) {
                Log.infoln(F("[APRS] Message digipeated via %s"), aprsPacketRx.path);
                strcpy(aprsPacketTx.source, aprsPacketRx.source);
                strcpy(aprsPacketTx.path, aprsPacketRx.path);
                strcpy(aprsPacketTx.destination, aprsPacketRx.destination);
                strcpy(aprsPacketTx.content, aprsPacketRx.content);
                aprsPacketTx.type = RawContent;
                shouldTx = send();
            }
        }
    }

    if (!shouldTx) {
        system->turnOffRGB();
        Radio.IrqProcess();
        Radio.RxBoosted(0);
    }
}

void Communication::resetWatchdogLoraTx() {
    timerLoraTxWatchdog.restart();

    Log.infoln(F("[LORA_TX_WATCHDOG] Reset watchdog LoRa TX timer to %ums"), timerLoraTxWatchdog.getTimeLeft());
}
