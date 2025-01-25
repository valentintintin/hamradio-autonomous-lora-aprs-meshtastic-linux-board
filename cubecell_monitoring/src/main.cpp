#include <Arduino.h>
#include <radio/radio.h>
#include <ArduinoLog.h>
#include "System.h"

#ifdef USE_SOFT_SERIAL
#include <softSerial.h>
softSerial *SerialPi = new softSerial(PIN_PI_TX, PIN_PI_RX);
#else
HardwareSerial *SerialPi = &Serial1;
#endif
JsonWriter serialJsonWriter(SerialPi);
char bufferText[BUFFER_LENGTH]{};

RadioEvents_t radioEvents;
TimerEvent_t wakeUpEvent;

CubeCell_NeoPixel pixels = CubeCell_NeoPixel(1, RGB, NEO_GRB + NEO_KHZ800);
System systemControl(&pixels, &wakeUpEvent);
Communication communication(&systemControl);

void sentEvent() {
    communication.sent();
}

void receivedEvent(uint8_t * payload, uint16_t size, int16_t rssi, int8_t snr) {
    communication.received(payload, size, rssi, snr);
}

void radioRxErrorEvent() {
    systemControl.serialError(PSTR("Radio RX error"));
    Radio.RxBoosted(0);
}

void radioTxTimeoutEvent() {
    systemControl.serialError(PSTR("Radio TX Timeout"));
    Radio.RxBoosted(0);
}

void radioRxTimeoutEvent() {
    systemControl.serialError(PSTR("Radio RX Timeout"));
    Radio.RxBoosted(0);
}

void onWakeUp() {
    CySoftwareReset();
}

void setup() {
    radioEvents.RxDone = receivedEvent;
    radioEvents.TxDone = sentEvent;
    radioEvents.RxError = radioRxErrorEvent;
    radioEvents.TxTimeout = radioTxTimeoutEvent;
    radioEvents.RxTimeout = radioRxTimeoutEvent;
    TimerInit(&wakeUpEvent, onWakeUp);

    boardInitMcu();

    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW); // 3.3V

    Serial.begin(115200);
#ifdef USE_SOFT_SERIAL
    SerialPi->begin(38400);
#else
    SerialPi->begin(115200);
#endif

    Log.begin(LOG_LEVEL, &Serial);

    systemControl.begin(&radioEvents);
}

void loop() {
    systemControl.update();
}