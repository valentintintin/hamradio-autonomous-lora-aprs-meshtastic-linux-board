#include "utils.h"
#include "System.h"

char bufferText[BUFFER_LENGTH]{};
uint8_t buffer[BUFFER_LENGTH]{};

System systemControl;

void setup() {
    systemControl.begin();
}

void loop() {
    systemControl.loop();
}

void delayWdt(const uint32_t milliseconds) {
    if (systemControl.settings.useInternalWatchdog) {
        const uint64_t startTime = millis();
        uint64_t elapsedTime = 0;

        while (elapsedTime < milliseconds) {
            if (milliseconds - elapsedTime >= 1000) {
                delay(1000);
                rp2040.wdt_reset();
            } else {
                delay(milliseconds - elapsedTime);
            }
            elapsedTime = millis() - startTime;
        }
    } else {
        delay(milliseconds);
    }
}

void ledBlink(const uint8_t howMany, const uint16_t milliseconds) {
    for (uint8_t i = 0; i < howMany; i++) {
        digitalWrite(PIN_LED, HIGH);
        delayWdt(milliseconds);
        digitalWrite(PIN_LED, LOW);
        delayWdt(milliseconds);
    }
}

void getDateTimeStringFromEpoch(uint64_t epoch, char* buffer, size_t size) {
    const time_t epochTimeT = epoch;
    const tm ts = *localtime(&epochTimeT);
    strftime(buffer, size, "%Y-%m-%dT%H:%M:%SZ", &ts);
    // snprintf(buffer, size, "%d-%d-%dT%d:%d:%dZ", ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec);
}

float clamp(const float v, const float lo, const float hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}
