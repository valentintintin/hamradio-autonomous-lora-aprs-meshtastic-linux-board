
#include "Gpio.h"
#include "ArduinoLog.h"
#include "Config.h"
#include "System.h"

Gpio::Gpio(System *system) : system(system) {
    setRelay1(relay1);
    setRelay2(relay2);
    initialized = true;
}

void Gpio::setState(uint8_t pin, bool enabled, const char* name, bool &status, bool inverted) {
    pinMode(pin, OUTPUT);

    if (inverted) {
        digitalWrite(pin, !enabled);
    } else {
        digitalWrite(pin, enabled);
    }

    status = enabled;

    if (initialized) {
        Log.infoln(F("[GPIO] %s (%d) change to state %T"), name, pin, status);

        printJson();
    }
}

void Gpio::setRelay1(bool enabled) {
    setState(PIN_RELAY_1, enabled, PSTR("Relay 1"), relay1, true);
}

void Gpio::setRelay2(bool enabled) {
    if (relay2 != enabled) {
        system->forceSendTelemetry = true;
    }

    setState(PIN_RELAY_2, enabled, PSTR("Relay 2"), relay2, true);
}

uint16_t Gpio::getLdr() const {
//    return getAdcState(PIN_LDR, PSTR("LDR"));
    return getState(PIN_LDR, PSTR("LDR")) ? 0 : LDR_ALARM_LEVEL;
}

bool Gpio::getState(uint8_t pin, const char* name) const {
    pinMode(pin, INPUT);
    return digitalRead(pin);
}

uint16_t Gpio::getAdcState(uint8_t pin, const char *name) const {
    pinMode(pin, INPUT);
    return analogRead(pin);
}

void Gpio::printJson() const {
    Log.infoln(F("[GPIO] Relay 1: %T Relay 2: %T Box LDR: %d"), isRelay1Enabled(), isRelay2Enabled(), getLdr());

    serialJsonWriter
            .beginObject()
            .property(F("type"), PSTR("gpio"))
            .property(F("relay1"), isRelay1Enabled())
            .property(F("relay2"), isRelay2Enabled())
            .property(F("ldr"), getLdr())
            .endObject(); SerialPi->println();
}
