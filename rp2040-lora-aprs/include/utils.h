#ifndef RP2040_LORA_APRS_UTILS_H
#define RP2040_LORA_APRS_UTILS_H

#include <Arduino.h>
#include <ArduinoLog.h>

void ledBlink(uint8_t howMany = 3, uint16_t milliseconds = 250);
void delayWdt(uint32_t milliseconds);

#endif //RP2040_LORA_APRS_UTILS_H
