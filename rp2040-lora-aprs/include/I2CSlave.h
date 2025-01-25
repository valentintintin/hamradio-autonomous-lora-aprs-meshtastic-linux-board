#ifndef RP2040_LORA_APRS_I2CSLAVE_H
#define RP2040_LORA_APRS_I2CSLAVE_H

#include <Wire.h>

#define REG_BATTERY_VOLTAGE 0x0
#define REG_BATTERY_CURRENT 0x2
#define REG_SOLAR_VOLTAGE 0x4
#define REG_SOLAR_CURRENT 0x6
#define REG_TEMPERATURE 0x8
#define REG_PRESSURE 0xA
#define REG_HUMIDITY 0xC
#define REG_SECONDS 0xE
#define REG_MINUTES 0x10
#define REG_HOURS 0x12
#define REG_DAYS 0x14
#define REG_MONTHS 0x16
#define REG_YEARS 0x18
#define REG_PING 0x1A

#define HAS_POWER 0b1
#define HAS_WEATHER 0b10
#define HAS_RTC 0b100

class System;

class I2CSlave {
public:
    static void begin(System *system);
    static void end();
private:
    static System *system;

    static uint8_t currentRegToRead;

    static void onRequest();
    static void onReceive(int bytes);
};


#endif //RP2040_LORA_APRS_I2CSLAVE_H
