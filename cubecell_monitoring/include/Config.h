#ifndef CUBECELL_MONITORING_CONFIG_H
#define CUBECELL_MONITORING_CONFIG_H

#include <softSerial.h>
#include <JsonWriter.h>

#define LOG_LEVEL LOG_LEVEL_TRACE
#define BUFFER_LENGTH 512

//#define USE_SCREEN
#define USE_SOFT_SERIAL
#define USE_RF true
#define USE_RTC false
#define SET_RTC 0

#define RF_FREQUENCY 433775000 // Hz
#define LORA_BANDWIDTH 0 // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_OUTPUT_POWER 22
#define TRX_BUFFER 259 // 256 + 3 because 3 bytes for LoRa APRS

#define INTERVAL_TELEMETRY_APRS 900000 // 15 minutes
#define INTERVAL_POSITION_AND_WEATHER_APRS 900000 // 15 minutes
#define INTERVAL_STATUS_APRS 86400000 // 1 day
#define INTERVAL_ALARM_BOX_OPENED_APRS 300000 // 5 minutes

#define INTERVAL_WEATHER 30000 // 30 secondes
#define INTERVAL_MPPT 30000 // 30 secondes
#define INTERVAL_STATE 30000 // 30 secondes
#define INTERVAL_BLINKER 5000 // 5 secondes

#define APRS_CALLSIGN "F4HVV-15"
#define APRS_PATH "WIDE1-1"
#define APRS_DESTINATION "APLV1" // APL is LoRa : http://www.aprs.org/aprs11/tocalls.txt --> V1 is a new for this project
#define APRS_SYMBOL '#'
#define APRS_SYMBOL_TABLE 'L'
#define APRS_LATITUDE 45.325776
#define APRS_LONGITUDE 5.63658087
#define APRS_ALTITUDE 830
#define APRS_COMMENT "f4hvv.valentin-saugnier.fr"
#define APRS_STATUS "Digi & Meshtastic Gaulix 868"
#define APRS_TELEMETRY_IN_POSITION false

#define PIN_RELAY_1 GPIO0
#define PIN_RELAY_2 GPIO5
#define PIN_LDR GPIO3

#define RELAY_1_INITIAL_STATE false
#define RELAY_2_INITIAL_STATE false

#ifdef USE_SOFT_SERIAL
#define PIN_PI_RX GPIO2
#define PIN_PI_TX GPIO1
#endif

#define MPPT_WATCHDOG_TIMEOUT 255
#define LDR_ALARM_LEVEL 1000
#define VOLTAGE_TO_SLEEP 11000
#define DURATION_TO_WAIT_BEFORE_SLEEP 120000 // 2 minutes
#define SLEEP_DURATION 3600000 // 60 minutes
#define MAX_ERROR_TO_RESET 10
#define TIME_LORA_TX_WATCHDOG 1200000 // 20 minutes
#define CURRENT_YEAR 2024

#define EEPROM_ADDRESS_MPPT_WATCHDOG 0
#define EEPROM_ADDRESS_APRS_DIGIPEATER 1
#define EEPROM_ADDRESS_APRS_TELEMETRY 2
#define EEPROM_ADDRESS_APRS_POSITION 3
#define EEPROM_ADDRESS_SLEEP 4
#define EEPROM_ADDRESS_RESET_ON_ERROR 5
#define EEPROM_ADDRESS_WATCHDOG 6
#define EEPROM_ADDRESS_WATCHDOG_LORA_TX 7
#define EEPROM_ADDRESS_VERSION 0xFF
#define EEPROM_VERSION 6

#define COLOR_RED 0x500000
#define COLOR_VIOLET 0x500050
#define COLOR_BLUE 0x000050
#define COLOR_YELLOW 0xcfcf02
#define COLOR_ORANGE 0xa88d32
#define COLOR_GREEN 0x005000
#define COLOR_CYAN 0x00bbff

#ifdef USE_SOFT_SERIAL
#include <softSerial.h>
extern softSerial *SerialPi;
#else
extern HardwareSerial *SerialPi;
#endif
extern JsonWriter serialJsonWriter;
extern char bufferText[BUFFER_LENGTH];

#endif //CUBECELL_MONITORING_CONFIG_H


/*
 * Arduino log : format
* %s	display as string (char*)
* %S    display as string from flash memory (__FlashStringHelper* or char[] PROGMEM)
* %c	display as single character
* %C    display as single character or as hexadecimal value (prefixed by `0x`) if not a printable character
* %d	display as integer value
* %l	display as long value
* %u	display as unsigned long value
* %x	display as hexadecimal value
* %X	display as hexadecimal value prefixed by `0x` and leading zeros
* %b	display as binary number
* %B	display as binary number, prefixed by `0b`
* %t	display as boolean value "t" or "f"
* %T	display as boolean value "true" or "false"
* %D,%F display as double value
* %p    display a  printable object
 */