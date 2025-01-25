#ifndef RP2040_LORA_APRS_PICOSLEEP_H
#define RP2040_LORA_APRS_PICOSLEEP_H

#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <ctime>

void epoch_to_datetime(time_t epoch, datetime_t *dt);

void cpuDeepSleep(uint32_t msecs);

#endif //RP2040_LORA_APRS_PICOSLEEP_H
