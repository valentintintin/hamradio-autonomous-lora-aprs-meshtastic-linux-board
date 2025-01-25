#ifndef RP2040_LORA_APRS_WEATHERTHREAD_H
#define RP2040_LORA_APRS_WEATHERTHREAD_H

#include "vt_bme280"

#include "MyThread.h"

using namespace vt;

class System;

class WeatherThread : public MyThread {
public:
    explicit WeatherThread(System *system);

    inline float getTemperature() const {
        return temperature;
    }

    inline float getHumidity() const {
        return humidity;
    }

    inline float getPressure() const {
        return pressure;
    }
protected:
    bool init() override;
    bool runOnce() override;
private:
    bme280_t sensor = bme280_t();

    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
};

#endif //RP2040_LORA_APRS_WEATHERTHREAD_H
