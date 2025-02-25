#ifndef RP2040_LORA_APRS_WEATHERTHREAD_H
#define RP2040_LORA_APRS_WEATHERTHREAD_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

#include "MyThread.h"

class System;

enum SensorType {
    None,
    BME280,
    BMP280
};

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
    Adafruit_BMP280 bmp280 = Adafruit_BMP280();
    Adafruit_BME280 bme280 = Adafruit_BME280();
    SensorType sensorType = None;

    float temperature = 0;
    float humidity = 0;
    float pressure = 0;

    float readTemperature();
    float readHumidity();
    float readPressure();
};

#endif //RP2040_LORA_APRS_WEATHERTHREAD_H
