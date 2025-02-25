#include "Threads/WeatherThread.h"
#include "ArduinoLog.h"
#include "System.h"
#include "utils.h"

WeatherThread::WeatherThread(System *system) : MyThread(system, system->settings.weather.intervalCheck, PSTR("WEATHER")) {
    enabled = system->settings.weather.enabled;
    force = true;
}

bool WeatherThread::init() {
    if (bmp280.begin()) {
        sensorType = BMP280;

        bmp280.setSampling(Adafruit_BMP280::MODE_FORCED,
                           Adafruit_BMP280::SAMPLING_X1, // Temp. oversampling
                           Adafruit_BMP280::SAMPLING_X1, // Pressure oversampling
                           Adafruit_BMP280::FILTER_OFF, Adafruit_BMP280::STANDBY_MS_1000);

        Log.info("[WEATHER] BMP280 found OK");
    } else if (bme280.begin()) {
        sensorType = BME280;

        bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                           Adafruit_BME280::SAMPLING_X1, // Temp. oversampling
                           Adafruit_BME280::SAMPLING_X1, // Pressure oversampling
                           Adafruit_BME280::SAMPLING_X1, // Humidity oversampling
                           Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_1000);

        Log.info("[WEATHER] BME280 found OK");
    } else {
        sensorType = None;
        return false;
    }

    for (uint8_t i = 0; i < 3; i++) {
        delayWdt(150);
        readTemperature(); // To have correct value for first runOnce
    }
    return true;
}

bool WeatherThread::runOnce() {
    temperature = readTemperature();
    humidity = readHumidity();
    pressure = readTemperature();

    if (pressure <= 700 || pressure >= 1200 || temperature >= 50 || temperature <= -15) {
        Log.warningln(F("[WEATHER] Error ! Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);
        begin();
        return false;
    }

    pressure *= static_cast<float>(pow((1 - system->settings.aprs.altitude / 44330.0), -5.255));

    Log.infoln(F("[WEATHER] Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);

    return true;
}

float WeatherThread::readTemperature() {
    switch (sensorType) {
        case BME280:
            bme280.takeForcedMeasurement();
            return bme280.readTemperature();
        case BMP280:
            bmp280.takeForcedMeasurement();
            return bmp280.readTemperature();
        default:
            return 0;
    }
}

float WeatherThread::readHumidity() {
    switch (sensorType) {
        case BME280:
            return bme280.readHumidity();
        case BMP280:
        default:
            return 0;
    }
}

float WeatherThread::readPressure() {
    switch (sensorType) {
        case BME280:
            return bme280.readPressure();
        case BMP280:
            return bmp280.readPressure();
        default:
            return 0;
    }
}
