#include "Threads/WeatherThread.h"
#include "ArduinoLog.h"
#include "System.h"
#include "utils.h"

WeatherThread::WeatherThread(System *system) : MyThread(system, system->settings.weather.intervalCheck, PSTR("WEATHER")) {
    enabled = system->settings.weather.enabled;
    force = true;
}

bool WeatherThread::init() {
    if (sensor.begin()) {
        for (uint8_t i = 0; i < 3; i++) {
            delayWdt(150);
            sensor.read_temperature_c(); // To have correct value for first runOnce
        }
        return true;
    }

    return false;
}

bool WeatherThread::runOnce() {
    temperature = sensor.read_temperature_c();
    humidity = sensor.read_humidity();
    pressure = sensor.read_pressure();

    if (pressure <= 700 || pressure >= 1200 || temperature >= 50 || temperature <= -15) {
        Log.warningln(F("[WEATHER] Error ! Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);
        begin();
        return false;
    }

    pressure *= static_cast<float>(pow((1 - system->settings.aprs.altitude / 44330.0), -5.255));

    Log.infoln(F("[WEATHER] Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);

    return true;
}