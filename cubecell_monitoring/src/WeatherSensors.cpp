#include "WeatherSensors.h"
#include "ArduinoLog.h"
#include "System.h"

WeatherSensors::WeatherSensors(System *system) : system(system) {
}

bool WeatherSensors::begin() {
    return sensor.begin();
}

bool WeatherSensors::update(bool force) {
    if (!force && !timer.hasExpired()) {
        return false;
    }

    Log.traceln(F("[WEATHER] Fetch weather sensors data"));

    temperature = sensor.read_temperature_c();
    humidity = sensor.read_humidity();
    pressure = sensor.read_pressure();

    timer.restart();

    if (pressure <= 700 || pressure >= 1200 || temperature >= 50 || temperature <= -15) {
        lastUpdateHasError = true;
        Log.warningln(F("[WEATHER] Error ! Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);
        system->serialError(PSTR("[WEATHER] Fetch weather error"));
        return false;
    }

    lastUpdateHasError = false;

    serialJsonWriter
            .beginObject()
            .property(F("type"), PSTR("weather"))
            .property(F("temperature"), temperature)
            .property(F("humidity"), humidity)
            .property(F("pressure"), pressure)
            .endObject(); SerialPi->println();

    Log.infoln(F("[WEATHER] Temperature: %FC Humidity: %F%% Pressure: %FhPa"), temperature, humidity, pressure);

    return true;
}