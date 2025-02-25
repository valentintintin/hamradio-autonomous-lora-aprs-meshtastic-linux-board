#include "Threads/Energy/EnergyMpptChgThread.h"
#include "ArduinoLog.h"
#include "System.h"

EnergyMpptChgThread::EnergyMpptChgThread(System *system, uint16_t *ocv, const size_t numOcvPoints) : EnergyThread(system, PSTR("ENERGY_MPPTCHG"), ocv, numOcvPoints) {
    charger = &system->mpptChgCharger;
}

bool EnergyMpptChgThread::init() {
    const SettingsEnergy settings = system->settings.energy;
    return charger->begin() && setPowerOnOff(settings.mpptPowerOffVoltage, settings.mpptPowerOnVoltage);
}

void EnergyMpptChgThread::run() {
    EnergyThread::run();
    fetchOthersData();
}

bool EnergyMpptChgThread::fetchVoltageBattery() {
    return charger->getIndexedValue(VAL_VB, &vb);
}

bool EnergyMpptChgThread::fetchCurrentBattery() {
    return charger->getIndexedValue(VAL_IB, &ib);
}

bool EnergyMpptChgThread::fetchVoltageSolar() {
    return charger->getIndexedValue(VAL_VS, &vs);
}

bool EnergyMpptChgThread::fetchCurrentSolar() {
    return charger->getIndexedValue(VAL_IS, &is);
}

bool EnergyMpptChgThread::fetchOthersData() {
    return charger->isNight(&_isNight) && charger->getStatusValue(SYS_STATUS, &status);
}

bool EnergyMpptChgThread::setPowerOnOff(const uint16_t powerOnVoltage, const uint16_t powerOffVoltage) const {
    Log.infoln(F("[ENERGY_MPPTCHG] Power On : %dmV and Power Off : %dmV"), powerOnVoltage, powerOffVoltage);

    if (!charger->setConfigurationValue(CFG_PWR_ON_TH, powerOnVoltage)
        || !charger->setConfigurationValue(CFG_PWR_OFF_TH, powerOffVoltage)) {
        Log.warningln(F("[ENERGY_MPPTCHG] Failed to set power on off"));
        return false;
    }

    return true;
}
