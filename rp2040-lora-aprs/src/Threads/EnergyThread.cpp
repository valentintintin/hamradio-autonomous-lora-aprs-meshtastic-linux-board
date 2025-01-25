#include "ArduinoLog.h"
#include "System.h"
#include "Threads/EnergyThread.h"

EnergyThread::EnergyThread(System *system, const char *name) : MyThread(system, system->settings.energy.intervalCheck, name) {
    force = true;
}

bool EnergyThread::runOnce() {
    Log.traceln(F("[ENERGY] Fetch charger data"));

    if (!fetchVoltageBattery()) {
        Log.errorln(F("[MPPT] Fetch charger data VB error"));
        return false;
    }

    if (!fetchCurrentBattery()) {
        Log.errorln(F("[MPPT] Fetch charger data IB error"));
        return false;
    }

    if (!fetchVoltageSolar()) {
        Log.errorln(F("[MPPT] Fetch charger data VS error"));
        return false;
    }

    if (!fetchCurrentSolar()) {
        Log.errorln(F("[MPPT] Fetch charger data IS error"));
        return false;
    }

    Log.infoln(F("[ENERGY] Vb: %dmV Ib: %dmA Vs: %dmV Is: %dmA Ic: %dmA Night: %T"), vb, ib, vs, is, getCurrentCharge(), isNight());

    return true;
}