#include "ArduinoLog.h"
#include "System.h"
#include "MpptMonitor.h"
#include <EEPROM.h>

MpptMonitor::MpptMonitor(System *system, TwoWire &wire) : system(system), wire(wire) {
}

bool MpptMonitor::begin() {
    if (!charger.begin(wire)) {
        system->serialError(PSTR("[MPPT] Charger error"));

        return false;
    }

    init = true;

    return true;
}

bool MpptMonitor::update(bool force) {
    if (!force && !timer.hasExpired()) {
        return false;
    }

    timer.restart();

    if (!init && !begin()) {
        return false;
    }

    bool hasImportantChange = false;
    bool oldBoolValue;

    Log.traceln(F("[MPPT] Fetch charger data"));

    if (!charger.getStatusValue(SYS_STATUS, &status)) {
        system->serialError(PSTR("[MPPT] Fetch charger data status error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger status : %s"), mpptChg::getStatusAsString(status));
    }

    if (!charger.getIndexedValue(VAL_VS, &vs)) {
        system->serialError(PSTR("[MPPT] Fetch charger data VS error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger VS : %d"), vs);
    }

    if (!charger.getIndexedValue(VAL_IS, &is)) {
        system->serialError(PSTR("[MPPT] Fetch charger data IS error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger IS : %d"), is);
    }

    if (!charger.getIndexedValue(VAL_VB, &vb)) {
        system->serialError(PSTR("[MPPT] Fetch charger data VB error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger VB : %d"), vb);
    }

    if (!charger.getIndexedValue(VAL_IB, &ib)) {
        system->serialError(PSTR("[MPPT] Fetch charger data IB error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger IB : %d"), ib);
    }

    oldBoolValue = night;
    if (!charger.isNight(&night)) {
        system->serialError(PSTR("[MPPT] Fetch charger data night error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger night : %T"), night);
        hasImportantChange |= oldBoolValue != night;
    }

    oldBoolValue = alert;
    if (!charger.isAlert(&alert)) {
        system->serialError(PSTR("[MPPT] Fetch charger data alert error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger alert : %T"), alert);
        hasImportantChange |= oldBoolValue != alert;
    }

    oldBoolValue = powerEnabled;
    if (!charger.isPowerEnabled(&powerEnabled)) {
        system->serialError(PSTR("[MPPT] Fetch charger data power enabled error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger power enabled : %T"), powerEnabled);
        hasImportantChange |= oldBoolValue != powerEnabled;
    }

    oldBoolValue = watchdogEnabled;
    if (!charger.getWatchdogEnable(&watchdogEnabled)) {
        system->serialError(PSTR("[MPPT] Fetch charger data watchdog error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger watchdog : %T"), watchdogEnabled);
        hasImportantChange |= oldBoolValue != watchdogEnabled;
    }

    uint16_t oldIntValue = watchdogPowerOffTime;
    if (!charger.getWatchdogPoweroff(&watchdogPowerOffTime)) {
        system->serialError(PSTR("[MPPT] Fetch charger data watchdog poweroff error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger watchdog poweroff : %d"), watchdogPowerOffTime);
        hasImportantChange |= oldIntValue != watchdogPowerOffTime;
    }

    if (!charger.getWatchdogTimeout(&watchdogCounter)) {
        system->serialError(PSTR("[MPPT] Fetch charger data watchdog counter error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger watchdog counter : %d"), watchdogCounter);
    }

    if (!charger.getConfigurationValue(CFG_PWR_OFF_TH, &powerOffVoltage)) {
        system->serialError(PSTR("[MPPT] Fetch charger data power off voltage error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger power off voltage : %d"), powerOffVoltage);
    }

    if (!charger.getConfigurationValue(CFG_PWR_ON_TH, &powerOnVoltage)) {
        system->serialError(PSTR("[MPPT] Fetch charger data power on voltage error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger power on voltage : %d"), powerOnVoltage);
    }

    if (!charger.getIndexedValue(VAL_INT_TEMP, &temperature)) {
        system->serialError(PSTR("[MPPT] Fetch charger data temperature error"));
        init = false;
        return false;
    } else {
        Log.verboseln(F("[MPPT] Charger temperature : %d"), temperature);
    }

    serialJsonWriter
            .beginObject()
            .property(F("type"), PSTR("mppt"))
            .property(F("batteryVoltage"), vb)
            .property(F("batteryCurrent"), ib)
            .property(F("solarVoltage"), vs)
            .property(F("solarCurrent"), is)
            .property(F("currentCharge"), getCurrentCharge())
            .property(F("temperature"), temperature / 10.0)
            .property(F("status"), status)
            .property(F("statusString"), (char*) mpptChg::getStatusAsString(status))
            .property(F("night"), night)
            .property(F("alert"), alert)
            .property(F("watchdogEnabled"), watchdogEnabled)
            .property(F("watchdogPowerOffTime"), watchdogPowerOffTime)
            .property(F("watchdogCounter"), watchdogCounter)
            .property(F("powerEnabled"), powerEnabled)
            .property(F("powerOffVoltage"), powerOffVoltage)
            .property(F("powerOnVoltage"), powerOnVoltage)
            .endObject(); SerialPi->println();

    Log.infoln(F("[MPPT] Vb: %dmV Ib: %dmA Vs: %dmV Is: %dmA Ic: %dmA Status: %s Night: %T Alert: %T WD: %T WDOff: %ds WDCnt: %ds 5V: %T PowOffVolt: %dV PowOnVolt: %dV Temperature: %DC"), vb, ib, vs, is, getCurrentCharge(), mpptChg::getStatusAsString(status), night, alert, watchdogEnabled, watchdogPowerOffTime, watchdogCounter, powerEnabled, powerOffVoltage, powerOnVoltage, temperature / 10.0f);

    timer.restart();

    if (hasImportantChange) {
        system->forceSendTelemetry = true;
    }

    system->removeOneError();

    return true;
}

bool MpptMonitor::setWatchdog(uint16_t powerOffTime, uint8_t timeoutTime) {
    if (!init) {
        if (!begin()) {
            return false;
        }
    }

    bool enabled = powerOffTime > 0;
    system->forceSendTelemetry = watchdogEnabled != enabled || powerOffTime != watchdogPowerOffTime;

    if (enabled) {
        if (!charger.setWatchdogPoweroff(powerOffTime)) {
            system->serialError(PSTR("[MPPT_WATCHDOG]Change watchdog poweroff error"));
            init = false;
            system->forceSendTelemetry = false;
            return false;
        }

        watchdogPowerOffTime = powerOffTime;

        if (!charger.setWatchdogTimeout(timeoutTime)) {
            system->serialError(PSTR("[MPPT_WATCHDOG]Change watchdog counter error"));
            init = false;
            system->forceSendTelemetry = false;
            return false;
        }

        watchdogCounter = timeoutTime;
    }

    if (!charger.setWatchdogEnable(enabled)) {
        system->serialError(PSTR("[MPPT_WATCHDOG]Change watchdog enable error"));
        init = false;
        system->forceSendTelemetry = false;
        return false;
    }

    if (!charger.getWatchdogEnable(&watchdogEnabled)) {
        system->serialError(PSTR("[MPPT] Fetch charger data watchdog error"));
        init = false;
        system->forceSendTelemetry = false;
        return false;
    }

    watchdogManagedByUser = !enabled;

    Log.infoln(F("[MPPT_WATCHDOG] Watchdog state : %T. PowerOff : %d, Counter : %d"), enabled, powerOffTime, MPPT_WATCHDOG_TIMEOUT);

    timer.setExpired();
    system->removeOneError();

    return true;
}

bool MpptMonitor::setPowerOnOff(uint16_t powerOnVoltage, uint16_t powerOffVoltage) {
    Log.infoln(F("[MPPT_POWER] On : %dmV Off : %dmV"), powerOnVoltage, powerOffVoltage);

    return charger.setConfigurationValue(CFG_PWR_ON_TH, powerOnVoltage)
           && charger.setConfigurationValue(CFG_PWR_OFF_TH, powerOffVoltage);
}

void MpptMonitor::feedWatchdog() {
    if (watchdogEnabled && !alert && !watchdogManagedByUser) {
        Log.infoln(F("[MPPT_WATCHDOG] Feed watchdog"));
        setWatchdog(10);
    }
}
