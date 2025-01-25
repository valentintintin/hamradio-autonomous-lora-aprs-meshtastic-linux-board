#include "Threads/Send/SendTelemetriesThread.h"
#include "System.h"
#include "ArduinoLog.h"

SendTelemetriesThread::SendTelemetriesThread(System *system) : SendThread(system, system->settings.aprs.intervalTelemetry, PSTR("SEND_TELEMETRIES"), system->settings.aprs.telemetryEnabled) {
    force = true;
}

bool SendTelemetriesThread::runOnce() {
    return system->communication.sendTelemetry();
}

bool SendTelemetriesThread::shouldRun(const unsigned long time) {
    // We want to be at boot so bypass super class
    return MyThread::shouldRun(time); // NOLINT(*-parent-virtual-call)
}
