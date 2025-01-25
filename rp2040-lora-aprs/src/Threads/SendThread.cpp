#include "Threads/SendThread.h"
#include "System.h"
#include "ArduinoLog.h"

SendThread::SendThread(System *system, const unsigned long interval, const char *name, const bool enabled) : MyThread(system, interval, name, false, enabled) {
}

bool SendThread::shouldRun(const unsigned long time) {
    return MyThread::shouldRun(time) && isAfterBoot();
}