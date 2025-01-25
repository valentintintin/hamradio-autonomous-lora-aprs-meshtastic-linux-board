#include "Threads/WatchdogThread.h"

WatchdogThread::WatchdogThread(System *system, const unsigned long interval, const char *name, const bool enabled) : MyThread(system, interval, name, false, enabled) {
}

bool WatchdogThread::shouldRun(const unsigned long time) {
    return MyThread::shouldRun(time) && millis() > TIME_AFTER_BOOT;
}