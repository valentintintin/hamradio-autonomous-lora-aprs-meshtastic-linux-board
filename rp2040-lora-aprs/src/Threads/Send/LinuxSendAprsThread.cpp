#include "Threads/Send/LinuxSendAprsThread.h"
#include "System.h"

LinuxSendAprsThread::LinuxSendAprsThread(System *system) : SendThread(system, system->settings.linux.intervalSendItem, PSTR("SEND_LINUX_APRS"), system->settings.linux.aprsSendItemEnabled) {}

bool LinuxSendAprsThread::runOnce() {
    return system->communication.sendItem(system->settings.linux.itemName,
                                          system->settings.linux.symbol,
                                          system->settings.linux.symbolTable,
                                          system->settings.linux.itemComment,
                                          system->settings.linux.latitude,
                                          system->settings.linux.longitude,
                                          system->settings.linux.altitude,
                                          system->watchdogLinux->isFed() || force);
}

bool LinuxSendAprsThread::shouldRun(const unsigned long time) {
    return SendThread::shouldRun(time) && system->watchdogLinux->enabled;
}
