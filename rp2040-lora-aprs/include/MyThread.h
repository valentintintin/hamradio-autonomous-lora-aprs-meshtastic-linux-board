#ifndef RP2040_LORA_APRS_MYTHREAD_H
#define RP2040_LORA_APRS_MYTHREAD_H

#include "Thread.h"
#include "config.h"

class System;

class MyThread : public Thread {
public:
    explicit MyThread(System *system, unsigned long interval, const char *name, bool noLog = false, bool enabled = true);

    bool begin();
    void run() override;
    void forceRun();

    bool shouldRun(unsigned long time) override;

    inline void setRunned() {
        runned();
    }

    inline bool hasError() const {
        return lastUpdateHasError;
    }

    inline uint64_t timeBeforeRun() const {
        return _cached_next_run - millis();
    }
protected:
    virtual bool init() {
        return true;
    }

    inline bool isAfterBoot() const {
        return millis() > TIME_AFTER_BOOT;
    }

    virtual bool runOnce() = 0;

    System *system;
    bool force = false;
    bool initiated = false;
    bool lastUpdateHasError = false;
    bool noLog = false;
};


#endif //RP2040_LORA_APRS_MYTHREAD_H
