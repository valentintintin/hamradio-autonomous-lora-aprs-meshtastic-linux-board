#ifndef CUBECELL_MONITORING_ENERGY_H
#define CUBECELL_MONITORING_ENERGY_H

#include "MyThread.h"
#include "Timer.h"
#include "mpptChg.h"
#include "Communication.h"
#include "config.h"

class EnergyThread : public MyThread {
public:
    explicit EnergyThread(System *system, const char *name);

    inline int16_t getVoltageBattery() const {
        return vb;
    }

    inline int16_t getVoltageSolar() const {
        return vs;
    }

    inline int16_t getCurrentBattery() const {
        return ib;
    }

    inline int16_t getCurrentSolar() const {
        return is;
    }

    inline int16_t getCurrentCharge() const {
        return is - ib;
    }

    virtual inline bool isNight() const {
        return vs < 2000;
    }
protected:
    bool runOnce() override;
    virtual bool fetchVoltageBattery() = 0;
    virtual bool fetchCurrentBattery() {
        return true;
    }
    virtual bool fetchVoltageSolar() {
        return true;
    }
    virtual bool fetchCurrentSolar() {
        return true;
    }

    int16_t vs = 0, is = 0, vb = 0, ib = 0;
};

#endif //CUBECELL_MONITORING_ENERGY_H
