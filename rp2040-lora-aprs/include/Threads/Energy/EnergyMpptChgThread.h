#ifndef RP2040_LORA_APRS_ENERGYMPPTCHGTHREAD_H
#define RP2040_LORA_APRS_ENERGYMPPTCHGTHREAD_H

#include "Threads/EnergyThread.h"

class EnergyMpptChgThread : public EnergyThread {
public:
    explicit EnergyMpptChgThread(System *system);
    void run() override;

    bool setPowerOnOff(uint16_t powerOnVoltage, uint16_t powerOffVoltage) const;

    inline bool isNight() const override {
        return _isNight;
    }

    inline uint16_t getStatus() const {
        return status;
    }
protected:
    bool init() override;
    bool fetchVoltageBattery() override;
    bool fetchCurrentBattery() override;
    bool fetchVoltageSolar() override;
    bool fetchCurrentSolar() override;

private:
    bool fetchOthersData();

    bool _isNight = false;
    uint16_t status = 0;
    mpptChg *charger;
};

#endif //RP2040_LORA_APRS_ENERGYMPPTCHGTHREAD_H
