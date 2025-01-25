#ifndef RP2040_LORA_APRS_ENERGYADCTHREAD_H
#define RP2040_LORA_APRS_ENERGYADCTHREAD_H

#include "Threads/EnergyThread.h"
#include "GpioPin.h"

class EnergyAdcThread : public EnergyThread {
public:
    explicit EnergyAdcThread(System* system);
protected:
    bool fetchVoltageBattery() override;
private:
    GpioPin *adc;
};


#endif //RP2040_LORA_APRS_ENERGYADCTHREAD_H
