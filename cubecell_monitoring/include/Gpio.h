#ifndef CUBECELL_MONITORING_GPIO_H
#define CUBECELL_MONITORING_GPIO_H

#include <cstdint>
#include "Config.h"

class System;

class Gpio {
public:
    explicit Gpio(System *system);

    void printJson() const;

    void setRelay1(bool enabled);
    void setRelay2(bool enabled);

    uint16_t getLdr() const;

    inline bool isRelay1Enabled() const {
        return relay1;
    }

    inline bool isRelay2Enabled() const {
        return relay2;
    }
private:
    System *system;
    bool initialized = false;

    bool relay1 = RELAY_1_INITIAL_STATE;
    bool relay2 = RELAY_2_INITIAL_STATE;

    void setState(uint8_t pin, bool enabled, const char* name, bool &status, bool inverted = false);
    bool getState(uint8_t pin, const char* name) const;
    uint16_t getAdcState(uint8_t pin, const char* name) const;
};

#endif //CUBECELL_MONITORING_GPIO_H
