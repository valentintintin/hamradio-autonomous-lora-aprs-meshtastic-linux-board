#include <hardware/pll.h>
#include <hardware/vreg.h>
#include "utils.h"
#include "System.h"

char bufferText[BUFFER_LENGTH]{};
uint8_t buffer[BUFFER_LENGTH]{};

System systemControl;
auto gpioSlowClock = GpioPin(SLOW_CLOCK_PIN, INPUT);

void setSlowClock();

void setup() {
    if (gpioSlowClock.getState()) {
        systemControl.setSlowClock();
        Serial2.begin(115200);
        Log.begin(LOG_LEVEL_INFO, &Serial2, true, true);
        Log.infoln(F("[MAIN] Use slow clock"));
        setSlowClock();
    } else {
        Serial.begin(115200);
        Log.begin(LOG_LEVEL_TRACE, &Serial, true, true);
        delay(2500); // Wait for serial debug
        Log.infoln(F("[MAIN] Debug mode"));
    }

    Serial1.begin(115200);

    systemControl.begin();
}

void loop() {
    systemControl.loop();
}


void delayWdt(const uint32_t milliseconds) {
    if (systemControl.settings.useInternalWatchdog) {
        const uint64_t startTime = millis();
        uint64_t elapsedTime = 0;

        while (elapsedTime < milliseconds) {
            if (milliseconds - elapsedTime >= 1000) {
                delay(1000);
                rp2040.wdt_reset();
            } else {
                delay(milliseconds - elapsedTime);
            }
            elapsedTime = millis() - startTime;
        }
    } else {
        delay(milliseconds);
    }
}

void ledBlink(const uint8_t howMany, const uint16_t milliseconds) {
    for (uint8_t i = 0; i < howMany; i++) {
        digitalWrite(PIN_LED, HIGH);
        delayWdt(milliseconds);
        digitalWrite(PIN_LED, LOW);
        delayWdt(milliseconds);
    }
}

void setSlowClock() {
    /* Set the system frequency to 18 MHz. */
    set_sys_clock_khz(18 * KHZ, false);
    /* The previous line automatically detached clk_peri from clk_sys, and
       attached it to pll_usb. We need to attach clk_peri back to system PLL to keep SPI
       working at this low speed.
       For details see https://github.com/jgromes/RadioLib/discussions/938
    */
    clock_configure(clk_peri,
                    0,                                                // No glitchless mux
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
                    18 * MHZ,                                         // Input frequency
                    18 * MHZ                                          // Output (must be same as no divider)
    );
    /* Run also ADC on lower clk_sys. */
    clock_configure(clk_adc, 0, CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 18 * MHZ, 18 * MHZ);
    /* Run RTC from XOSC since USB clock is off */
    clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12 * MHZ, 47 * KHZ);
    vreg_set_voltage(VREG_VOLTAGE_0_90);
    /* Turn off USB PLL */
    pll_deinit(pll_usb);
}