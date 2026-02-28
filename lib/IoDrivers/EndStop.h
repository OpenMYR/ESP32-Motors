#ifndef MYR_ENDSTOP_H
#define MYR_ENDSTOP_H

/**
 * @file Endstop.h
 * @brief Endstop ISR/debounce interface for the stepper driver.
 */

#include <stdint.h>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Endstop
{
public:
    static constexpr gpio_num_t kPinA = static_cast<gpio_num_t>(21);
    static constexpr gpio_num_t kPinB = static_cast<gpio_num_t>(22);
    static constexpr uint32_t kDefaultDebounceMs = 10; // TODO: NVS config

    static void configureGpioAndAttachIsr(bool &gpioIsrServiceInstalled);
    static void primeStateFromPins();
    static void IRAM_ATTR endstopAInterrupt(void *arg);
    static void IRAM_ATTR endstopBInterrupt(void *arg);
    static bool IRAM_ATTR isTripped();
    static bool getTrippedPinSetting();
    static esp_err_t setTrippedPinSetting(uint8_t setting);

private:
    static bool DRAM_ATTR s_isEndstopTrippedHigh;
    static uint32_t DRAM_ATTR s_debounceTimeMs;

    static DRAM_ATTR portMUX_TYPE s_endstopAMux;
    static bool DRAM_ATTR s_isEndstopAActiveNow;
    static volatile uint32_t DRAM_ATTR s_numberOfEndstopAIsr;
    static bool DRAM_ATTR s_lastStateEndstopAIsr;
    static volatile uint32_t DRAM_ATTR s_debounceTimeoutEndstopAIsr;

    static DRAM_ATTR portMUX_TYPE s_endstopBMux;
    static bool DRAM_ATTR s_isEndstopBActiveNow;
    static volatile uint32_t DRAM_ATTR s_numberOfEndstopBIsr;
    static volatile bool DRAM_ATTR s_lastStateEndstopBIsr;
    static volatile uint32_t DRAM_ATTR s_debounceTimeoutEndstopBIsr;
};

#endif // MYR_ENDSTOP_H
