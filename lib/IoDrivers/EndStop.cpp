/**
 * @file Endstop.cpp
 * @brief Endstop ISR/debounce implementation for the stepper driver.
 */

#include "EndStop.h"

bool Endstop::s_isEndstopTrippedHigh = false;
uint32_t Endstop::s_debounceTimeMs = Endstop::kDefaultDebounceMs;

portMUX_TYPE Endstop::s_endstopAMux = portMUX_INITIALIZER_UNLOCKED;
bool Endstop::s_isEndstopAActiveNow = false;
volatile uint32_t Endstop::s_numberOfEndstopAIsr = 0;
bool Endstop::s_lastStateEndstopAIsr = false;
volatile uint32_t Endstop::s_debounceTimeoutEndstopAIsr = 0;

portMUX_TYPE Endstop::s_endstopBMux = portMUX_INITIALIZER_UNLOCKED;
bool Endstop::s_isEndstopBActiveNow = false;
volatile uint32_t Endstop::s_numberOfEndstopBIsr = 0;
volatile bool Endstop::s_lastStateEndstopBIsr = false;
volatile uint32_t Endstop::s_debounceTimeoutEndstopBIsr = 0;

void Endstop::configureGpioAndAttachIsr(bool &gpioIsrServiceInstalled)
{
    gpio_config_t ioAConfig = {};
    ioAConfig.pin_bit_mask = (1ULL << static_cast<uint32_t>(kPinA));
    ioAConfig.mode = GPIO_MODE_INPUT;
    ioAConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    ioAConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioAConfig.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&ioAConfig));

    gpio_config_t ioBConfig = {};
    ioBConfig.pin_bit_mask = (1ULL << static_cast<uint32_t>(kPinB));
    ioBConfig.mode = GPIO_MODE_INPUT;
    ioBConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    ioBConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioBConfig.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&ioBConfig));

    if (!gpioIsrServiceInstalled)
    {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
        {
            gpioIsrServiceInstalled = true;
        }
        else
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        }
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(kPinA, &Endstop::endstopAInterrupt, nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(kPinB, &Endstop::endstopBInterrupt, nullptr));
}

void Endstop::primeStateFromPins()
{
    endstopAInterrupt(nullptr);
    endstopBInterrupt(nullptr);
}

void IRAM_ATTR Endstop::endstopAInterrupt(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_endstopAMux);
    s_numberOfEndstopAIsr = s_numberOfEndstopAIsr + 1;
    s_lastStateEndstopAIsr = gpio_get_level(kPinA);
    s_debounceTimeoutEndstopAIsr = xTaskGetTickCountFromISR();
    portEXIT_CRITICAL_ISR(&s_endstopAMux);
}

void IRAM_ATTR Endstop::endstopBInterrupt(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_endstopBMux);
    s_numberOfEndstopBIsr = s_numberOfEndstopBIsr + 1;
    s_lastStateEndstopBIsr = gpio_get_level(kPinB);
    s_debounceTimeoutEndstopBIsr = xTaskGetTickCountFromISR();
    portEXIT_CRITICAL_ISR(&s_endstopBMux);
}

bool IRAM_ATTR Endstop::isTripped()
{
    uint32_t saveDebounceTimeout;
    bool saveLastState;
    uint32_t hasChanged;
    bool currentState = false;
    bool returnValue = false;

    portENTER_CRITICAL_ISR(&s_endstopAMux);
    hasChanged = s_numberOfEndstopAIsr;
    saveDebounceTimeout = s_debounceTimeoutEndstopAIsr;
    saveLastState = s_lastStateEndstopAIsr;
    portEXIT_CRITICAL_ISR(&s_endstopAMux);

    currentState = gpio_get_level(kPinA);
    if ((hasChanged != 0) &&
        (currentState == saveLastState) &&
        ((xTaskGetTickCount() - saveDebounceTimeout) > pdMS_TO_TICKS(s_debounceTimeMs)))
    {
        portENTER_CRITICAL_ISR(&s_endstopAMux);
        s_numberOfEndstopAIsr = 0;
        portEXIT_CRITICAL_ISR(&s_endstopAMux);

        s_isEndstopAActiveNow = (currentState == s_isEndstopTrippedHigh);
    }

    returnValue |= s_isEndstopAActiveNow;

    portENTER_CRITICAL_ISR(&s_endstopBMux);
    hasChanged = s_numberOfEndstopBIsr;
    saveDebounceTimeout = s_debounceTimeoutEndstopBIsr;
    saveLastState = s_lastStateEndstopBIsr;
    portEXIT_CRITICAL_ISR(&s_endstopBMux);

    currentState = gpio_get_level(kPinB);
    if ((hasChanged != 0) &&
        (currentState == saveLastState) &&
        ((xTaskGetTickCount() - saveDebounceTimeout) > pdMS_TO_TICKS(s_debounceTimeMs)))
    {
        portENTER_CRITICAL_ISR(&s_endstopBMux);
        s_numberOfEndstopBIsr = 0;
        portEXIT_CRITICAL_ISR(&s_endstopBMux);

        s_isEndstopBActiveNow = (currentState == s_isEndstopTrippedHigh);
    }

    returnValue |= s_isEndstopBActiveNow;
    return returnValue;
}

bool Endstop::getTrippedPinSetting()
{
    return s_isEndstopTrippedHigh;
}

esp_err_t Endstop::setTrippedPinSetting(uint8_t setting)
{
    if (setting > 1)
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_isEndstopTrippedHigh = static_cast<bool>(setting);
    return ESP_OK;
}
