#ifndef MYR_PULSEENGINE_H
#define MYR_PULSEENGINE_H

#include <stdint.h>

#include <driver/gpio.h>
#include <esp_err.h>

#define MYR_PULSE_ENGINE_TYPE_GPTIMER 1
#define MYR_PULSE_ENGINE_TYPE_LEDC 2
#define MYR_PULSE_ENGINE_TYPE_RMT 3

// Override via build flags, e.g. -D MYR_PULSE_ENGINE_TYPE=2.
#ifndef MYR_PULSE_ENGINE_TYPE
#define MYR_PULSE_ENGINE_TYPE MYR_PULSE_ENGINE_TYPE_GPTIMER
#endif

class PulseEngine
{
public:
    struct StartConfig
    {
        uint32_t pulseCount = 0;
        uint32_t startSpeedHz = 0;
        uint32_t endSpeedHz = 0;
        uint32_t runToken = 0;
    };

    using CompletionCallback = void (*)(uint32_t pulsesCompleted, uint32_t runToken, void *userCtx);

    static esp_err_t init(gpio_num_t stepPin);
    static void registerCompletionCallback(CompletionCallback callback, void *userCtx);
    static esp_err_t startPulses(const StartConfig &config);
    static uint32_t stop();
    static void service();
    static bool isRunning();

private:
    static bool isInitialized;
};

#endif // MYR_PULSEENGINE_H
