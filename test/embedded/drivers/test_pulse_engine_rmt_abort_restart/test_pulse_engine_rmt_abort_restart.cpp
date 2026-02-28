#include <unity.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
constexpr gpio_num_t kStepPin = GPIO_NUM_16;
constexpr uint32_t kCycles = 80;
constexpr uint32_t kRetryBudget = 25;
constexpr uint64_t kRetryDelayMs = 1;

bool start_with_retry(const PulseEngine::StartConfig &config, uint32_t maxRetries)
{
    for (uint32_t attempt = 0; attempt < maxRetries; ++attempt)
    {
        const esp_err_t err = PulseEngine::startPulses(config);
        if (err == ESP_OK) return true;
        if (err != ESP_ERR_INVALID_STATE) return false;
        PulseEngine::service();
        vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
    }
    return false;
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i)
    {
        printf("test_pulse_engine_rmt_abort_restart bootstrap %d/8\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
} // namespace

void setUp(void)
{
}

void tearDown(void)
{
}

void test_abort_restart_does_not_starve_new_starts(void)
{
#if MYR_PULSE_ENGINE_TYPE != MYR_PULSE_ENGINE_TYPE_RMT
    TEST_IGNORE_MESSAGE("requires MYR_PULSE_ENGINE_TYPE_RMT build");
#else
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::stop();

    PulseEngine::StartConfig longRun = {};
    longRun.pulseCount = 5000;
    longRun.startSpeedHz = 12000;
    longRun.endSpeedHz = 12000;

    PulseEngine::StartConfig shortRun = {};
    shortRun.pulseCount = 32;
    shortRun.startSpeedHz = 16000;
    shortRun.endSpeedHz = 16000;

    for (uint32_t i = 0; i < kCycles; ++i)
    {
        TEST_ASSERT_TRUE_MESSAGE(start_with_retry(longRun, kRetryBudget), "failed to start long run");
        vTaskDelay(pdMS_TO_TICKS(1));
        (void)PulseEngine::stop();

        const uint64_t restartStartUs = esp_timer_get_time();
        TEST_ASSERT_TRUE_MESSAGE(start_with_retry(shortRun, kRetryBudget), "failed to restart short run");
        const uint64_t restartElapsedUs = esp_timer_get_time() - restartStartUs;
        TEST_ASSERT_TRUE_MESSAGE(restartElapsedUs < 200000ULL, "restart acceptance exceeded bound");
        (void)PulseEngine::stop();
    }
#endif
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_abort_restart_does_not_starve_new_starts);
    UNITY_END();
}
