#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
constexpr gpio_num_t kStepPin = GPIO_NUM_16;

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i)
    {
        printf("test_pulse_engine_rmt_signal_validation bootstrap %d/8\n", i + 1);
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

void test_rmt_init_accepts_forced_signal_test_env(void)
{
#if MYR_PULSE_ENGINE_TYPE != MYR_PULSE_ENGINE_TYPE_RMT
    TEST_IGNORE_MESSAGE("requires MYR_PULSE_ENGINE_TYPE_RMT build");
#else
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::StartConfig run = {};
    run.pulseCount = 16;
    run.startSpeedHz = 2000;
    run.endSpeedHz = 2000;
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::startPulses(run));
    vTaskDelay(pdMS_TO_TICKS(5));
    (void)PulseEngine::stop();
#endif
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_rmt_init_accepts_forced_signal_test_env);
    UNITY_END();
}
