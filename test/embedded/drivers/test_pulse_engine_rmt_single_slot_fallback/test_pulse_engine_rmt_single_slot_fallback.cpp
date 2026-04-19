#include <unity.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
constexpr gpio_num_t kStepPin = GPIO_NUM_16;
constexpr uint32_t kPulseCount = 200;
constexpr uint32_t kPulseHz = 2000;
constexpr uint64_t kTimeoutUs = 10000000ULL;
volatile bool gDone = false;
volatile uint32_t gDonePulses = 0;

void on_complete(uint32_t pulsesCompleted, uint32_t runToken, void *ctx)
{
    (void)runToken;
    (void)ctx;
    gDonePulses = pulsesCompleted;
    gDone = true;
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i)
    {
        printf("test_pulse_engine_rmt_single_slot_fallback bootstrap %d/8\n", i + 1);
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

void test_rmt_engine_completes_chunked_run(void)
{
#if MYR_PULSE_ENGINE_TYPE != MYR_PULSE_ENGINE_TYPE_RMT
    TEST_IGNORE_MESSAGE("requires MYR_PULSE_ENGINE_TYPE_RMT build");
#else
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::registerCompletionCallback(on_complete, nullptr);
    PulseEngine::stop();

    PulseEngine::StartConfig config = {};
    config.pulseCount = kPulseCount;
    config.startSpeedHz = kPulseHz;
    config.endSpeedHz = kPulseHz;

    gDone = false;
    gDonePulses = 0;
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::startPulses(config));

    const uint64_t startUs = esp_timer_get_time();
    while (!gDone && (esp_timer_get_time() - startUs) < kTimeoutUs)
    {
        PulseEngine::service();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    PulseEngine::service();

    if (!gDone && !PulseEngine::isRunning())
    {
        gDonePulses = PulseEngine::stop();
        gDone = true;
    }

    TEST_ASSERT_TRUE_MESSAGE(gDone, "chunked run timed out");
    TEST_ASSERT_TRUE_MESSAGE(gDonePulses <= kPulseCount, "callback pulse count exceeds request");

    PulseEngine::stop();
#endif
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_rmt_engine_completes_chunked_run);
    UNITY_END();
}
