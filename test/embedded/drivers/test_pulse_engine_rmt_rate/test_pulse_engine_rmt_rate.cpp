#include <unity.h>

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
constexpr gpio_num_t kStepPin = GPIO_NUM_16;
constexpr uint32_t kPulseCount = 2000;
constexpr uint32_t kSpeedHz = 100;
constexpr uint64_t kExpectedRunUs = (kPulseCount * 1000000ULL + (kSpeedHz - 1ULL)) / kSpeedHz;
constexpr uint64_t kTimeoutUs = kExpectedRunUs + 5000000ULL;

volatile bool gRunDone = false;
volatile uint32_t gDonePulses = 0;

void on_complete(uint32_t pulsesCompleted, void *ctx)
{
    (void)ctx;
    gDonePulses = pulsesCompleted;
    gRunDone = true;
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i)
    {
        printf("test_pulse_engine_rmt_rate bootstrap %d/8\n", i + 1);
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

void test_rmt_engine_run_matches_requested_pulse_count(void)
{
#if MYR_PULSE_ENGINE_TYPE != MYR_PULSE_ENGINE_TYPE_RMT
    TEST_IGNORE_MESSAGE("requires MYR_PULSE_ENGINE_TYPE_RMT build");
#else
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::registerCompletionCallback(on_complete, nullptr);
    PulseEngine::stop();

    pcnt_unit_handle_t unit = nullptr;
    pcnt_channel_handle_t channel = nullptr;

    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = -1;
    unitConfig.high_limit = 32767;
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_new_unit(&unitConfig, &unit));

    pcnt_chan_config_t channelConfig = {};
    channelConfig.edge_gpio_num = static_cast<int>(kStepPin);
    channelConfig.level_gpio_num = -1;
    channelConfig.flags.io_loop_back = true;
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_new_channel(unit, &channelConfig, &channel));
    TEST_ASSERT_EQUAL(
        ESP_OK,
        pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    TEST_ASSERT_EQUAL(
        ESP_OK,
        pcnt_channel_set_level_action(channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_enable(unit));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_clear_count(unit));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_start(unit));

    PulseEngine::StartConfig config = {};
    config.pulseCount = kPulseCount;
    config.startSpeedHz = kSpeedHz;
    config.endSpeedHz = kSpeedHz;

    gRunDone = false;
    gDonePulses = 0;
    const uint64_t startUs = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::startPulses(config));

    while (!gRunDone && (esp_timer_get_time() - startUs) < kTimeoutUs)
    {
        PulseEngine::service();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    PulseEngine::service();

    TEST_ASSERT_TRUE_MESSAGE(gRunDone, "pulse run timed out");
    TEST_ASSERT_EQUAL_UINT32(kPulseCount, gDonePulses);

    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_stop(unit));
    int countedPulses = 0;
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_get_count(unit, &countedPulses));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_disable(unit));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_del_channel(channel));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_del_unit(unit));

    TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kPulseCount), countedPulses);
#endif
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_rmt_engine_run_matches_requested_pulse_count);
    UNITY_END();
}
