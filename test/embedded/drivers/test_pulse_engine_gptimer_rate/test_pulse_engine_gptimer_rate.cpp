#include <unity.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
constexpr gpio_num_t kStepPin = GPIO_NUM_16;
const char *TAG = "test_gptimer_rate";
constexpr uint32_t kPulseCount = 1200;
constexpr uint64_t kTimeoutPadUs = 5000000ULL;
constexpr double kRateTolerancePct = 5.0;
constexpr uint64_t kCompletionSlackUs = 3000ULL;

volatile bool gRunDone = false;
volatile uint32_t gDonePulses = 0;
volatile uint64_t gDoneAtUs = 0;

void on_complete(uint32_t pulsesCompleted, uint32_t runToken, void *ctx)
{
    (void)runToken;
    (void)ctx;
    gDonePulses = pulsesCompleted;
    gDoneAtUs = static_cast<uint64_t>(esp_timer_get_time());
    gRunDone = true;
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i)
    {
        printf("test_pulse_engine_gptimer_rate bootstrap %d/8\n", i + 1);
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

void run_known_rate_case(uint32_t targetHz)
{
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
    config.startSpeedHz = targetHz;
    config.endSpeedHz = targetHz;

    gRunDone = false;
    gDonePulses = 0;
    gDoneAtUs = 0;
    const uint64_t startUs = esp_timer_get_time();
    const uint64_t expectedRunUs = (kPulseCount * 1000000ULL + (targetHz - 1ULL)) / targetHz;
    const uint64_t timeoutUs = expectedRunUs + kTimeoutPadUs;
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::startPulses(config));

    while (!gRunDone && (esp_timer_get_time() - startUs) < timeoutUs)
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

    const uint64_t endUs = gDoneAtUs > startUs ? gDoneAtUs : esp_timer_get_time();
    const uint64_t elapsedUs = endUs - startUs;
    TEST_ASSERT_TRUE_MESSAGE(elapsedUs > 0, "elapsed time must be > 0");
    TEST_ASSERT_TRUE_MESSAGE(
        elapsedUs <= (expectedRunUs + kCompletionSlackUs),
        "pulse train completion exceeded expected duration bound");

    const double measuredHz = (static_cast<double>(countedPulses) * 1000000.0) / static_cast<double>(elapsedUs);
    const double errorPct = ((measuredHz - static_cast<double>(targetHz)) * 100.0) / static_cast<double>(targetHz);
    const double absErrorPct = errorPct >= 0.0 ? errorPct : -errorPct;

    ESP_LOGI(
        TAG,
        "known-rate case: target_hz=%u measured_hz=%.3f error_pct=%.3f pulses=%d elapsed_us=%llu",
        static_cast<unsigned>(targetHz),
        measuredHz,
        errorPct,
        countedPulses,
        static_cast<unsigned long long>(elapsedUs));

    TEST_ASSERT_TRUE_MESSAGE(absErrorPct <= kRateTolerancePct, "measured pulse rate error exceeds tolerance");
}

void test_gptimer_engine_known_pulse_train_hz_round_trip(void)
{
#if MYR_PULSE_ENGINE_TYPE != MYR_PULSE_ENGINE_TYPE_GPTIMER
    TEST_IGNORE_MESSAGE("requires MYR_PULSE_ENGINE_TYPE_GPTIMER build");
#else
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::registerCompletionCallback(on_complete, nullptr);
    PulseEngine::stop();

    run_known_rate_case(110);
    run_known_rate_case(440);
    run_known_rate_case(1000);
#endif
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_gptimer_engine_known_pulse_train_hz_round_trip);
    UNITY_END();
}
