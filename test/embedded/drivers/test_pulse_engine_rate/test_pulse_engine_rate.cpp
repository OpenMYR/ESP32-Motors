#include <unity.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PulseEngine.h"

namespace {
const char *TAG = "TestPulseRate";
constexpr gpio_num_t kStepPin = GPIO_NUM_16;
constexpr uint32_t kPulseCount = 3000;
constexpr uint32_t kTargetHz = 1000;
constexpr uint64_t kTimeoutUs = 6000000ULL;
constexpr uint32_t kCountTolerance = 2;
constexpr double kRateTolerancePct = 5.0;

volatile bool gRunDone = false;
volatile uint32_t gCallbackPulses = 0;

void on_pulse_complete(uint32_t pulsesCompleted, uint32_t runToken, void *userCtx)
{
    (void)runToken;
    (void)userCtx;
    gCallbackPulses = pulsesCompleted;
    gRunDone = true;
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 8; ++i) {
        printf("test_pulse_engine_rate bootstrap %d/8\n", i + 1);
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

void test_pulse_engine_self_check_reports_measured_rate(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::init(kStepPin));
    PulseEngine::registerCompletionCallback(on_pulse_complete, nullptr);
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

    gRunDone = false;
    gCallbackPulses = 0;

    PulseEngine::StartConfig config = {};
    config.pulseCount = kPulseCount;
    config.startSpeedHz = kTargetHz;
    config.endSpeedHz = kTargetHz;

    const uint64_t startUs = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, PulseEngine::startPulses(config));

    const uint64_t timeoutAtUs = startUs + kTimeoutUs;
    while (!gRunDone && esp_timer_get_time() < timeoutAtUs) {
        PulseEngine::service();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    PulseEngine::service();

    const uint64_t endUs = esp_timer_get_time();
    TEST_ASSERT_TRUE_MESSAGE(gRunDone, "pulse run timed out");
    TEST_ASSERT_EQUAL_UINT32(kPulseCount, gCallbackPulses);

    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_stop(unit));

    int countedPulses = 0;
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_get_count(unit, &countedPulses));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_unit_disable(unit));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_del_channel(channel));
    TEST_ASSERT_EQUAL(ESP_OK, pcnt_del_unit(unit));

    const uint64_t elapsedUs = endUs - startUs;
    TEST_ASSERT_TRUE_MESSAGE(elapsedUs > 0, "elapsed time must be > 0");

    const double measuredHz = (static_cast<double>(countedPulses) * 1000000.0) / static_cast<double>(elapsedUs);
    const double errorPct = ((measuredHz - static_cast<double>(kTargetHz)) * 100.0) / static_cast<double>(kTargetHz);
    const double absErrorPct = errorPct >= 0.0 ? errorPct : -errorPct;

    ESP_LOGI(
        TAG,
        "Pulse self-check: target_hz=%u callback_pulses=%u counted_pulses=%d elapsed_us=%llu measured_hz=%.6f error_pct=%.3f",
        static_cast<unsigned>(kTargetHz),
        static_cast<unsigned>(gCallbackPulses),
        countedPulses,
        static_cast<unsigned long long>(elapsedUs),
        measuredHz,
        errorPct);

    TEST_ASSERT_INT_WITHIN_MESSAGE(
        static_cast<int>(kCountTolerance),
        static_cast<int>(kPulseCount),
        countedPulses,
        "pulse counter value differs from requested pulse count");
    TEST_ASSERT_TRUE_MESSAGE(absErrorPct <= kRateTolerancePct, "measured pulse rate error exceeds tolerance");

    PulseEngine::stop();
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_pulse_engine_self_check_reports_measured_rate);
    UNITY_END();
}
