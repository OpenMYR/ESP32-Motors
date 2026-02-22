#include "PulseEngine.h"

#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>

namespace {
enum Phase
{
    RISE,
    FALL
};

constexpr uint32_t kDefaultTimerHz = 1000000;
constexpr uint32_t kPulseHighUs = 2;
const char *TAG = "PulseEngine";

gptimer_handle_t gTimer = nullptr;
gpio_num_t gStepPin = GPIO_NUM_NC;

volatile uint32_t gPulsesDone = 0;
uint32_t gPulsesTotal = 0;

uint32_t gTimerHz = kDefaultTimerHz;
uint32_t gHighTicks = 1;
uint32_t gPeriodTicks = 1;

uint64_t gNextRise = 0;
volatile Phase gPhase = RISE;

PulseEngine::StartConfig gMove = {};

PulseEngine::CompletionCallback gCompletionCallback = nullptr;
void *gCompletionCtx = nullptr;

volatile bool gRunning = false;
volatile bool gCompletionPending = false;
volatile uint32_t gCompletionPulses = 0;

static inline uint32_t period_ticks_for_step(uint32_t step)
{
    if (gMove.startSpeedHz == gMove.endSpeedHz) return gTimerHz / gMove.startSpeedHz;
    if (gPulsesTotal <= 1) return gTimerHz / gMove.endSpeedHz;

    const uint32_t span = gPulsesTotal - 1;
    const int64_t hz = static_cast<int64_t>(gMove.startSpeedHz) +
                       ((static_cast<int64_t>(gMove.endSpeedHz) - static_cast<int64_t>(gMove.startSpeedHz)) *
                        static_cast<int64_t>(step > span ? span : step)) /
                           static_cast<int64_t>(span);

    return gTimerHz / static_cast<uint32_t>(hz);
}

bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *eventData, void *arg)
{
    (void)arg;
    const uint64_t now = eventData->count_value;

    if (!gRunning)
        return false;

    if (gPulsesDone >= gPulsesTotal)
    {
        gpio_set_level(gStepPin, 0);
        gRunning = false;
        gCompletionPending = true;
        gCompletionPulses = gPulsesDone;
        return false;
    }

    if (gPhase == RISE)
    {
        gpio_set_level(gStepPin, 1);
        gPhase = FALL;

        gptimer_alarm_config_t alarm = {};
        alarm.alarm_count = now + gHighTicks;
        gptimer_set_alarm_action(timer, &alarm);
        return false;
    }

    gpio_set_level(gStepPin, 0);
    gPhase = RISE;

    gPulsesDone = gPulsesDone + 1;
    if (gPulsesDone >= gPulsesTotal)
    {
        gRunning = false;
        gCompletionPending = true;
        gCompletionPulses = gPulsesDone;
        return false;
    }

    gPeriodTicks = period_ticks_for_step(gPulsesDone);
    while (gNextRise <= now)
        gNextRise += gPeriodTicks;

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = gNextRise;
    gptimer_set_alarm_action(timer, &alarm);

    gNextRise += gPeriodTicks;
    return false;
}
} // namespace

bool PulseEngine::isInitialized = false;

esp_err_t PulseEngine::init(gpio_num_t stepPin)
{
    if (isInitialized)
        return ESP_OK;

    gStepPin = stepPin;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(gStepPin);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK)
    {
        return err;
    }

    gpio_set_level(gStepPin, 0);

    gptimer_config_t timerConfig = {};
    timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig.direction = GPTIMER_COUNT_UP;
    timerConfig.resolution_hz = gTimerHz;

    err = gptimer_new_timer(&timerConfig, &gTimer);
    if (err != ESP_OK)
    {
        return err;
    }

    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = on_alarm;

    err = gptimer_register_event_callbacks(gTimer, &callbacks, nullptr);
    if (err != ESP_OK)
    {
        return err;
    }

    err = gptimer_enable(gTimer);
    if (err != ESP_OK)
    {
        return err;
    }

    gHighTicks = (gTimerHz * kPulseHighUs) / 1000000;
    if (gHighTicks == 0)
        gHighTicks = 1;

    isInitialized = true;
    return ESP_OK;
}

void PulseEngine::registerCompletionCallback(CompletionCallback callback, void *userCtx)
{
    gCompletionCallback = callback;
    gCompletionCtx = userCtx;
}

esp_err_t PulseEngine::startPulses(const StartConfig &config)
{
    // Normalize timer/output state before arming a new pulse run.
    gRunning = false;
    gCompletionPending = false;
    esp_err_t stopErr = gptimer_stop(gTimer);
    if (stopErr != ESP_OK && stopErr != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "startPulses pre-stop failed: err=%s", esp_err_to_name(stopErr));
    gpio_set_level(gStepPin, 0);

    gMove = config;
    gPulsesDone = 0;
    gPulsesTotal = config.pulseCount;
    gPhase = RISE;
    gCompletionPending = false;

    gPeriodTicks = period_ticks_for_step(0);

    uint64_t now = 0;
    gptimer_get_raw_count(gTimer, &now);

    gNextRise = now + gPeriodTicks;

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = gNextRise;

    esp_err_t err = gptimer_set_alarm_action(gTimer, &alarm);
    if (err != ESP_OK)
    {
        return err;
    }

    gNextRise += gPeriodTicks;
    gRunning = true;

    err = gptimer_start(gTimer);
    if (err != ESP_OK)
    {
        gRunning = false;
        return err;
    }

    return ESP_OK;
}

uint32_t PulseEngine::stop()
{
    const uint32_t pulses = gPulsesDone;

    gRunning = false;
    gCompletionPending = false;
    gptimer_stop(gTimer);
    gpio_set_level(gStepPin, 0);

    return pulses;
}

void PulseEngine::service()
{
    if (!gCompletionPending)
        return;

    gCompletionPending = false;
    gptimer_stop(gTimer);
    gpio_set_level(gStepPin, 0);

    if (gCompletionCallback != nullptr)
        gCompletionCallback(gCompletionPulses, gCompletionCtx);
}

bool PulseEngine::isRunning()
{
    return gRunning;
}
