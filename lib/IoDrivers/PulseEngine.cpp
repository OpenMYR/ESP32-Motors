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

// #region FIXME(PULSE-GPTIMER-DEBUG): Temporary diagnostics for GPTimer init/start invalid-state failures.
esp_err_t gLastInitErr = ESP_ERR_INVALID_STATE;
bool gInitGpioConfigured = false;
bool gInitTimerCreated = false;
bool gInitCallbacksRegistered = false;
bool gInitTimerEnabled = false;
// #endregion FIXME(PULSE-GPTIMER-DEBUG)

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

static inline uint32_t hz_to_ticks(uint32_t hz)
{
    if (hz == 0)
        hz = 1;

    uint32_t ticks = gTimerHz / hz;
    return ticks == 0 ? 1 : ticks;
}

static inline uint32_t normalized_start_hz()
{
    if (gMove.startSpeedHz != 0)
        return gMove.startSpeedHz;
    if (gMove.endSpeedHz != 0)
        return gMove.endSpeedHz;
    return 1;
}

static inline uint32_t normalized_end_hz()
{
    if (gMove.endSpeedHz != 0)
        return gMove.endSpeedHz;
    return normalized_start_hz();
}

static uint32_t current_hz(uint32_t step)
{
    const uint32_t startHz = normalized_start_hz();
    const uint32_t endHz = normalized_end_hz();

    if (startHz == endHz)
        return startHz;

    if (gPulsesTotal <= 1)
        return endHz;

    const uint32_t span = gPulsesTotal - 1;
    const uint32_t clampedStep = step > span ? span : step;

    const int64_t delta = static_cast<int64_t>(endHz) - static_cast<int64_t>(startHz);
    const int64_t hz = static_cast<int64_t>(startHz) + (delta * static_cast<int64_t>(clampedStep)) / static_cast<int64_t>(span);

    if (hz <= 0)
        return 1;

    return static_cast<uint32_t>(hz);
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

    gPeriodTicks = hz_to_ticks(current_hz(gPulsesDone));
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

    // #region FIXME(PULSE-GPTIMER-DEBUG): Capture init stage progress while root-causing startup failures.
    gLastInitErr = ESP_OK;
    gInitGpioConfigured = false;
    gInitTimerCreated = false;
    gInitCallbacksRegistered = false;
    gInitTimerEnabled = false;
    // #endregion FIXME(PULSE-GPTIMER-DEBUG)

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
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when init failure path is resolved.
        gLastInitErr = err;
        ESP_LOGW(TAG, "init failed at gpio_config: pin=%d err=%s", static_cast<int>(gStepPin), esp_err_to_name(err));
        return err;
    }
    gInitGpioConfigured = true;

    gpio_set_level(gStepPin, 0);

    gptimer_config_t timerConfig = {};
    // FIXME(PULSE-GPTIMER-DEBUG): Keep explicit clock-source selection while validating ESP_ERR_INVALID_ARG root cause on ESP32.
    timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig.direction = GPTIMER_COUNT_UP;
    timerConfig.resolution_hz = gTimerHz;

    err = gptimer_new_timer(&timerConfig, &gTimer);
    if (err != ESP_OK)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when init failure path is resolved.
        gLastInitErr = err;
        ESP_LOGW(TAG, "init failed at gptimer_new_timer: err=%s", esp_err_to_name(err));
        return err;
    }
    gInitTimerCreated = true;

    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = on_alarm;

    err = gptimer_register_event_callbacks(gTimer, &callbacks, nullptr);
    if (err != ESP_OK)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when init failure path is resolved.
        gLastInitErr = err;
        ESP_LOGW(TAG, "init failed at gptimer_register_event_callbacks: err=%s", esp_err_to_name(err));
        return err;
    }
    gInitCallbacksRegistered = true;

    err = gptimer_enable(gTimer);
    if (err != ESP_OK)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when init failure path is resolved.
        gLastInitErr = err;
        ESP_LOGW(TAG, "init failed at gptimer_enable: err=%s", esp_err_to_name(err));
        return err;
    }
    gInitTimerEnabled = true;

    gHighTicks = (gTimerHz * kPulseHighUs) / 1000000;
    if (gHighTicks == 0)
        gHighTicks = 1;

    gLastInitErr = ESP_OK;
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
    if (!isInitialized)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when invalid-state startup root cause is fixed.
        ESP_LOGW(
            TAG,
            "startPulses before init: lastInitErr=%s gpio=%d timer=%d callbacks=%d enabled=%d pulses=%u start=%u end=%u",
            esp_err_to_name(gLastInitErr),
            static_cast<int>(gInitGpioConfigured),
            static_cast<int>(gInitTimerCreated),
            static_cast<int>(gInitCallbacksRegistered),
            static_cast<int>(gInitTimerEnabled),
            static_cast<unsigned>(config.pulseCount),
            static_cast<unsigned>(config.startSpeedHz),
            static_cast<unsigned>(config.endSpeedHz));
        return ESP_ERR_INVALID_STATE;
    }

    if (config.pulseCount == 0)
        return ESP_ERR_INVALID_ARG;

    // #region FIXME(PULSE-GPTIMER-RESET): Keep explicit pre-stop until GPTimer state transitions are fully validated.
    gRunning = false;
    gCompletionPending = false;
    esp_err_t stopErr = gptimer_stop(gTimer);
    if (stopErr != ESP_OK && stopErr != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "startPulses pre-stop failed: err=%s", esp_err_to_name(stopErr));
    gpio_set_level(gStepPin, 0);
    // #endregion FIXME(PULSE-GPTIMER-RESET)

    gMove = config;
    gPulsesDone = 0;
    gPulsesTotal = config.pulseCount;
    gPhase = RISE;
    gCompletionPending = false;

    gPeriodTicks = hz_to_ticks(current_hz(0));

    uint64_t now = 0;
    gptimer_get_raw_count(gTimer, &now);

    gNextRise = now + gPeriodTicks;

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = gNextRise;

    esp_err_t err = gptimer_set_alarm_action(gTimer, &alarm);
    if (err != ESP_OK)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when invalid-state startup root cause is fixed.
        ESP_LOGW(
            TAG,
            "startPulses alarm setup failed: pulses=%u start=%u end=%u err=%s",
            static_cast<unsigned>(config.pulseCount),
            static_cast<unsigned>(config.startSpeedHz),
            static_cast<unsigned>(config.endSpeedHz),
            esp_err_to_name(err));
        return err;
    }

    gNextRise += gPeriodTicks;
    gRunning = true;

    err = gptimer_start(gTimer);
    if (err != ESP_OK)
    {
        // FIXME(PULSE-GPTIMER-DEBUG): Remove when invalid-state startup root cause is fixed.
        ESP_LOGW(
            TAG,
            "startPulses gptimer_start failed: pulses=%u start=%u end=%u err=%s",
            static_cast<unsigned>(config.pulseCount),
            static_cast<unsigned>(config.startSpeedHz),
            static_cast<unsigned>(config.endSpeedHz),
            esp_err_to_name(err));
        gRunning = false;
        return err;
    }

    return ESP_OK;
}

uint32_t PulseEngine::stop()
{
    if (!isInitialized)
        return 0;

    const uint32_t pulses = gPulsesDone;

    gRunning = false;
    gCompletionPending = false;
    gptimer_stop(gTimer);
    gpio_set_level(gStepPin, 0);

    return pulses;
}

void PulseEngine::service()
{
    if (!isInitialized)
        return;

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
