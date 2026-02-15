#include "PulseEngine.h"

#include <driver/ledc.h>
#include <driver/pulse_cnt.h>
#include <esp_attr.h>
#include <esp_log.h>

namespace {
const char *TAG = "PulseEngine";

constexpr ledc_mode_t kPulseLedsMode = LEDC_HIGH_SPEED_MODE;
constexpr ledc_timer_t kPulseLedsTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kPulseLedsChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kPulseLedsResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kPulseLedsDuty = 512;
constexpr uint32_t kPulseLedsClkSrc = LEDC_AUTO_CLK;

constexpr int kPulseHighLimit = 100;
constexpr int kPulseLowLimit = -100;
constexpr int kPulseWatchPoint = 1;

gpio_num_t gStepPin = GPIO_NUM_NC;
pcnt_unit_handle_t gPulseCountUnit = nullptr;
pcnt_channel_handle_t gPulseCountChannel = nullptr;
bool gPwmInitialized = false;

volatile bool gRunning = false;
volatile uint32_t gTargetPulses = 0;
volatile uint32_t gPulsesCompleted = 0;
volatile uint32_t gSpeedSwitchPulse = 0;
volatile bool gEndSpeedPending = false;

uint32_t gStartSpeedHz = 0;
uint32_t gEndSpeedHz = 0;

PulseEngine::CompletionCallback gCompletionCallback = nullptr;
void *gCompletionCtx = nullptr;

uint32_t abs_speed_hz(uint32_t speedHz) {
    return speedHz;
}

esp_err_t set_pulse_speed_hz(uint32_t speedHz) {
    if (speedHz == 0) return ESP_ERR_INVALID_ARG;

    const uint32_t runSpeedHz = abs_speed_hz(speedHz);
    if (!gPwmInitialized) {
        ledc_timer_config_t timerConfig = {};
        timerConfig.speed_mode = kPulseLedsMode;
        timerConfig.timer_num = kPulseLedsTimer;
        timerConfig.duty_resolution = kPulseLedsResolution;
        timerConfig.freq_hz = runSpeedHz;
        timerConfig.clk_cfg = static_cast<ledc_clk_cfg_t>(kPulseLedsClkSrc);
        esp_err_t timerErr = ledc_timer_config(&timerConfig);
        if (timerErr != ESP_OK) return timerErr;

        ledc_channel_config_t channelConfig = {};
        channelConfig.gpio_num = gStepPin;
        channelConfig.speed_mode = kPulseLedsMode;
        channelConfig.channel = kPulseLedsChannel;
        channelConfig.intr_type = LEDC_INTR_DISABLE;
        channelConfig.timer_sel = kPulseLedsTimer;
        channelConfig.duty = kPulseLedsDuty;
        channelConfig.hpoint = 0;
        channelConfig.flags.output_invert = 0;
        esp_err_t channelErr = ledc_channel_config(&channelConfig);
        if (channelErr != ESP_OK) return channelErr;

        gPwmInitialized = true;
        return ESP_OK;
    }

    uint32_t setFreqResult = ledc_set_freq(kPulseLedsMode, kPulseLedsTimer, runSpeedHz);
    if (setFreqResult == 0) return ESP_FAIL;
    esp_err_t dutyErr = ledc_set_duty(kPulseLedsMode, kPulseLedsChannel, kPulseLedsDuty);
    if (dutyErr != ESP_OK) return dutyErr;
    return ledc_update_duty(kPulseLedsMode, kPulseLedsChannel);
}

void stop_pwm() {
    if (gPwmInitialized) {
        ledc_stop(kPulseLedsMode, kPulseLedsChannel, 0);
        gPwmInitialized = false;
    }
}

bool IRAM_ATTR pulse_watch_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    (void)unit;
    (void)edata;
    (void)user_ctx;

    if (!gRunning) return false;

    gPulsesCompleted = gPulsesCompleted + 1;
    pcnt_unit_clear_count(gPulseCountUnit);

    if (gSpeedSwitchPulse > 0 && gPulsesCompleted >= gSpeedSwitchPulse) {
        gEndSpeedPending = true;
        gSpeedSwitchPulse = 0;
    }

    if (gPulsesCompleted < gTargetPulses) return false;

    gRunning = false;
    stop_pwm();
    if (gCompletionCallback != nullptr) {
        gCompletionCallback(gPulsesCompleted, gCompletionCtx);
    }
    return false;
}
} // namespace

bool PulseEngine::isInitialized = false;

esp_err_t PulseEngine::init(gpio_num_t stepPin)
{
    if (isInitialized) return ESP_OK;

    gStepPin = stepPin;

    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = kPulseLowLimit;
    unitConfig.high_limit = kPulseHighLimit;
    unitConfig.intr_priority = 0;
    unitConfig.flags.accum_count = 0;
    esp_err_t err = pcnt_new_unit(&unitConfig, &gPulseCountUnit);
    if (err != ESP_OK) return err;

    pcnt_chan_config_t channelConfig = {};
    channelConfig.edge_gpio_num = gStepPin;
    channelConfig.level_gpio_num = -1;
    channelConfig.flags.invert_edge_input = 0;
    channelConfig.flags.invert_level_input = 0;
    channelConfig.flags.virt_edge_io_level = 0;
    channelConfig.flags.virt_level_io_level = 0;
    channelConfig.flags.io_loop_back = 0;
    err = pcnt_new_channel(gPulseCountUnit, &channelConfig, &gPulseCountChannel);
    if (err != ESP_OK) return err;

    err = pcnt_channel_set_edge_action(
        gPulseCountChannel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (err != ESP_OK) return err;

    err = pcnt_channel_set_level_action(
        gPulseCountChannel,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) return err;

    pcnt_event_callbacks_t callbacks = {};
    callbacks.on_reach = pulse_watch_handler;
    err = pcnt_unit_register_event_callbacks(gPulseCountUnit, &callbacks, nullptr);
    if (err != ESP_OK) return err;

    err = pcnt_unit_add_watch_point(gPulseCountUnit, kPulseWatchPoint);
    if (err != ESP_OK) return err;

    err = pcnt_unit_set_glitch_filter(gPulseCountUnit, nullptr);
    if (err != ESP_OK) return err;

    err = pcnt_unit_enable(gPulseCountUnit);
    if (err != ESP_OK) return err;

    err = pcnt_unit_clear_count(gPulseCountUnit);
    if (err != ESP_OK) return err;

    err = pcnt_unit_start(gPulseCountUnit);
    if (err != ESP_OK) return err;

    isInitialized = true;
    ESP_LOGI(TAG, "Pulse engine initialized: step_pin=%u", static_cast<unsigned>(stepPin));
    return ESP_OK;
}

void PulseEngine::registerCompletionCallback(CompletionCallback callback, void *userCtx)
{
    gCompletionCallback = callback;
    gCompletionCtx = userCtx;
}

esp_err_t PulseEngine::startPulses(const StartConfig &config)
{
    if (!isInitialized) return ESP_ERR_INVALID_STATE;
    if (config.pulseCount == 0) return ESP_ERR_INVALID_ARG;

    const uint32_t startHz = abs_speed_hz(config.startSpeedHz);
    const uint32_t endHz = abs_speed_hz(config.endSpeedHz);
    if (startHz == 0 && endHz == 0) return ESP_ERR_INVALID_ARG;

    gStartSpeedHz = startHz == 0 ? endHz : startHz;
    gEndSpeedHz = endHz == 0 ? gStartSpeedHz : endHz;

    gTargetPulses = config.pulseCount;
    gPulsesCompleted = 0;
    gSpeedSwitchPulse = gStartSpeedHz == gEndSpeedHz ? 0 : (config.pulseCount / 2);
    gEndSpeedPending = false;
    gRunning = true;

    esp_err_t err = set_pulse_speed_hz(gStartSpeedHz);
    if (err != ESP_OK) {
        gRunning = false;
        return err;
    }

    err = pcnt_unit_clear_count(gPulseCountUnit);
    if (err != ESP_OK) {
        gRunning = false;
        stop_pwm();
        return err;
    }

    return ESP_OK;
}

uint32_t PulseEngine::stop()
{
    if (!isInitialized) return 0;
    if (!gRunning) return 0;

    gRunning = false;
    gSpeedSwitchPulse = 0;
    gEndSpeedPending = false;
    stop_pwm();
    return gPulsesCompleted;
}

void PulseEngine::service()
{
    if (!isInitialized) return;
    if (!gRunning) return;
    if (!gEndSpeedPending) return;

    gEndSpeedPending = false;
    esp_err_t err = set_pulse_speed_hz(gEndSpeedHz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "apply end speed failed: hz=%lu err=%s", static_cast<unsigned long>(gEndSpeedHz), esp_err_to_name(err));
    }
}

bool PulseEngine::isRunning()
{
    return gRunning;
}
