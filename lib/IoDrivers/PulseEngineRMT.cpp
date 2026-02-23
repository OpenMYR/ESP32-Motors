#include "PulseEngine.h"

#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <driver/rmt_common.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

namespace {
const char *TAG = "PulseEngine";

constexpr uint32_t kRmtResolutionHz = 1000000;
constexpr uint32_t kPulseHighUs = 2;
constexpr uint16_t kSymbolDurationMax = 32767;
constexpr int kRmtIntrPriority = 1;
constexpr size_t kRmtMemBlockSymbols = 512;
constexpr size_t kRmtQueueDepth = 1;

constexpr int kPcntHighLimit = 30000;
constexpr int kPcntLowLimit = -1;

constexpr uint32_t kWorkerTaskStackWords = 2048;
constexpr UBaseType_t kWorkerTaskPriority = 6;

rmt_channel_handle_t gTxChannel = nullptr;
rmt_encoder_handle_t gEncoder = nullptr;
pcnt_unit_handle_t gPulseCountUnit = nullptr;
pcnt_channel_handle_t gPulseCountChannel = nullptr;
TaskHandle_t gWorkerTask = nullptr;
gpio_num_t gStepPin = GPIO_NUM_NC;

PulseEngine::StartConfig gMove = {};
uint32_t gPulsesTotal = 0;
uint16_t gHighTicks = 1;

struct EncodeProgress
{
    uint32_t pulseIndex = 0;
    uint32_t lowTailTicks = 0;
};
EncodeProgress gEncodeProgress = {};

PulseEngine::CompletionCallback gCompletionCallback = nullptr;
void *gCompletionCtx = nullptr;

volatile bool gRunning = false;
volatile bool gCompletionPending = false;
volatile uint32_t gCompletionPulses = 0;
volatile uint32_t gTxDoneIsrCount = 0;
bool gTxEnabled = false;

#if CONFIG_RMT_TX_ISR_CACHE_SAFE
#define MYR_RMT_CALLBACK_ATTR IRAM_ATTR
#else
#define MYR_RMT_CALLBACK_ATTR
#endif

static inline uint32_t IRAM_ATTR period_ticks_for_step(uint32_t step)
{
    if (gMove.startSpeedHz == gMove.endSpeedHz) return kRmtResolutionHz / gMove.startSpeedHz;
    if (gPulsesTotal <= 1) return kRmtResolutionHz / gMove.endSpeedHz;

    const uint32_t span = gPulsesTotal - 1;
    const uint32_t clampedStep = step > span ? span : step;
    const int64_t hz = static_cast<int64_t>(gMove.startSpeedHz) +
                       ((static_cast<int64_t>(gMove.endSpeedHz) - static_cast<int64_t>(gMove.startSpeedHz)) *
                        static_cast<int64_t>(clampedStep)) /
                           static_cast<int64_t>(span);

    uint32_t period = kRmtResolutionHz / static_cast<uint32_t>(hz > 0 ? hz : 1);
    if (period == 0) period = 1;

    const uint32_t minPeriod = static_cast<uint32_t>(gHighTicks) + 1U;
    if (period < minPeriod) period = minPeriod;
    return period;
}

RMT_ENCODER_FUNC_ATTR
static size_t encode_pulse_train(
    const void *data,
    size_t data_size,
    size_t symbols_written,
    size_t symbols_free,
    rmt_symbol_word_t *symbols,
    bool *done,
    void *arg)
{
    (void)data;
    (void)data_size;
    (void)arg;
    if (symbols == nullptr || done == nullptr) return 0;

    if (symbols_written == 0)
    {
        gEncodeProgress.pulseIndex = 0;
        gEncodeProgress.lowTailTicks = 0;
    }

    size_t encoded = 0;
    while (encoded < symbols_free)
    {
        if (gEncodeProgress.lowTailTicks > 0)
        {
            const uint16_t chunk0 = gEncodeProgress.lowTailTicks > kSymbolDurationMax
                                        ? kSymbolDurationMax
                                        : static_cast<uint16_t>(gEncodeProgress.lowTailTicks);
            gEncodeProgress.lowTailTicks -= chunk0;

            const uint16_t chunk1 = gEncodeProgress.lowTailTicks > kSymbolDurationMax
                                        ? kSymbolDurationMax
                                        : static_cast<uint16_t>(gEncodeProgress.lowTailTicks);
            gEncodeProgress.lowTailTicks -= chunk1;

            symbols[encoded] = {
                .duration0 = chunk0,
                .level0 = 0,
                .duration1 = chunk1,
                .level1 = 0,
            };
            encoded = encoded + 1;
            continue;
        }

        if (gEncodeProgress.pulseIndex >= gPulsesTotal)
        {
            *done = true;
            break;
        }

        const uint32_t periodTicks = period_ticks_for_step(gEncodeProgress.pulseIndex);
        const uint32_t lowTicksTotal = periodTicks > gHighTicks ? (periodTicks - gHighTicks) : 1U;
        const uint16_t lowChunk = lowTicksTotal > kSymbolDurationMax ? kSymbolDurationMax : static_cast<uint16_t>(lowTicksTotal);

        symbols[encoded] = {
            .duration0 = gHighTicks,
            .level0 = 1,
            .duration1 = lowChunk,
            .level1 = 0,
        };

        encoded = encoded + 1;
        gEncodeProgress.pulseIndex = gEncodeProgress.pulseIndex + 1;
        gEncodeProgress.lowTailTicks = lowTicksTotal - lowChunk;
    }

    if (gEncodeProgress.pulseIndex >= gPulsesTotal && gEncodeProgress.lowTailTicks == 0)
    {
        *done = true;
    }

    return encoded;
}

MYR_RMT_CALLBACK_ATTR
static bool on_rmt_tx_done(
    rmt_channel_handle_t txChannel,
    const rmt_tx_done_event_data_t *eventData,
    void *userCtx)
{
    (void)txChannel;
    (void)eventData;
    (void)userCtx;

    if (!gRunning) return false;

    __atomic_add_fetch(&gTxDoneIsrCount, 1U, __ATOMIC_RELAXED);

    BaseType_t highTaskWoken = pdFALSE;
    if (gWorkerTask != nullptr)
    {
        vTaskNotifyGiveFromISR(gWorkerTask, &highTaskWoken);
    }
    return highTaskWoken == pdTRUE;
}

static esp_err_t ensure_tx_enabled()
{
    if (gTxEnabled) return ESP_OK;
    if (gTxChannel == nullptr) return ESP_ERR_INVALID_STATE;

    esp_err_t err = rmt_enable(gTxChannel);
    if (err == ESP_OK) gTxEnabled = true;
    return err;
}

static void disable_tx()
{
    if (!gTxEnabled || gTxChannel == nullptr) return;

    esp_err_t err = rmt_disable(gTxChannel);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "rmt_disable failed: err=%s", esp_err_to_name(err));
    }
    gTxEnabled = false;
}

static uint32_t pulse_count_snapshot()
{
    if (gPulseCountUnit == nullptr) return 0;

    int count = 0;
    if (pcnt_unit_get_count(gPulseCountUnit, &count) != ESP_OK) return 0;
    if (count < 0) count = 0;

    uint32_t pulses = static_cast<uint32_t>(count);
    if (pulses > gPulsesTotal) pulses = gPulsesTotal;
    return pulses;
}

static void drain_tx_done_events()
{
    const uint32_t txDoneEvents = __atomic_exchange_n(&gTxDoneIsrCount, 0U, __ATOMIC_ACQ_REL);
    for (uint32_t i = 0; i < txDoneEvents; ++i)
    {
        if (!gRunning) continue;
        gRunning = false;
        gCompletionPending = true;
        gCompletionPulses = pulse_count_snapshot();
    }
}

static void emit_completion_if_pending()
{
    if (!gCompletionPending) return;

    gCompletionPending = false;
    const uint32_t completionPulses = gCompletionPulses;

    disable_tx();
    gpio_set_level(gStepPin, 0);

    if (gCompletionCallback != nullptr)
    {
        gCompletionCallback(completionPulses, gCompletionCtx);
    }
}

static void process_engine_events()
{
    drain_tx_done_events();
    emit_completion_if_pending();
}

static void pulse_engine_worker(void *arg)
{
    (void)arg;
    while (true)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        process_engine_events();
    }
}

static esp_err_t ensure_worker_task()
{
    if (gWorkerTask != nullptr) return ESP_OK;

    const BaseType_t created = xTaskCreatePinnedToCore(
        pulse_engine_worker,
        "pulse_evt",
        kWorkerTaskStackWords,
        nullptr,
        kWorkerTaskPriority,
        &gWorkerTask,
        tskNO_AFFINITY);

    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t init_pulse_counter()
{
    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = kPcntLowLimit;
    unitConfig.high_limit = kPcntHighLimit;
    unitConfig.intr_priority = 0;
    unitConfig.flags.accum_count = 1;

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

    err = pcnt_channel_set_edge_action(gPulseCountChannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (err != ESP_OK) return err;

    err = pcnt_channel_set_level_action(gPulseCountChannel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) return err;

    err = pcnt_unit_add_watch_point(gPulseCountUnit, kPcntHighLimit);
    if (err != ESP_OK) return err;

    err = pcnt_unit_set_glitch_filter(gPulseCountUnit, nullptr);
    if (err != ESP_OK) return err;

    err = pcnt_unit_enable(gPulseCountUnit);
    if (err != ESP_OK) return err;

    err = pcnt_unit_clear_count(gPulseCountUnit);
    if (err != ESP_OK) return err;

    return pcnt_unit_start(gPulseCountUnit);
}
} // namespace

bool PulseEngine::isInitialized = false;

esp_err_t PulseEngine::init(gpio_num_t stepPin)
{
    if (isInitialized) return ESP_OK;
    gStepPin = stepPin;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(gStepPin);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    gpio_set_level(gStepPin, 0);

    err = ensure_worker_task();
    if (err != ESP_OK) return err;

    err = init_pulse_counter();
    if (err != ESP_OK) return err;

    rmt_tx_channel_config_t txConfig = {};
    txConfig.gpio_num = gStepPin;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.resolution_hz = kRmtResolutionHz;
    txConfig.mem_block_symbols = kRmtMemBlockSymbols;
    txConfig.trans_queue_depth = kRmtQueueDepth;
    txConfig.intr_priority = kRmtIntrPriority;
    txConfig.flags.invert_out = 0;
    txConfig.flags.with_dma = 0;
    txConfig.flags.io_loop_back = 0;
    txConfig.flags.io_od_mode = 0;
    txConfig.flags.allow_pd = 0;

    err = rmt_new_tx_channel(&txConfig, &gTxChannel);
    if (err != ESP_OK) return err;

    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = on_rmt_tx_done;
    err = rmt_tx_register_event_callbacks(gTxChannel, &callbacks, nullptr);
    if (err != ESP_OK) return err;

    rmt_simple_encoder_config_t encoderConfig = {};
    encoderConfig.callback = encode_pulse_train;
    encoderConfig.arg = nullptr;
    encoderConfig.min_chunk_size = 16;
    err = rmt_new_simple_encoder(&encoderConfig, &gEncoder);
    if (err != ESP_OK) return err;

    err = ensure_tx_enabled();
    if (err != ESP_OK) return err;

    gHighTicks = (kRmtResolutionHz * kPulseHighUs) / 1000000U;
    if (gHighTicks == 0) gHighTicks = 1;

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
    if (!isInitialized) return ESP_ERR_INVALID_STATE;
    if (config.pulseCount == 0) return ESP_ERR_INVALID_ARG;
    if (config.startSpeedHz == 0 && config.endSpeedHz == 0) return ESP_ERR_INVALID_ARG;

    gRunning = false;
    gCompletionPending = false;
    gCompletionPulses = 0;
    __atomic_store_n(&gTxDoneIsrCount, 0U, __ATOMIC_RELEASE);

    disable_tx();
    gpio_set_level(gStepPin, 0);

    gMove = config;
    if (gMove.startSpeedHz == 0) gMove.startSpeedHz = gMove.endSpeedHz;
    if (gMove.endSpeedHz == 0) gMove.endSpeedHz = gMove.startSpeedHz;
    gPulsesTotal = gMove.pulseCount;

    gEncodeProgress.pulseIndex = 0;
    gEncodeProgress.lowTailTicks = 0;
    (void)pcnt_unit_clear_count(gPulseCountUnit);

    esp_err_t err = ensure_tx_enabled();
    if (err != ESP_OK) return err;

    rmt_transmit_config_t txConfig = {};
    txConfig.loop_count = 0;
    txConfig.flags.eot_level = 0;
    txConfig.flags.queue_nonblocking = 0;

    gRunning = true;
    err = rmt_transmit(gTxChannel, gEncoder, &gMove, sizeof(gMove), &txConfig);
    if (err != ESP_OK)
    {
        gRunning = false;
        disable_tx();
        gpio_set_level(gStepPin, 0);
        return err;
    }

    return ESP_OK;
}

uint32_t PulseEngine::stop()
{
    gRunning = false;
    gCompletionPending = false;
    __atomic_store_n(&gTxDoneIsrCount, 0U, __ATOMIC_RELEASE);

    const uint32_t pulses = pulse_count_snapshot();

    disable_tx();
    gpio_set_level(gStepPin, 0);
    return pulses;
}

void PulseEngine::service()
{
    if (gWorkerTask == nullptr)
    {
        process_engine_events();
    }
}

bool PulseEngine::isRunning()
{
    return gRunning;
}
