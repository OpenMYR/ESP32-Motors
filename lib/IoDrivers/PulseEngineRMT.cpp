#include "PulseEngine.h"

#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
const char *TAG = "PulseEngine";

constexpr uint32_t kRmtResolutionHz = 1000000;
constexpr uint32_t kPulseHighUs = 2;
constexpr uint16_t kRmtDurationMax = 32767;
constexpr int kRmtIntrPriority = 1;
constexpr size_t kRmtMemBlockSymbols = 128;
constexpr size_t kRmtQueueDepth = 4;

#ifndef MYR_RMT_CHUNK_HORIZON_US
#define MYR_RMT_CHUNK_HORIZON_US 20000
#endif
#ifndef MYR_RMT_CHUNK_MAX_STEPS
#define MYR_RMT_CHUNK_MAX_STEPS 64
#endif

constexpr uint32_t kChunkHorizonUsRaw = static_cast<uint32_t>(MYR_RMT_CHUNK_HORIZON_US);
constexpr uint32_t kChunkHorizonUs =
    kChunkHorizonUsRaw < 2000U ? 2000U : (kChunkHorizonUsRaw > 20000U ? 20000U : kChunkHorizonUsRaw);
constexpr uint32_t kChunkHorizonTicks = (kRmtResolutionHz / 1000000U) * kChunkHorizonUs;
constexpr uint32_t kChunkMaxSteps = static_cast<uint32_t>(MYR_RMT_CHUNK_MAX_STEPS);

constexpr uint32_t kMaxExpectedStepHz = 10000;
constexpr uint32_t kWorkerTaskStackWords = 3072;
constexpr UBaseType_t kWorkerTaskPriority = 6;
#ifndef MYR_RMT_WORKER_CORE
#define MYR_RMT_WORKER_CORE 1
#endif
constexpr BaseType_t kWorkerTaskCore = static_cast<BaseType_t>(MYR_RMT_WORKER_CORE);

constexpr uint32_t kEvtStartRequest = 1U << 0;
constexpr uint32_t kEvtAbortRequest = 1U << 1;
constexpr uint32_t kEvtTxDone = 1U << 2;
constexpr uint32_t kStopSyncTimeoutUs = 120000U;

constexpr size_t kChunkSymbolCapacity = 768;
constexpr uint8_t kChunkPipelineDepth = 4;

struct ChunkBuffer
{
    rmt_symbol_word_t symbols[kChunkSymbolCapacity] = {};
    uint16_t symbolCount = 0;
    uint16_t pulseCount = 0;
    uint64_t horizonTicksUsed = 0;
};

enum class EngineState : uint8_t
{
    UNINIT = 0,
    IDLE,
    START_PENDING,
    RUNNING,
    ABORTING,
    ERROR,
};

struct RunContext
{
    PulseEngine::StartConfig move = {};
    uint32_t targetPulses = 0;
    uint32_t nextPulseIndex = 0;
    uint32_t completedPulses = 0;
    uint32_t activeRunToken = 0;
    uint32_t activeAbortGen = 0;
};

struct Metrics
{
    uint32_t txChunksSent = 0;
    uint32_t abortCount = 0;
    uint32_t restartCount = 0;
    uint32_t invalidStateErrors = 0;
    uint64_t lastAbortToIdleUs = 0;
};

rmt_channel_handle_t gTxChannel = nullptr;
rmt_encoder_handle_t gCopyEncoder = nullptr;
TaskHandle_t gWorkerTask = nullptr;
gpio_num_t gStepPin = GPIO_NUM_NC;
uint16_t gPulseHighTicks = 1;

portMUX_TYPE gStateMux = portMUX_INITIALIZER_UNLOCKED;
volatile EngineState gState = EngineState::UNINIT;
uint32_t gAbortGeneration = 1;
uint32_t gPendingTxDoneCount = 0;
volatile bool gCompletionPending = false;
volatile uint32_t gCompletionPulses = 0;
volatile uint32_t gCompletionRunToken = 0;

PulseEngine::CompletionCallback gCompletionCallback = nullptr;
void *gCompletionCtx = nullptr;

PulseEngine::StartConfig gPendingStart = {};
RunContext gRun = {};
Metrics gMetrics = {};
ChunkBuffer gChunkPipeline[kChunkPipelineDepth] = {};
uint8_t gChunkHead = 0;
uint8_t gChunkTail = 0;
uint8_t gChunkCount = 0;
uint64_t gAbortRequestedAtUs = 0;

#if CONFIG_RMT_TX_ISR_CACHE_SAFE
#define MYR_RMT_CALLBACK_ATTR IRAM_ATTR
#else
#define MYR_RMT_CALLBACK_ATTR
#endif

static inline uint32_t safe_div_u32(uint64_t numer, uint32_t denom)
{
    if (denom == 0) return 0;
    return static_cast<uint32_t>(numer / static_cast<uint64_t>(denom));
}

static inline uint32_t period_ticks_for_step(const RunContext &run, uint32_t stepIndex)
{
    uint32_t startHz = run.move.startSpeedHz;
    uint32_t endHz = run.move.endSpeedHz;
    if (startHz == 0 && endHz == 0) return 0;
    if (startHz == 0) startHz = endHz;
    if (endHz == 0) endHz = startHz;

    uint32_t speedHz = startHz;
    if (startHz != endHz)
    {
        if (run.targetPulses <= 1)
        {
            speedHz = endHz;
        }
        else
        {
            const uint32_t span = run.targetPulses - 1;
            const uint32_t clampedStep = stepIndex > span ? span : stepIndex;
            if (endHz > startHz)
            {
                const uint64_t delta = static_cast<uint64_t>(endHz - startHz) * static_cast<uint64_t>(clampedStep);
                speedHz = startHz + static_cast<uint32_t>(delta / static_cast<uint64_t>(span));
            }
            else
            {
                const uint64_t delta = static_cast<uint64_t>(startHz - endHz) * static_cast<uint64_t>(clampedStep);
                speedHz = startHz - static_cast<uint32_t>(delta / static_cast<uint64_t>(span));
            }
        }
    }

    if (speedHz == 0) speedHz = 1;
    uint32_t period = safe_div_u32(static_cast<uint64_t>(kRmtResolutionHz) + static_cast<uint64_t>(speedHz / 2U), speedHz);
    if (period == 0) period = 1;

    const uint32_t minPeriod = static_cast<uint32_t>(gPulseHighTicks) + 1U;
    if (period < minPeriod) period = minPeriod;
    return period;
}

static inline uint16_t symbols_needed_for_period(uint32_t periodTicks)
{
    const uint32_t lowTicks = periodTicks > gPulseHighTicks ? (periodTicks - gPulseHighTicks) : 1U;
    uint32_t remain = lowTicks;
    uint16_t symbols = 1;

    const uint32_t low0 = remain > kRmtDurationMax ? kRmtDurationMax : remain;
    remain -= low0;

    while (remain > 0)
    {
        const uint32_t d0 = remain > kRmtDurationMax ? kRmtDurationMax : remain;
        remain -= d0;
        const uint32_t d1 = remain > kRmtDurationMax ? kRmtDurationMax : remain;
        remain -= d1;
        symbols = symbols + 1;
    }

    return symbols;
}

static inline bool append_pulse_symbols(ChunkBuffer &chunk, uint32_t periodTicks)
{
    const uint16_t required = symbols_needed_for_period(periodTicks);
    if ((static_cast<size_t>(chunk.symbolCount) + required) > kChunkSymbolCapacity) return false;

    uint32_t lowTicks = periodTicks > gPulseHighTicks ? (periodTicks - gPulseHighTicks) : 1U;
    const uint16_t high = gPulseHighTicks > kRmtDurationMax ? kRmtDurationMax : gPulseHighTicks;
    const uint16_t low0 = lowTicks > kRmtDurationMax ? kRmtDurationMax : static_cast<uint16_t>(lowTicks);
    lowTicks -= low0;

    chunk.symbols[chunk.symbolCount++] = {
        .duration0 = high,
        .level0 = 1,
        .duration1 = low0,
        .level1 = 0,
    };

    while (lowTicks > 0)
    {
        const uint16_t d0 = lowTicks > kRmtDurationMax ? kRmtDurationMax : static_cast<uint16_t>(lowTicks);
        lowTicks -= d0;
        const uint16_t d1 = lowTicks > kRmtDurationMax ? kRmtDurationMax : static_cast<uint16_t>(lowTicks);
        lowTicks -= d1;

        chunk.symbols[chunk.symbolCount++] = {
            .duration0 = d0,
            .level0 = 0,
            .duration1 = d1,
            .level1 = 0,
        };
    }

    chunk.pulseCount = static_cast<uint16_t>(chunk.pulseCount + 1U);
    chunk.horizonTicksUsed = chunk.horizonTicksUsed + static_cast<uint64_t>(periodTicks);
    return true;
}

static bool build_next_chunk(const RunContext &run, ChunkBuffer &chunk)
{
    chunk = {};
    if (run.nextPulseIndex >= run.targetPulses) return false;

    uint32_t remaining = run.targetPulses - run.nextPulseIndex;
    uint32_t maxChunkSteps = remaining;
    if (kChunkMaxSteps > 0 && maxChunkSteps > kChunkMaxSteps) maxChunkSteps = kChunkMaxSteps;
    if (remaining < maxChunkSteps) maxChunkSteps = remaining;
    if (maxChunkSteps == 0) maxChunkSteps = 1;
    const bool enforceHorizon = run.move.startSpeedHz != run.move.endSpeedHz;

    for (uint32_t i = 0; i < maxChunkSteps; ++i)
    {
        const uint32_t pulseIndex = run.nextPulseIndex + i;
        const uint32_t periodTicks = period_ticks_for_step(run, pulseIndex);
        if (periodTicks == 0) return false;

        if (enforceHorizon && chunk.pulseCount > 0)
        {
            if ((chunk.horizonTicksUsed + periodTicks) > static_cast<uint64_t>(kChunkHorizonTicks)) break;
        }

        if (!append_pulse_symbols(chunk, periodTicks))
        {
            if (chunk.pulseCount == 0) return false;
            break;
        }
    }

    if (chunk.pulseCount == 0)
    {
        // Always make forward progress, even at very low speed where one period exceeds the horizon.
        const uint32_t periodTicks = period_ticks_for_step(run, run.nextPulseIndex);
        if (periodTicks == 0) return false;
        if (!append_pulse_symbols(chunk, periodTicks)) return false;
    }

    return chunk.pulseCount > 0;
}

static void set_state(EngineState state)
{
    portENTER_CRITICAL(&gStateMux);
    gState = state;
    portEXIT_CRITICAL(&gStateMux);
}

static EngineState read_state()
{
    EngineState state = EngineState::UNINIT;
    portENTER_CRITICAL(&gStateMux);
    state = gState;
    portEXIT_CRITICAL(&gStateMux);
    return state;
}

static inline bool is_active_state(EngineState state)
{
    return state == EngineState::RUNNING || state == EngineState::ABORTING || state == EngineState::START_PENDING;
}

static void reset_chunk_pipeline()
{
    gChunkHead = 0;
    gChunkTail = 0;
    gChunkCount = 0;
}

static bool enqueue_chunk()
{
    if (gChunkCount >= kChunkPipelineDepth)
    {
        set_state(EngineState::ERROR);
        ESP_LOGE(TAG, "RMT chunk pipeline overflow");
        return false;
    }

    ChunkBuffer &chunk = gChunkPipeline[gChunkTail];
    if (!build_next_chunk(gRun, chunk))
    {
        set_state(EngineState::ERROR);
        ESP_LOGE(TAG, "RMT chunk build failed at pulse=%u/%u", static_cast<unsigned>(gRun.nextPulseIndex), static_cast<unsigned>(gRun.targetPulses));
        return false;
    }

    rmt_transmit_config_t txCfg = {};
    txCfg.loop_count = 0;
    txCfg.flags.eot_level = 0;
    txCfg.flags.queue_nonblocking = 0;

    const size_t payloadBytes = static_cast<size_t>(chunk.symbolCount) * sizeof(rmt_symbol_word_t);
    const esp_err_t err = rmt_transmit(gTxChannel, gCopyEncoder, chunk.symbols, payloadBytes, &txCfg);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_INVALID_STATE) gMetrics.invalidStateErrors = gMetrics.invalidStateErrors + 1U;
        set_state(EngineState::ERROR);
        ESP_LOGE(TAG, "RMT transmit failed: err=%s pulses=%u symbols=%u", esp_err_to_name(err), static_cast<unsigned>(chunk.pulseCount), static_cast<unsigned>(chunk.symbolCount));
        return false;
    }

    gChunkTail = static_cast<uint8_t>((gChunkTail + 1U) % kChunkPipelineDepth);
    gChunkCount = static_cast<uint8_t>(gChunkCount + 1U);
    gRun.nextPulseIndex = gRun.nextPulseIndex + chunk.pulseCount;
    gMetrics.txChunksSent = gMetrics.txChunksSent + 1U;
    return true;
}

static void finalize_abort_to_idle()
{
    set_state(EngineState::IDLE);
    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    if (gAbortRequestedAtUs != 0 && nowUs >= gAbortRequestedAtUs)
        gMetrics.lastAbortToIdleUs = nowUs - gAbortRequestedAtUs;

    gRun.targetPulses = 0;
    gRun.nextPulseIndex = 0;
    reset_chunk_pipeline();
}

static void handle_start_request()
{
    if (read_state() != EngineState::START_PENDING) return;

    gRun = {};
    gRun.move = gPendingStart;
    if (gRun.move.startSpeedHz == 0) gRun.move.startSpeedHz = gRun.move.endSpeedHz;
    if (gRun.move.endSpeedHz == 0) gRun.move.endSpeedHz = gRun.move.startSpeedHz;

    gRun.targetPulses = gRun.move.pulseCount;
    gRun.activeRunToken = gRun.move.runToken;
    gRun.activeAbortGen = __atomic_load_n(&gAbortGeneration, __ATOMIC_ACQUIRE);
    __atomic_store_n(&gPendingTxDoneCount, 0U, __ATOMIC_RELEASE);

    if (gRun.targetPulses == 0 || gRun.move.startSpeedHz == 0 || gRun.move.endSpeedHz == 0)
    {
        set_state(EngineState::ERROR);
        ESP_LOGE(TAG, "start rejected in worker: invalid config pulses=%u startHz=%u endHz=%u",
                 static_cast<unsigned>(gRun.targetPulses),
                 static_cast<unsigned>(gRun.move.startSpeedHz),
                 static_cast<unsigned>(gRun.move.endSpeedHz));
        return;
    }

    set_state(EngineState::RUNNING);
    reset_chunk_pipeline();
    while (gChunkCount < kChunkPipelineDepth && gRun.nextPulseIndex < gRun.targetPulses)
    {
        if (!enqueue_chunk()) break;
    }

    if (gChunkCount == 0)
    {
        set_state(EngineState::ERROR);
        ESP_LOGE(TAG, "RMT run start failed: no chunk queued");
        return;
    }

    ESP_LOGV(TAG, "RMT run started: token=%u pulses=%u abort_gen=%u",
             static_cast<unsigned>(gRun.activeRunToken),
             static_cast<unsigned>(gRun.targetPulses),
             static_cast<unsigned>(gRun.activeAbortGen));
}

static void handle_abort_request()
{
    gMetrics.abortCount = gMetrics.abortCount + 1U;
    gAbortRequestedAtUs = static_cast<uint64_t>(esp_timer_get_time());
    gCompletionPending = false;
    gCompletionPulses = 0;
    gCompletionRunToken = 0;
    __atomic_store_n(&gPendingTxDoneCount, 0U, __ATOMIC_RELEASE);

    const EngineState state = read_state();
    if (state == EngineState::RUNNING || state == EngineState::START_PENDING)
    {
        set_state(EngineState::ABORTING);
        if (gChunkCount == 0)
        {
            finalize_abort_to_idle();
        }
    }
}

static void handle_tx_done_one()
{
    if (gChunkCount > 0)
    {
        gRun.completedPulses = gRun.completedPulses + gChunkPipeline[gChunkHead].pulseCount;
        gChunkHead = static_cast<uint8_t>((gChunkHead + 1U) % kChunkPipelineDepth);
        gChunkCount = static_cast<uint8_t>(gChunkCount - 1U);
    }

    const EngineState state = read_state();
    if (state == EngineState::ABORTING)
    {
        if (gChunkCount == 0) finalize_abort_to_idle();
        return;
    }

    if (state != EngineState::RUNNING)
    {
        return;
    }

    if (gRun.activeAbortGen != __atomic_load_n(&gAbortGeneration, __ATOMIC_ACQUIRE))
    {
        set_state(EngineState::ABORTING);
        finalize_abort_to_idle();
        return;
    }

    if (gRun.completedPulses >= gRun.targetPulses)
    {
        gCompletionPulses = gRun.completedPulses;
        gCompletionRunToken = gRun.activeRunToken;
        gCompletionPending = true;
        set_state(EngineState::IDLE);
        gRun.targetPulses = 0;
        gRun.nextPulseIndex = 0;
        reset_chunk_pipeline();
        return;
    }

    while (gChunkCount < kChunkPipelineDepth && gRun.nextPulseIndex < gRun.targetPulses)
    {
        if (!enqueue_chunk()) break;
    }
}

static void handle_tx_done_events()
{
    while (true)
    {
        uint32_t pending = __atomic_load_n(&gPendingTxDoneCount, __ATOMIC_ACQUIRE);
        if (pending == 0U) break;
        if (!__atomic_compare_exchange_n(&gPendingTxDoneCount, &pending, pending - 1U, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
            continue;
        handle_tx_done_one();
    }
}

static void pulse_engine_worker(void *arg)
{
    (void)arg;

    while (true)
    {
        uint32_t events = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY) != pdTRUE) continue;

        if ((events & kEvtAbortRequest) != 0U) handle_abort_request();
        if ((events & kEvtStartRequest) != 0U) handle_start_request();
        if ((events & kEvtTxDone) != 0U) handle_tx_done_events();
    }
}

MYR_RMT_CALLBACK_ATTR
static bool on_rmt_tx_done(rmt_channel_handle_t txChannel, const rmt_tx_done_event_data_t *eventData, void *userCtx)
{
    (void)txChannel;
    (void)eventData;
    (void)userCtx;

    BaseType_t highTaskWoken = pdFALSE;
    if (gWorkerTask != nullptr)
    {
        (void)__atomic_add_fetch(&gPendingTxDoneCount, 1U, __ATOMIC_ACQ_REL);
        xTaskNotifyFromISR(gWorkerTask, kEvtTxDone, eSetBits, &highTaskWoken);
    }

    return highTaskWoken == pdTRUE;
}

static esp_err_t ensure_worker_task()
{
    if (gWorkerTask != nullptr) return ESP_OK;

    const BaseType_t ok = xTaskCreatePinnedToCore(
        pulse_engine_worker,
        "pulse_rmt",
        kWorkerTaskStackWords,
        nullptr,
        kWorkerTaskPriority,
        &gWorkerTask,
        kWorkerTaskCore);

    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
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

    rmt_tx_channel_config_t txCfg = {};
    txCfg.gpio_num = gStepPin;
    txCfg.clk_src = RMT_CLK_SRC_DEFAULT;
    txCfg.resolution_hz = kRmtResolutionHz;
    txCfg.mem_block_symbols = kRmtMemBlockSymbols;
    txCfg.trans_queue_depth = kRmtQueueDepth;
    txCfg.intr_priority = kRmtIntrPriority;
    txCfg.flags.with_dma = 0;
    txCfg.flags.invert_out = 0;
    txCfg.flags.io_loop_back = 0;
    txCfg.flags.io_od_mode = 0;
    txCfg.flags.allow_pd = 0;

    err = rmt_new_tx_channel(&txCfg, &gTxChannel);
    if (err != ESP_OK) return err;

    rmt_copy_encoder_config_t copyCfg = {};
    err = rmt_new_copy_encoder(&copyCfg, &gCopyEncoder);
    if (err != ESP_OK) return err;

    rmt_tx_event_callbacks_t cb = {};
    cb.on_trans_done = on_rmt_tx_done;
    err = rmt_tx_register_event_callbacks(gTxChannel, &cb, nullptr);
    if (err != ESP_OK) return err;

    err = rmt_enable(gTxChannel);
    if (err != ESP_OK) return err;

    gPulseHighTicks = static_cast<uint16_t>((kRmtResolutionHz * kPulseHighUs) / 1000000U);
    if (gPulseHighTicks == 0) gPulseHighTicks = 1;

    gMetrics = {};
    gRun = {};
    reset_chunk_pipeline();
    gCompletionPending = false;
    gCompletionPulses = 0;
    gCompletionRunToken = 0;
    __atomic_store_n(&gAbortGeneration, 1U, __ATOMIC_RELEASE);
    set_state(EngineState::IDLE);

    ESP_LOGI(TAG,
             "RMT init: step_pin=%d horizon_us=%u max_chunk_steps=%u worker_core=%d",
             static_cast<int>(gStepPin),
             static_cast<unsigned>(kChunkHorizonUs),
             static_cast<unsigned>(kChunkMaxSteps),
             static_cast<int>(kWorkerTaskCore));

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

    const EngineState state = read_state();
    if (state != EngineState::IDLE)
    {
        gMetrics.invalidStateErrors = gMetrics.invalidStateErrors + 1U;
        return ESP_ERR_INVALID_STATE;
    }

    gPendingStart = config;
    gMetrics.restartCount = gMetrics.restartCount + 1U;
    set_state(EngineState::START_PENDING);

    xTaskNotify(gWorkerTask, kEvtStartRequest, eSetBits);
    return ESP_OK;
}

uint32_t PulseEngine::stop()
{
    if (!isInitialized) return 0;
    if (gWorkerTask == nullptr) return 0;

    const EngineState state = read_state();
    if (!is_active_state(state))
    {
        gCompletionPending = false;
        gCompletionPulses = 0;
        gCompletionRunToken = 0;
        return 0;
    }

    gCompletionPending = false;
    gCompletionPulses = 0;
    gCompletionRunToken = 0;
    (void)__atomic_add_fetch(&gAbortGeneration, 1U, __ATOMIC_ACQ_REL);
    xTaskNotify(gWorkerTask, kEvtAbortRequest, eSetBits);

    const int64_t waitStartUs = esp_timer_get_time();
    while (is_active_state(read_state()))
    {
        const int64_t elapsedUs = esp_timer_get_time() - waitStartUs;
        if (elapsedUs >= static_cast<int64_t>(kStopSyncTimeoutUs))
        {
            ESP_LOGW(TAG, "RMT stop sync timed out at state=%u", static_cast<unsigned>(read_state()));
            break;
        }
        taskYIELD();
    }

    return gRun.completedPulses;
}

void PulseEngine::service()
{
    if (!gCompletionPending) return;

    const uint32_t pulses = gCompletionPulses;
    const uint32_t token = gCompletionRunToken;
    gCompletionPending = false;

    if (gCompletionCallback != nullptr)
        gCompletionCallback(pulses, token, gCompletionCtx);
}

bool PulseEngine::isRunning()
{
    const EngineState state = read_state();
    return state == EngineState::RUNNING || state == EngineState::ABORTING || state == EngineState::START_PENDING;
}
