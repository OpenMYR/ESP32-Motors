#ifndef MYR_PULSE_ENGINE_RMT_SEQUENCER_H
#define MYR_PULSE_ENGINE_RMT_SEQUENCER_H

#include <stddef.h>
#include <stdint.h>

#include "PulseEngineRmtModel.h"

class PulseEngineRmtSequencer
{
public:
    class ITx
    {
    public:
        virtual ~ITx() = default;
        virtual bool queueTransaction(const PulseEngineRmtSymbol *symbols, size_t symbolCount) = 0;
    };

    static constexpr size_t kFrameRingSize = 8;

    bool start(ITx *tx, const PulseEngineRmtModel::BuildConfig &config);
    uint32_t abort();
    void onTransactionDone();

    bool isRunning() const;
    uint32_t pulsesDone() const;

    bool completionPending() const;
    uint32_t takeCompletionPulses();

    bool hadTxError() const;
    uint32_t nearUnderrunEvents() const;
    const PulseEngineRmtModel::Stats &stats() const;

private:
    bool queue_next_frame();
    void reset_state();
    void mark_complete();

    ITx *tx_ = nullptr;
    PulseEngineRmtModel model_ = {};
    PulseEngineRmtFrame frameRing_[kFrameRingSize] = {};

    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;

    uint32_t pulsesDone_ = 0;
    uint32_t completionPulses_ = 0;

    bool running_ = false;
    bool completionPending_ = false;
    bool txError_ = false;
    uint32_t nearUnderrunEvents_ = 0;
};

#endif // MYR_PULSE_ENGINE_RMT_SEQUENCER_H
