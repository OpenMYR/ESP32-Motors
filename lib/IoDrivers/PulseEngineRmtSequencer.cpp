#include "PulseEngineRmtSequencer.h"

void PulseEngineRmtSequencer::reset_state()
{
    head_ = 0;
    tail_ = 0;
    count_ = 0;

    pulsesDone_ = 0;
    completionPulses_ = 0;

    running_ = false;
    completionPending_ = false;
    txError_ = false;
    nearUnderrunEvents_ = 0;
}

bool PulseEngineRmtSequencer::start(ITx *tx, const PulseEngineRmtModel::BuildConfig &config)
{
    reset_state();
    if (tx == nullptr) return false;
    if (!model_.begin(config)) return false;

    tx_ = tx;

    while (count_ < kFrameRingSize && model_.hasPendingPulses())
    {
        if (!queue_next_frame())
        {
            txError_ = true;
            break;
        }
    }

    if (count_ == 0)
    {
        mark_complete();
        return !txError_;
    }

    running_ = true;
    return !txError_;
}

uint32_t PulseEngineRmtSequencer::abort()
{
    const uint32_t pulses = pulsesDone_;
    reset_state();
    return pulses;
}

bool PulseEngineRmtSequencer::queue_next_frame()
{
    if (tx_ == nullptr) return false;
    if (count_ >= kFrameRingSize) return false;

    PulseEngineRmtFrame *frame = &frameRing_[tail_];
    if (!model_.buildNextFrame(frame))
    {
        return false;
    }

    if (!tx_->queueTransaction(frame->symbols, frame->symbolCount))
    {
        return false;
    }

    tail_ = (tail_ + 1U) % kFrameRingSize;
    count_ = count_ + 1U;
    return true;
}

void PulseEngineRmtSequencer::mark_complete()
{
    running_ = false;
    completionPending_ = true;
    completionPulses_ = pulsesDone_;
}

void PulseEngineRmtSequencer::onTransactionDone()
{
    if (count_ == 0)
    {
        if (!running_ && !completionPending_ && !model_.hasPendingPulses())
        {
            mark_complete();
        }
        return;
    }

    if (running_ && count_ == 1 && model_.hasPendingPulses())
    {
        nearUnderrunEvents_ = nearUnderrunEvents_ + 1U;
    }

    const PulseEngineRmtFrame *completedFrame = &frameRing_[head_];
    pulsesDone_ = pulsesDone_ + completedFrame->pulseCount;

    head_ = (head_ + 1U) % kFrameRingSize;
    count_ = count_ - 1U;

    if (running_)
    {
        while (count_ < kFrameRingSize && model_.hasPendingPulses())
        {
            if (!queue_next_frame())
            {
                txError_ = true;
                break;
            }
        }

        if (txError_)
        {
            running_ = false;
        }
        if (!model_.hasPendingPulses() && count_ == 0)
        {
            mark_complete();
        }
    }
}

bool PulseEngineRmtSequencer::isRunning() const
{
    return running_;
}

uint32_t PulseEngineRmtSequencer::pulsesDone() const
{
    return pulsesDone_;
}

bool PulseEngineRmtSequencer::completionPending() const
{
    return completionPending_;
}

uint32_t PulseEngineRmtSequencer::takeCompletionPulses()
{
    if (!completionPending_) return 0;

    completionPending_ = false;
    return completionPulses_;
}

bool PulseEngineRmtSequencer::hadTxError() const
{
    return txError_;
}

uint32_t PulseEngineRmtSequencer::nearUnderrunEvents() const
{
    return nearUnderrunEvents_;
}

const PulseEngineRmtModel::Stats &PulseEngineRmtSequencer::stats() const
{
    return model_.stats();
}
