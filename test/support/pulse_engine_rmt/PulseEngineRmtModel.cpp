#include "PulseEngineRmtModel.h"

#include <limits.h>

namespace {
uint32_t rmt_model_max_u32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

uint32_t rmt_model_min_u32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

uint32_t rmt_model_clamp_min_u32(uint32_t value, uint32_t min)
{
    return value < min ? min : value;
}
} // namespace

bool PulseEngineRmtModel::begin(const BuildConfig &config)
{
    stats_ = {};
    config_ = config;

    if (config_.pulseCount == 0) return false;
    if (config_.resolutionHz == 0) return false;

    if (config_.pulseHighTicks == 0) config_.pulseHighTicks = 1;

    if (config_.startSpeedHz == 0 && config_.endSpeedHz == 0) return false;
    if (config_.startSpeedHz == 0) config_.startSpeedHz = config_.endSpeedHz;
    if (config_.endSpeedHz == 0) config_.endSpeedHz = config_.startSpeedHz;

    useTrapezoid_ = config_.useTrapezoid || config_.cruiseSpeedHz > 0 || config_.accelHzPerSec2 > 0;
    if (useTrapezoid_)
    {
        PulseEngineTrapezoid::Config trapezoidConfig = {};
        trapezoidConfig.totalSteps = config_.pulseCount;
        trapezoidConfig.startSpeedHz = config_.startSpeedHz;
        trapezoidConfig.endSpeedHz = config_.endSpeedHz;
        trapezoidConfig.cruiseSpeedHz =
            config_.cruiseSpeedHz > 0 ? config_.cruiseSpeedHz : rmt_model_max_u32(config_.startSpeedHz, config_.endSpeedHz);
        trapezoidConfig.accelHzPerSec2 =
            config_.accelHzPerSec2 > 0 ? config_.accelHzPerSec2 : kDefaultAccelHzPerSec2;

        if (!PulseEngineTrapezoid::begin(trapezoidConfig, &trapezoidState_)) return false;
    }

    return true;
}

bool PulseEngineRmtModel::hasPendingPulses() const
{
    return stats_.pulsesPlanned < config_.pulseCount;
}

uint32_t PulseEngineRmtModel::plannedPulseCount() const
{
    return stats_.pulsesPlanned;
}

const PulseEngineRmtModel::Stats &PulseEngineRmtModel::stats() const
{
    return stats_;
}

uint32_t PulseEngineRmtModel::linear_period_ticks_for_step(uint32_t stepIndex) const
{
    const uint32_t start = config_.startSpeedHz;
    const uint32_t end = config_.endSpeedHz;

    uint32_t speedHz = start;
    if (start != end)
    {
        if (config_.pulseCount <= 1)
        {
            speedHz = end;
        }
        else
        {
            const uint32_t span = config_.pulseCount - 1;
            const uint32_t step = stepIndex > span ? span : stepIndex;

            if (end > start)
            {
                const uint64_t delta = static_cast<uint64_t>(end - start) * static_cast<uint64_t>(step);
                speedHz = start + static_cast<uint32_t>(delta / static_cast<uint64_t>(span));
            }
            else
            {
                const uint64_t delta = static_cast<uint64_t>(start - end) * static_cast<uint64_t>(step);
                speedHz = start - static_cast<uint32_t>(delta / static_cast<uint64_t>(span));
            }
        }
    }

    return period_ticks_from_speed(speedHz);
}

uint32_t PulseEngineRmtModel::period_ticks_from_speed(uint32_t speedHz) const
{
    const uint32_t safeSpeedHz = rmt_model_clamp_min_u32(speedHz, 1);

    uint32_t period =
        static_cast<uint32_t>((static_cast<uint64_t>(config_.resolutionHz) + (safeSpeedHz / 2U)) /
                              static_cast<uint64_t>(safeSpeedHz));

    if (period == 0) period = 1;
    const uint32_t minPeriod = config_.pulseHighTicks + 1U;
    if (period < minPeriod) period = minPeriod;
    return period;
}

uint16_t PulseEngineRmtModel::symbols_needed_for_period(uint32_t periodTicks) const
{
    const uint32_t lowTicks = periodTicks > config_.pulseHighTicks ? (periodTicks - config_.pulseHighTicks) : 1U;

    uint16_t symbols = 1;
    uint32_t remain = lowTicks;

    const uint32_t firstLow = rmt_model_min_u32(remain, static_cast<uint32_t>(kMaxDurationPerHalfSymbol));
    remain -= firstLow;

    while (remain > 0)
    {
        const uint32_t d0 = rmt_model_min_u32(remain, static_cast<uint32_t>(kMaxDurationPerHalfSymbol));
        remain -= d0;
        const uint32_t d1 = rmt_model_min_u32(remain, static_cast<uint32_t>(kMaxDurationPerHalfSymbol));
        remain -= d1;
        symbols = symbols + 1;
    }

    return symbols;
}

bool PulseEngineRmtModel::append_pulse_symbols(PulseEngineRmtFrame *frame, uint32_t periodTicks)
{
    if (frame == nullptr) return false;
    if (frame->symbolCount >= PulseEngineRmtFrame::kMaxSymbols) return false;

    uint32_t lowTicks = periodTicks > config_.pulseHighTicks ? (periodTicks - config_.pulseHighTicks) : 1U;

    const uint16_t high =
        static_cast<uint16_t>(rmt_model_min_u32(config_.pulseHighTicks, static_cast<uint32_t>(kMaxDurationPerHalfSymbol)));
    const uint16_t low0 =
        static_cast<uint16_t>(rmt_model_min_u32(lowTicks, static_cast<uint32_t>(kMaxDurationPerHalfSymbol)));
    lowTicks -= low0;

    frame->symbols[frame->symbolCount++] = PulseEngineRmtSymbol::make(high, true, low0, false);

    while (lowTicks > 0)
    {
        if (frame->symbolCount >= PulseEngineRmtFrame::kMaxSymbols) return false;
        const uint16_t d0 =
            static_cast<uint16_t>(rmt_model_min_u32(lowTicks, static_cast<uint32_t>(kMaxDurationPerHalfSymbol)));
        lowTicks -= d0;
        const uint16_t d1 =
            static_cast<uint16_t>(rmt_model_min_u32(lowTicks, static_cast<uint32_t>(kMaxDurationPerHalfSymbol)));
        lowTicks -= d1;
        frame->symbols[frame->symbolCount++] = PulseEngineRmtSymbol::make(d0, false, d1, false);
    }

    return true;
}

bool PulseEngineRmtModel::buildNextFrame(PulseEngineRmtFrame *frame)
{
    if (frame == nullptr) return false;

    *frame = {};

    while (stats_.pulsesPlanned < config_.pulseCount)
    {
        if (config_.frameMaxPulses > 0 && frame->pulseCount >= config_.frameMaxPulses)
        {
            break;
        }

        uint32_t periodTicks = 0;
        PulseEngineTrapezoid::Phase phase = PulseEngineTrapezoid::Phase::CRUISE;
        PulseEngineTrapezoid::State probeState = trapezoidState_;

        if (useTrapezoid_)
        {
            uint32_t speedHz = 0;
            if (!PulseEngineTrapezoid::nextStep(&probeState, &speedHz, &phase))
            {
                break;
            }
            periodTicks = period_ticks_from_speed(speedHz);
        }
        else
        {
            periodTicks = linear_period_ticks_for_step(stats_.pulsesPlanned);
        }

        const uint16_t requiredSymbols = symbols_needed_for_period(periodTicks);
        if ((frame->symbolCount + requiredSymbols) > PulseEngineRmtFrame::kMaxSymbols)
        {
            if (frame->pulseCount == 0)
            {
                return false;
            }
            break;
        }

        if (!append_pulse_symbols(frame, periodTicks))
        {
            if (frame->pulseCount == 0) return false;
            break;
        }

        if (useTrapezoid_)
        {
            trapezoidState_ = probeState;
        }

        frame->pulseCount = frame->pulseCount + 1;
        frame->ticksTotal = frame->ticksTotal + static_cast<uint64_t>(periodTicks);

        stats_.pulsesPlanned = stats_.pulsesPlanned + 1;
        stats_.ticksPlanned = stats_.ticksPlanned + static_cast<uint64_t>(periodTicks);

        switch (phase)
        {
            case PulseEngineTrapezoid::Phase::ACCEL:
                frame->accelSteps = frame->accelSteps + 1;
                stats_.accelSteps = stats_.accelSteps + 1;
                break;
            case PulseEngineTrapezoid::Phase::DECEL:
                frame->decelSteps = frame->decelSteps + 1;
                stats_.decelSteps = stats_.decelSteps + 1;
                break;
            case PulseEngineTrapezoid::Phase::CRUISE:
            case PulseEngineTrapezoid::Phase::IDLE:
            default:
                frame->cruiseSteps = frame->cruiseSteps + 1;
                stats_.cruiseSteps = stats_.cruiseSteps + 1;
                break;
        }
    }

    return frame->pulseCount > 0;
}
