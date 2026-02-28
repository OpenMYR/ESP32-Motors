#ifndef MYR_PULSE_ENGINE_RMT_MODEL_H
#define MYR_PULSE_ENGINE_RMT_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "PulseEngineTrapezoid.h"

struct PulseEngineRmtSymbol
{
    uint32_t value = 0;

    static PulseEngineRmtSymbol make(uint16_t duration0, bool level0, uint16_t duration1, bool level1)
    {
        PulseEngineRmtSymbol symbol = {};
        symbol.value =
            (static_cast<uint32_t>(duration0 & 0x7FFFU)) |
            (static_cast<uint32_t>(level0 ? 1U : 0U) << 15U) |
            (static_cast<uint32_t>(duration1 & 0x7FFFU) << 16U) |
            (static_cast<uint32_t>(level1 ? 1U : 0U) << 31U);
        return symbol;
    }

    uint16_t duration0() const { return static_cast<uint16_t>(value & 0x7FFFU); }
    uint16_t duration1() const { return static_cast<uint16_t>((value >> 16U) & 0x7FFFU); }
    bool level0() const { return ((value >> 15U) & 0x1U) != 0U; }
    bool level1() const { return ((value >> 31U) & 0x1U) != 0U; }
};

struct PulseEngineRmtFrame
{
    static constexpr size_t kMaxSymbols = 512;

    PulseEngineRmtSymbol symbols[kMaxSymbols] = {};
    uint16_t symbolCount = 0;
    uint16_t pulseCount = 0;
    uint16_t accelSteps = 0;
    uint16_t cruiseSteps = 0;
    uint16_t decelSteps = 0;
    uint64_t ticksTotal = 0;
};

class PulseEngineRmtModel
{
public:
    struct BuildConfig
    {
        uint32_t resolutionHz = 1000000;
        uint32_t pulseHighTicks = 2;
        uint32_t pulseCount = 0;
        uint32_t startSpeedHz = 0;
        uint32_t endSpeedHz = 0;
        uint32_t cruiseSpeedHz = 0;
        uint32_t accelHzPerSec2 = 0;
        bool useTrapezoid = false;
        uint16_t frameMaxPulses = 0;
    };

    struct Stats
    {
        uint32_t pulsesPlanned = 0;
        uint32_t accelSteps = 0;
        uint32_t cruiseSteps = 0;
        uint32_t decelSteps = 0;
        uint64_t ticksPlanned = 0;
    };

    bool begin(const BuildConfig &config);
    bool buildNextFrame(PulseEngineRmtFrame *frame);

    bool hasPendingPulses() const;
    uint32_t plannedPulseCount() const;
    const Stats &stats() const;

private:
    static constexpr uint16_t kMaxDurationPerHalfSymbol = 0x7FFFU;
    static constexpr uint32_t kDefaultAccelHzPerSec2 = 4000;

    uint32_t linear_period_ticks_for_step(uint32_t stepIndex) const;
    uint32_t period_ticks_from_speed(uint32_t speedHz) const;
    uint16_t symbols_needed_for_period(uint32_t periodTicks) const;
    bool append_pulse_symbols(PulseEngineRmtFrame *frame, uint32_t periodTicks);

    BuildConfig config_ = {};
    Stats stats_ = {};

    bool useTrapezoid_ = false;
    PulseEngineTrapezoid::State trapezoidState_ = {};
};

#endif // MYR_PULSE_ENGINE_RMT_MODEL_H
