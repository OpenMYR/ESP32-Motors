#include "PulseEngineTrapezoid.h"

#include <limits.h>

namespace {
constexpr uint32_t kMinSpeedHz = 1;
constexpr uint32_t kMinAccelHzPerSec2 = 1;

uint32_t max_u32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

uint32_t min_u32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}
} // namespace

uint32_t PulseEngineTrapezoid::sanitize_speed(uint32_t speedHz)
{
    return speedHz == 0 ? kMinSpeedHz : speedHz;
}

uint32_t PulseEngineTrapezoid::isqrt_u64(uint64_t value)
{
    uint64_t rem = 0;
    uint64_t root = 0;

    for (int i = 0; i < 32; ++i)
    {
        root <<= 1;
        rem = (rem << 2) | (value >> 62);
        value <<= 2;

        const uint64_t candidate = (root << 1) + 1;
        if (rem >= candidate)
        {
            rem -= candidate;
            root += 1;
        }
    }

    if (root > UINT32_MAX) return UINT32_MAX;
    return static_cast<uint32_t>(root);
}

uint32_t PulseEngineTrapezoid::braking_steps(uint32_t speedHz, uint32_t endSpeedHz, uint32_t accelHzPerSec2)
{
    if (speedHz <= endSpeedHz) return 0;

    const uint32_t accel = accelHzPerSec2 == 0 ? kMinAccelHzPerSec2 : accelHzPerSec2;
    const uint64_t speed2 = static_cast<uint64_t>(speedHz) * static_cast<uint64_t>(speedHz);
    const uint64_t end2 = static_cast<uint64_t>(endSpeedHz) * static_cast<uint64_t>(endSpeedHz);
    if (speed2 <= end2) return 0;

    const uint64_t numerator = speed2 - end2;
    const uint64_t denominator = static_cast<uint64_t>(accel) * 2ULL;
    if (denominator == 0) return UINT32_MAX;

    const uint64_t steps = (numerator + denominator - 1ULL) / denominator;
    if (steps > UINT32_MAX) return UINT32_MAX;
    return static_cast<uint32_t>(steps);
}

uint32_t PulseEngineTrapezoid::next_accel_speed(uint32_t speedHz, uint32_t cruiseSpeedHz, uint32_t accelHzPerSec2)
{
    if (speedHz >= cruiseSpeedHz) return cruiseSpeedHz;

    const uint32_t accel = accelHzPerSec2 == 0 ? kMinAccelHzPerSec2 : accelHzPerSec2;
    const uint64_t speed2 = static_cast<uint64_t>(speedHz) * static_cast<uint64_t>(speedHz);
    const uint64_t target2 = speed2 + static_cast<uint64_t>(accel) * 2ULL;

    uint32_t next = isqrt_u64(target2);
    if (next <= speedHz) next = speedHz + 1;
    if (next > cruiseSpeedHz) next = cruiseSpeedHz;
    return next;
}

uint32_t PulseEngineTrapezoid::next_decel_speed(uint32_t speedHz, uint32_t endSpeedHz, uint32_t accelHzPerSec2)
{
    if (speedHz <= endSpeedHz) return endSpeedHz;

    const uint32_t accel = accelHzPerSec2 == 0 ? kMinAccelHzPerSec2 : accelHzPerSec2;
    const uint64_t speed2 = static_cast<uint64_t>(speedHz) * static_cast<uint64_t>(speedHz);
    const uint64_t delta = static_cast<uint64_t>(accel) * 2ULL;
    const uint64_t target2 = speed2 > delta ? speed2 - delta : 0;

    uint32_t next = isqrt_u64(target2);
    if (next >= speedHz && speedHz > 0) next = speedHz - 1;
    if (next < endSpeedHz) next = endSpeedHz;
    if (next == 0) next = kMinSpeedHz;
    return next;
}

bool PulseEngineTrapezoid::begin(const Config &config, State *state)
{
    if (state == nullptr) return false;
    if (config.totalSteps == 0) return false;
    if (config.startSpeedHz == 0 && config.endSpeedHz == 0) return false;

    state->config = config;
    state->config.startSpeedHz = sanitize_speed(config.startSpeedHz);
    state->config.endSpeedHz = sanitize_speed(config.endSpeedHz);

    const uint32_t maxTerminal = max_u32(state->config.startSpeedHz, state->config.endSpeedHz);
    uint32_t cruiseSpeedHz = config.cruiseSpeedHz;
    if (cruiseSpeedHz == 0) cruiseSpeedHz = maxTerminal;
    state->config.cruiseSpeedHz = max_u32(cruiseSpeedHz, maxTerminal);
    state->config.accelHzPerSec2 = config.accelHzPerSec2 == 0 ? kMinAccelHzPerSec2 : config.accelHzPerSec2;

    state->stepIndex = 0;
    state->speedHz = state->config.startSpeedHz;
    state->phase = Phase::IDLE;
    return true;
}

bool PulseEngineTrapezoid::nextStep(State *state, uint32_t *speedHz, Phase *phase)
{
    if (state == nullptr || speedHz == nullptr || phase == nullptr) return false;
    if (state->stepIndex >= state->config.totalSteps) return false;

    const uint32_t remaining = state->config.totalSteps - state->stepIndex;
    const uint32_t currentSpeed = sanitize_speed(state->speedHz);
    const uint32_t brakeSteps = braking_steps(currentSpeed, state->config.endSpeedHz, state->config.accelHzPerSec2);

    Phase nextPhase = Phase::CRUISE;
    if (remaining <= (brakeSteps + 1U) && currentSpeed > state->config.endSpeedHz)
    {
        nextPhase = Phase::DECEL;
    }
    else if (currentSpeed < state->config.cruiseSpeedHz)
    {
        nextPhase = Phase::ACCEL;
    }
    else if (currentSpeed > state->config.cruiseSpeedHz)
    {
        nextPhase = Phase::DECEL;
    }

    *speedHz = currentSpeed;
    *phase = nextPhase;

    uint32_t followingSpeed = currentSpeed;
    switch (nextPhase)
    {
        case Phase::ACCEL:
            followingSpeed = next_accel_speed(currentSpeed, state->config.cruiseSpeedHz, state->config.accelHzPerSec2);
            break;
        case Phase::DECEL:
            followingSpeed = next_decel_speed(currentSpeed, state->config.endSpeedHz, state->config.accelHzPerSec2);
            break;
        case Phase::CRUISE:
            followingSpeed = min_u32(max_u32(currentSpeed, state->config.endSpeedHz), state->config.cruiseSpeedHz);
            break;
        case Phase::IDLE:
        default:
            followingSpeed = currentSpeed;
            break;
    }

    state->phase = nextPhase;
    state->speedHz = sanitize_speed(followingSpeed);
    state->stepIndex = state->stepIndex + 1U;
    return true;
}

bool PulseEngineTrapezoid::isFinished(const State &state)
{
    return state.stepIndex >= state.config.totalSteps;
}
