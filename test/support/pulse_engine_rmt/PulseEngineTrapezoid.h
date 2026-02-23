#ifndef MYR_PULSE_ENGINE_TRAPEZOID_H
#define MYR_PULSE_ENGINE_TRAPEZOID_H

#include <stdint.h>

class PulseEngineTrapezoid
{
public:
    enum class Phase : uint8_t
    {
        IDLE = 0,
        ACCEL,
        CRUISE,
        DECEL,
    };

    struct Config
    {
        uint32_t totalSteps = 0;
        uint32_t startSpeedHz = 0;
        uint32_t cruiseSpeedHz = 0;
        uint32_t endSpeedHz = 0;
        uint32_t accelHzPerSec2 = 0;
    };

    struct State
    {
        Config config = {};
        uint32_t stepIndex = 0;
        uint32_t speedHz = 0;
        Phase phase = Phase::IDLE;
    };

    static bool begin(const Config &config, State *state);
    static bool nextStep(State *state, uint32_t *speedHz, Phase *phase);
    static bool isFinished(const State &state);

private:
    static uint32_t sanitize_speed(uint32_t speedHz);
    static uint32_t isqrt_u64(uint64_t value);
    static uint32_t braking_steps(uint32_t speedHz, uint32_t endSpeedHz, uint32_t accelHzPerSec2);
    static uint32_t next_accel_speed(uint32_t speedHz, uint32_t cruiseSpeedHz, uint32_t accelHzPerSec2);
    static uint32_t next_decel_speed(uint32_t speedHz, uint32_t endSpeedHz, uint32_t accelHzPerSec2);
};

#endif // MYR_PULSE_ENGINE_TRAPEZOID_H
