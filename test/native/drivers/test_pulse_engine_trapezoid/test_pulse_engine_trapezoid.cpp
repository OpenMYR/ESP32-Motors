#include <unity.h>

#include "../../../../lib/IoDrivers/PulseEngineTrapezoid.h"

// Native env compiles tests only, so include implementation units directly.
#include "../../../../lib/IoDrivers/PulseEngineTrapezoid.cpp"

void test_begin_rejects_invalid_total_steps(void)
{
    PulseEngineTrapezoid::State state = {};
    PulseEngineTrapezoid::Config config = {};
    config.totalSteps = 0;
    config.startSpeedHz = 100;
    config.endSpeedHz = 100;
    TEST_ASSERT_FALSE(PulseEngineTrapezoid::begin(config, &state));
}

void test_begin_rejects_zero_terminal_speeds(void)
{
    PulseEngineTrapezoid::State state = {};
    PulseEngineTrapezoid::Config config = {};
    config.totalSteps = 32;
    config.startSpeedHz = 0;
    config.endSpeedHz = 0;
    TEST_ASSERT_FALSE(PulseEngineTrapezoid::begin(config, &state));
}

void test_trapezoid_profile_covers_accel_cruise_decel_and_exact_step_count(void)
{
    PulseEngineTrapezoid::State state = {};
    PulseEngineTrapezoid::Config config = {};
    config.totalSteps = 200;
    config.startSpeedHz = 100;
    config.cruiseSpeedHz = 400;
    config.endSpeedHz = 100;
    config.accelHzPerSec2 = 2000;

    TEST_ASSERT_TRUE(PulseEngineTrapezoid::begin(config, &state));

    uint32_t steps = 0;
    uint32_t lastSpeed = 0;
    bool sawAccel = false;
    bool sawCruise = false;
    bool sawDecel = false;

    uint32_t speedHz = 0;
    PulseEngineTrapezoid::Phase phase = PulseEngineTrapezoid::Phase::IDLE;
    while (PulseEngineTrapezoid::nextStep(&state, &speedHz, &phase))
    {
        steps = steps + 1;
        TEST_ASSERT_GREATER_THAN_UINT32(0, speedHz);

        if (phase == PulseEngineTrapezoid::Phase::ACCEL)
        {
            sawAccel = true;
            TEST_ASSERT_TRUE(speedHz <= config.cruiseSpeedHz);
        }
        else if (phase == PulseEngineTrapezoid::Phase::CRUISE)
        {
            sawCruise = true;
        }
        else if (phase == PulseEngineTrapezoid::Phase::DECEL)
        {
            sawDecel = true;
            if (lastSpeed > 0)
            {
                TEST_ASSERT_TRUE(speedHz <= config.cruiseSpeedHz);
            }
        }

        lastSpeed = speedHz;
    }

    TEST_ASSERT_EQUAL_UINT32(config.totalSteps, steps);
    TEST_ASSERT_TRUE(sawAccel);
    TEST_ASSERT_TRUE(sawCruise);
    TEST_ASSERT_TRUE(sawDecel);
    TEST_ASSERT_TRUE(PulseEngineTrapezoid::isFinished(state));
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_begin_rejects_invalid_total_steps);
    RUN_TEST(test_begin_rejects_zero_terminal_speeds);
    RUN_TEST(test_trapezoid_profile_covers_accel_cruise_decel_and_exact_step_count);
    return UNITY_END();
}
