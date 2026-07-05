#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "StepperDriver.h"

static void assert_uint64_equal(uint64_t expected, uint64_t actual, const char *message)
{
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected >> 32), static_cast<uint32_t>(actual >> 32), message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual), message);
}

static void assert_int64_equal(int64_t expected, int64_t actual, const char *message)
{
    TEST_ASSERT_TRUE_MESSAGE(expected == actual, message);
}

static void assert_motion_plan_equal(const StepperDriver::MotionPlan &expected, const StepperDriver::MotionPlan &actual, const char *message)
{
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.goalStep, actual.goalStep, message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.steps, actual.steps, message);
    assert_uint64_equal(expected.durationUs, actual.durationUs, message);
}

static void assert_microstep_motion_plan_equal(
    const StepperDriver::MicrostepMotionPlan &expected,
    const StepperDriver::MicrostepMotionPlan &actual,
    const char *message)
{
    assert_int64_equal(expected.goalMicrosteps, actual.goalMicrosteps, message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.pulses, actual.pulses, message);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(expected.pulseRateHz, actual.pulseRateHz, message);
    assert_uint64_equal(expected.durationUs, actual.durationUs, message);
    assert_int64_equal(expected.microstepUnitsPerPulse, actual.microstepUnitsPerPulse, message);
}

static void wait_for_monitor_attach(void)
{
    // Keep a short bootstrap window so hardware CI serial monitor reliably attaches before Unity output starts.
    for (int i = 0; i < 16; ++i) {
        printf("test_stepper_motion_plan bootstrap %d/16\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_plan_relative_move_uses_delta_for_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(131, 100, 100);
    TEST_ASSERT_EQUAL_INT32(231, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(100, plan.steps);
    assert_uint64_equal(1000000ULL, plan.durationUs, "relative move duration");
}

void test_stepper_command_units_scale_to_internal_microsteps(void)
{
    assert_int64_equal(2560, StepperDriver::commandUnitsToMicrosteps(10, 1), "full-step command units");
    assert_int64_equal(10, StepperDriver::commandUnitsToMicrosteps(10, 256), "finest microstep command units");
    assert_int64_equal(-640, StepperDriver::commandUnitsToMicrosteps(-10, 4), "coarse microstep command units");
}

void test_microstep_absolute_move_uses_internal_full_step_units(void)
{
    StepperDriver::MicrostepMotionPlan expected = {};
    expected.goalMicrosteps = 256;
    expected.pulses = 1;
    expected.pulseRateHz = 100;
    expected.durationUs = 10000ULL;
    expected.microstepUnitsPerPulse = 256;

    const StepperDriver::MicrostepMotionPlan actual = StepperDriver::planAbsoluteMoveMicrosteps(0, 256, 100, 1);

    assert_microstep_motion_plan_equal(expected, actual, "full-step internal-unit absolute plan");
}

void test_microstep_finest_mode_command_rate_is_pulse_rate(void)
{
    StepperDriver::MicrostepMotionPlan expected = {};
    expected.goalMicrosteps = 256;
    expected.pulses = 256;
    expected.pulseRateHz = 51200;
    expected.durationUs = 5000ULL;
    expected.microstepUnitsPerPulse = 1;

    const StepperDriver::MicrostepMotionPlan actual = StepperDriver::planAbsoluteMoveMicrosteps(0, 256, 51200, 256);

    assert_microstep_motion_plan_equal(expected, actual, "finest microstep absolute plan");
}

void test_microstep_coarse_microstep_preserves_fractional_position_until_motion(void)
{
    StepperDriver::MicrostepMotionPlan expected = {};
    expected.goalMicrosteps = 384;
    expected.pulses = 1;
    expected.pulseRateHz = 100;
    expected.durationUs = 10000ULL;
    expected.microstepUnitsPerPulse = 256;

    const StepperDriver::MicrostepMotionPlan actual = StepperDriver::planRelativeMoveMicrosteps(128, 256, 100, 1);

    assert_microstep_motion_plan_equal(expected, actual, "coarse microstep carry-forward plan");
}

void test_microstep_run_end_position_applies_pulse_size(void)
{
    assert_int64_equal(384, StepperDriver::computeRunEndPositionMicrosteps(128, true, 1, 256), "forward run end");
    assert_int64_equal(-128, StepperDriver::computeRunEndPositionMicrosteps(128, false, 1, 256), "reverse run end");
}

void test_microstep_normalization(void)
{
    TEST_ASSERT_EQUAL_UINT16(1, StepperDriver::normalizeMicrostepsPerFullStep(0));
    TEST_ASSERT_EQUAL_UINT16(256, StepperDriver::normalizeMicrostepsPerFullStep(256));
    TEST_ASSERT_EQUAL_UINT16(1, StepperDriver::normalizeMicrostepsPerFullStep(3));
    assert_int64_equal(1, StepperDriver::microstepUnitsPerPulse(256), "finest microstep units per pulse");
    assert_int64_equal(256, StepperDriver::microstepUnitsPerPulse(1), "full-step units per pulse");
}

void test_plan_relative_move_handles_negative_delta(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(200, -50, 25);
    TEST_ASSERT_EQUAL_INT32(150, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(50, plan.steps);
    assert_uint64_equal(2000000ULL, plan.durationUs, "negative delta duration");
}

void test_plan_absolute_move_uses_distance_for_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planAbsoluteMove(231, 100, 100);
    TEST_ASSERT_EQUAL_INT32(100, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(131, plan.steps);
    assert_uint64_equal(1310000ULL, plan.durationUs, "absolute move duration");
}

void test_plan_move_with_zero_rate_has_zero_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(10, 40, 0);
    TEST_ASSERT_EQUAL_INT32(50, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(40, plan.steps);
    assert_uint64_equal(0, plan.durationUs, "zero-rate move is immediate");
}

void test_plan_move_with_zero_steps_has_zero_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(42, 0, 100);
    TEST_ASSERT_EQUAL_INT32(42, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(0, plan.steps);
    assert_uint64_equal(0, plan.durationUs, "zero-step duration");
}

void test_plan_relative_move_matches_absolute_path(void)
{
    const int32_t current = 150;
    const int32_t delta = -75;
    const int32_t target = current + delta;
    const uint16_t rate = 50;

    const StepperDriver::MotionPlan relativePlan = StepperDriver::planRelativeMove(current, delta, rate);
    const StepperDriver::MotionPlan absolutePlan = StepperDriver::planAbsoluteMove(current, target, rate);

    assert_motion_plan_equal(absolutePlan, relativePlan, "relative plan should match absolute target path");
}

void test_plan_relative_zero_delta_matches_absolute_path(void)
{
    const int32_t current = 42;
    const int32_t delta = 0;
    const int32_t target = current + delta;
    const uint16_t rate = 100;

    const StepperDriver::MotionPlan relativePlan = StepperDriver::planRelativeMove(current, delta, rate);
    const StepperDriver::MotionPlan absolutePlan = StepperDriver::planAbsoluteMove(current, target, rate);

    assert_motion_plan_equal(absolutePlan, relativePlan, "zero-delta relative plan should match absolute path");
}

void test_plan_relative_zero_delta_zero_rate_stays_zero_duration(void)
{
    const int32_t current = -8;
    const int32_t delta = 0;
    const int32_t target = current + delta;
    const uint16_t rate = 0;

    const StepperDriver::MotionPlan relativePlan = StepperDriver::planRelativeMove(current, delta, rate);
    const StepperDriver::MotionPlan absolutePlan = StepperDriver::planAbsoluteMove(current, target, rate);

    assert_motion_plan_equal(absolutePlan, relativePlan, "zero-delta at zero-rate should stay zero-duration");
}

void test_plan_dwell_duration_uses_rate_per_second(void)
{
    const uint64_t duration = StepperDriver::planDwellDurationUs(5000, 1000);
    assert_uint64_equal(5000000ULL, duration, "dwell duration from cycles per second");
}

void test_plan_stop_duration_uses_unsigned_interval_product(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(10, 1000);
    assert_uint64_equal(10000ULL, duration, "Stop duration from interval");
}

void test_plan_stop_duration_uses_negative_wait_magnitude(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(-2, 1250);
    assert_uint64_equal(2500ULL, duration, "negative Stop wait count");
}

void test_plan_stop_duration_zero_interval_completes_immediately(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MAX, 0);
    assert_uint64_equal(0, duration, "zero Stop interval");
}

void test_plan_stop_duration_handles_positive_max(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MAX, UINT16_MAX);
    assert_uint64_equal(140735340806145ULL, duration, "positive max Stop duration");
}

void test_plan_stop_duration_handles_int32_min_magnitude(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MIN, UINT16_MAX);
    assert_uint64_equal(140735340871680ULL, duration, "INT32_MIN Stop duration");
}

void test_find_active_pulse_owner_returns_matching_motor_index(void)
{
    const uint32_t activeTokens[MAX_STEPPER_MOTORS] = {0, 42, 77};
    TEST_ASSERT_EQUAL_INT(1, StepperDriver::findActiveRunOwner(42, activeTokens, MAX_STEPPER_MOTORS));
    TEST_ASSERT_EQUAL_INT(2, StepperDriver::findActiveRunOwner(77, activeTokens, MAX_STEPPER_MOTORS));
}

void test_find_active_pulse_owner_rejects_zero_and_unknown_tokens(void)
{
    const uint32_t activeTokens[MAX_STEPPER_MOTORS] = {11, 0, 33};
    TEST_ASSERT_EQUAL_INT(-1, StepperDriver::findActiveRunOwner(0, activeTokens, MAX_STEPPER_MOTORS));
    TEST_ASSERT_EQUAL_INT(-1, StepperDriver::findActiveRunOwner(99, activeTokens, MAX_STEPPER_MOTORS));
}

void test_opcode_sequence_stale_matches_abort_watermark_contract(void)
{
    TEST_ASSERT_FALSE(StepperDriver::isCommandSequenceStale(0, 10));
    TEST_ASSERT_FALSE(StepperDriver::isCommandSequenceStale(11, 10));
    TEST_ASSERT_TRUE(StepperDriver::isCommandSequenceStale(10, 10));
    TEST_ASSERT_TRUE(StepperDriver::isCommandSequenceStale(9, 10));
}

void test_compute_pulse_end_step_applies_direction_per_motor(void)
{
    TEST_ASSERT_EQUAL_INT32(150, StepperDriver::computeRunEndStep(100, true, 50));
    TEST_ASSERT_EQUAL_INT32(50, StepperDriver::computeRunEndStep(100, false, 50));
}

void test_plan_dwell_duration_handles_negative_wait_cycles(void)
{
    const uint64_t duration = StepperDriver::planDwellDurationUs(-2, 1250);
    assert_uint64_equal(1600ULL, duration, "negative dwell cycles");
}

void test_plan_dwell_duration_matches_hz_example(void)
{
    const uint64_t duration = StepperDriver::planDwellDurationUs(10, 1000);
    assert_uint64_equal(10000ULL, duration, "10 cycles at 1000Hz should be 10ms");
}

void test_endstop_policy_blocks_motion_commands_when_tripped(void)
{
    TEST_ASSERT_TRUE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Move, true));
    TEST_ASSERT_TRUE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Goto, true));
}

void test_endstop_policy_allows_dwell_commands_when_tripped(void)
{
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Stop, true));
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Sleep, true));
}

void test_endstop_policy_allows_commands_when_not_tripped(void)
{
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Move, false));
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Goto, false));
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Stop, false));
    TEST_ASSERT_FALSE(StepperDriver::shouldRejectForEndstop(MotorOpcode::Sleep, false));
}

void test_endstop_mapping_only_motor_one_exposes_stepper_endstops(void)
{
    TEST_ASSERT_FALSE(StepperDriver::getInstance()->isEndstopTripped(2));
    TEST_ASSERT_FALSE(StepperDriver::getInstance()->getEndstopTrippedPinSetting(2));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, StepperDriver::getInstance()->setEndstopTrippedPinSetting(true, 2));
}

void test_endstop_legacy_and_motor_one_setting_paths_match(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, StepperDriver::getInstance()->setEndstopTrippedPinSetting(false, 1));
    TEST_ASSERT_FALSE(StepperDriver::getInstance()->getEndstopTrippedPinSetting(1));
    TEST_ASSERT_FALSE(StepperDriver::getInstance()->getEndstopTrippedPinSetting());

    TEST_ASSERT_EQUAL(ESP_OK, StepperDriver::getInstance()->setEndstopTrippedPinSetting(true));
    TEST_ASSERT_TRUE(StepperDriver::getInstance()->getEndstopTrippedPinSetting(1));
    TEST_ASSERT_TRUE(StepperDriver::getInstance()->getEndstopTrippedPinSetting());
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_plan_relative_move_uses_delta_for_duration);
    RUN_TEST(test_stepper_command_units_scale_to_internal_microsteps);
    RUN_TEST(test_microstep_absolute_move_uses_internal_full_step_units);
    RUN_TEST(test_microstep_finest_mode_command_rate_is_pulse_rate);
    RUN_TEST(test_microstep_coarse_microstep_preserves_fractional_position_until_motion);
    RUN_TEST(test_microstep_run_end_position_applies_pulse_size);
    RUN_TEST(test_microstep_normalization);
    RUN_TEST(test_plan_relative_move_handles_negative_delta);
    RUN_TEST(test_plan_absolute_move_uses_distance_for_duration);
    RUN_TEST(test_plan_move_with_zero_rate_has_zero_duration);
    RUN_TEST(test_plan_move_with_zero_steps_has_zero_duration);
    RUN_TEST(test_plan_relative_move_matches_absolute_path);
    RUN_TEST(test_plan_relative_zero_delta_matches_absolute_path);
    RUN_TEST(test_plan_relative_zero_delta_zero_rate_stays_zero_duration);
    RUN_TEST(test_plan_dwell_duration_uses_rate_per_second);
    RUN_TEST(test_plan_stop_duration_uses_unsigned_interval_product);
    RUN_TEST(test_plan_stop_duration_uses_negative_wait_magnitude);
    RUN_TEST(test_plan_stop_duration_zero_interval_completes_immediately);
    RUN_TEST(test_plan_stop_duration_handles_positive_max);
    RUN_TEST(test_plan_stop_duration_handles_int32_min_magnitude);
    RUN_TEST(test_find_active_pulse_owner_returns_matching_motor_index);
    RUN_TEST(test_find_active_pulse_owner_rejects_zero_and_unknown_tokens);
    RUN_TEST(test_opcode_sequence_stale_matches_abort_watermark_contract);
    RUN_TEST(test_compute_pulse_end_step_applies_direction_per_motor);
    RUN_TEST(test_plan_dwell_duration_handles_negative_wait_cycles);
    RUN_TEST(test_plan_dwell_duration_matches_hz_example);
    RUN_TEST(test_endstop_policy_blocks_motion_commands_when_tripped);
    RUN_TEST(test_endstop_policy_allows_dwell_commands_when_tripped);
    RUN_TEST(test_endstop_policy_allows_commands_when_not_tripped);
    RUN_TEST(test_endstop_mapping_only_motor_one_exposes_stepper_endstops);
    RUN_TEST(test_endstop_legacy_and_motor_one_setting_paths_match);
    UNITY_END();
}
