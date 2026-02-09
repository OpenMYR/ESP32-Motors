#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "StepperDriver.h"

static void assert_uint64_equal(uint64_t expected, uint64_t actual, const char *message)
{
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected >> 32), static_cast<uint32_t>(actual >> 32), message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual), message);
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

void test_plan_move_with_zero_rate_has_unknown_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(10, 40, 0);
    TEST_ASSERT_EQUAL_INT32(50, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(40, plan.steps);
    assert_uint64_equal(UINT64_MAX, plan.durationUs, "zero-rate duration sentinel");
}

void test_plan_move_with_zero_steps_has_zero_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(42, 0, 100);
    TEST_ASSERT_EQUAL_INT32(42, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(0, plan.steps);
    assert_uint64_equal(0, plan.durationUs, "zero-step duration");
}

void test_plan_dwell_duration_uses_millisecond_precision(void)
{
    const uint64_t duration = StepperDriver::planDwellDurationUs(5, 200);
    assert_uint64_equal(1000000ULL, duration, "dwell duration from ms precision");
}

void test_plan_dwell_duration_handles_negative_wait_cycles(void)
{
    const uint64_t duration = StepperDriver::planDwellDurationUs(-2, 125);
    assert_uint64_equal(250000ULL, duration, "negative dwell cycles");
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_plan_relative_move_uses_delta_for_duration);
    RUN_TEST(test_plan_relative_move_handles_negative_delta);
    RUN_TEST(test_plan_absolute_move_uses_distance_for_duration);
    RUN_TEST(test_plan_move_with_zero_rate_has_unknown_duration);
    RUN_TEST(test_plan_move_with_zero_steps_has_zero_duration);
    RUN_TEST(test_plan_dwell_duration_uses_millisecond_precision);
    RUN_TEST(test_plan_dwell_duration_handles_negative_wait_cycles);
    UNITY_END();
}
