#include <unity.h>

#include "StepperDriver.h"

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
    TEST_ASSERT_EQUAL_UINT64(1000000ULL, plan.durationUs);
}

void test_plan_relative_move_handles_negative_delta(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(200, -50, 25);
    TEST_ASSERT_EQUAL_INT32(150, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(50, plan.steps);
    TEST_ASSERT_EQUAL_UINT64(2000000ULL, plan.durationUs);
}

void test_plan_absolute_move_uses_distance_for_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planAbsoluteMove(231, 100, 100);
    TEST_ASSERT_EQUAL_INT32(100, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(131, plan.steps);
    TEST_ASSERT_EQUAL_UINT64(1310000ULL, plan.durationUs);
}

void test_plan_move_with_zero_rate_has_unknown_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(10, 40, 0);
    TEST_ASSERT_EQUAL_INT32(50, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(40, plan.steps);
    TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, plan.durationUs);
}

void test_plan_move_with_zero_steps_has_zero_duration(void)
{
    const StepperDriver::MotionPlan plan = StepperDriver::planRelativeMove(42, 0, 100);
    TEST_ASSERT_EQUAL_INT32(42, plan.goalStep);
    TEST_ASSERT_EQUAL_UINT32(0, plan.steps);
    TEST_ASSERT_EQUAL_UINT64(0, plan.durationUs);
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_plan_relative_move_uses_delta_for_duration);
    RUN_TEST(test_plan_relative_move_handles_negative_delta);
    RUN_TEST(test_plan_absolute_move_uses_distance_for_duration);
    RUN_TEST(test_plan_move_with_zero_rate_has_unknown_duration);
    RUN_TEST(test_plan_move_with_zero_steps_has_zero_duration);
    UNITY_END();
}
