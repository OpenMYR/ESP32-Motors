#include <unity.h>

#include "../../../../lib/protocols/UdpPromptScheduler.h"

// Native env builds tests only, so include the implementation directly.
#include "../../../../lib/protocols/UdpPromptScheduler.cpp"

void test_computeNextIntervalUs_uses_warmup_window_limits(void)
{
    TEST_ASSERT_EQUAL_INT64(800000, UdpPromptScheduler::computeNextIntervalUs(0, -999999));
    TEST_ASSERT_EQUAL_INT64(1000000, UdpPromptScheduler::computeNextIntervalUs(10, 0));
    TEST_ASSERT_EQUAL_INT64(1200000, UdpPromptScheduler::computeNextIntervalUs(29999999, 999999));
}

void test_computeNextIntervalUs_switches_to_steady_state_limits_after_warmup(void)
{
    TEST_ASSERT_EQUAL_INT64(8000000, UdpPromptScheduler::computeNextIntervalUs(30000000, -9999999));
    TEST_ASSERT_EQUAL_INT64(10000000, UdpPromptScheduler::computeNextIntervalUs(45000000, 0));
    TEST_ASSERT_EQUAL_INT64(12000000, UdpPromptScheduler::computeNextIntervalUs(90000000, 9999999));
}

void test_computeNextIntervalUs_clamps_excess_jitter_to_active_window(void)
{
    TEST_ASSERT_EQUAL_INT64(800000, UdpPromptScheduler::computeNextIntervalUs(0, -5000000));
    TEST_ASSERT_EQUAL_INT64(12000000, UdpPromptScheduler::computeNextIntervalUs(60000000, 9000000));
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_computeNextIntervalUs_uses_warmup_window_limits);
    RUN_TEST(test_computeNextIntervalUs_switches_to_steady_state_limits_after_warmup);
    RUN_TEST(test_computeNextIntervalUs_clamps_excess_jitter_to_active_window);
    return UNITY_END();
}
