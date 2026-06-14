// Includes for unit test framework
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Includes for this unit test
#include "ServoDriver.h"

static void assert_uint64_equal(uint64_t expected, uint64_t actual, const char *message)
{
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected >> 32), static_cast<uint32_t>(actual >> 32), message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual), message);
}

void setUp(void) {
    // set stuff up here

}

void tearDown(void) {
    // clean stuff up here
}

void test_servo_singleton() {
    ServoDriver *driver = ServoDriver::getInstance();

    TEST_ASSERT_EQUAL(driver, ServoDriver::getInstance());

}

void test_servo_inactive_on_init() {
    ServoDriver::getInstance()->isrStartIoDriver();
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        TEST_ASSERT_EQUAL(false, ServoDriver::getInstance()->isMotorRunning(i));
    }
}

void test_servo_motorGoTo() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorGoTo(180000, 50000, i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);

    }        
}

void test_servo_motorGoTo_wait() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorGoTo(10000, 50000, i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
    }    

    vTaskDelay(pdMS_TO_TICKS(5000));
    
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        TEST_ASSERT_EQUAL(false, ServoDriver::getInstance()->isMotorRunning(i));
    }   
}

void test_servo_motorMove() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorMove(-100000, 50000, i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);
    }         
}

void test_servo_motorMove_negative_delta_finishes_after_expected_duration() {
    ServoDriver *driver = ServoDriver::getInstance();

    driver->abortCommand(1);
    driver->motorMove(-10000, 50000, 1);
    TEST_ASSERT_EQUAL(true, driver->isMotorRunning(1));

    vTaskDelay(pdMS_TO_TICKS(250));

    TEST_ASSERT_EQUAL(false, driver->isMotorRunning(1));
}

void test_servo_zero_rate_repositions_immediately() {
    ServoDriver *driver = ServoDriver::getInstance();

    driver->abortCommand(1);
    driver->motorGoTo(90000, 0, 1);
    TEST_ASSERT_EQUAL(false, driver->isMotorRunning(1));

    driver->motorMove(10000, 0, 1);
    TEST_ASSERT_EQUAL(false, driver->isMotorRunning(1));
}

void test_servo_motorStop() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorStop(100,10,i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);
    }         
}

void test_servo_plan_stop_duration_uses_unsigned_interval_product(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(10, 1000);
    assert_uint64_equal(10000ULL, duration, "servo Stop duration from interval");
}

void test_servo_plan_stop_duration_uses_negative_wait_magnitude(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(-2, 1250);
    assert_uint64_equal(2500ULL, duration, "servo negative Stop wait count");
}

void test_servo_plan_stop_duration_zero_interval_completes_immediately(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MAX, 0);
    assert_uint64_equal(0, duration, "servo zero Stop interval");
}

void test_servo_plan_stop_duration_handles_positive_max(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MAX, UINT16_MAX);
    assert_uint64_equal(140735340806145ULL, duration, "servo positive max Stop duration");
}

void test_servo_plan_stop_duration_handles_int32_min_magnitude(void)
{
    const uint64_t duration = MotorDriver::planStopDurationUs(INT32_MIN, UINT16_MAX);
    assert_uint64_equal(140735340871680ULL, duration, "servo INT32_MIN Stop duration");
}

void test_servo_motorSleep() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorStop(100,10,i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);
    }      
}

void test_servo_abortCommand() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorStop(100,10,i);
        ServoDriver::getInstance()->abortCommand(i);
        TEST_ASSERT_EQUAL(false, ServoDriver::getInstance()->isMotorRunning(i));
    }     
}


extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    RUN_TEST(test_servo_singleton);
    RUN_TEST(test_servo_inactive_on_init);
    RUN_TEST(test_servo_motorGoTo);
    RUN_TEST(test_servo_motorMove);
    RUN_TEST(test_servo_motorMove_negative_delta_finishes_after_expected_duration);
    RUN_TEST(test_servo_zero_rate_repositions_immediately);
    RUN_TEST(test_servo_motorStop);
    RUN_TEST(test_servo_plan_stop_duration_uses_unsigned_interval_product);
    RUN_TEST(test_servo_plan_stop_duration_uses_negative_wait_magnitude);
    RUN_TEST(test_servo_plan_stop_duration_zero_interval_completes_immediately);
    RUN_TEST(test_servo_plan_stop_duration_handles_positive_max);
    RUN_TEST(test_servo_plan_stop_duration_handles_int32_min_magnitude);
    RUN_TEST(test_servo_motorSleep);
    RUN_TEST(test_servo_abortCommand);
    RUN_TEST(test_servo_motorGoTo_wait);

    UNITY_END(); // stop unit testing
}
