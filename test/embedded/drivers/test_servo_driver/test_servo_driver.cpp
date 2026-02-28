// Includes for unit test framework
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Includes for this unit test
#include "ServoDriver.h"

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
        ServoDriver::getInstance()->motorGoTo(180,1,i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);

    }        
}

void test_servo_motorGoTo_wait() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorGoTo(10,100,i);
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
        ServoDriver::getInstance()->motorMove(-100,1,i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);
    }         
}

void test_servo_motorStop() {
    for (size_t i = 1; i <= MAX_MOTORS; i++)
    {
        ServoDriver::getInstance()->motorStop(100,10,i);
        TEST_ASSERT_EQUAL(true, ServoDriver::getInstance()->isMotorRunning(i));
        ServoDriver::getInstance()->abortCommand(i);
    }         
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
    RUN_TEST(test_servo_motorStop);
    RUN_TEST(test_servo_motorSleep);
    RUN_TEST(test_servo_abortCommand);
    RUN_TEST(test_servo_motorGoTo_wait);

    UNITY_END(); // stop unit testing
}
