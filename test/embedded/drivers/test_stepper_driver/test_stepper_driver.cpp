// Includes for unit test framework
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Includes for this unit test
#include "PulseEngine.h"
#include "StepperDriver.h"

#define UNITTEST

int motorsControlled = 1;
bool gDriverStarted = false;

static void ensure_driver_started(void)
{
    if (gDriverStarted) return;

    StepperDriver::getInstance()->isrStartIoDriver();
    vTaskDelay(pdMS_TO_TICKS(1000));
    gDriverStarted = true;
}

void setUp(void) {
    TEST_ASSERT_EQUAL(ESP_OK, StepperDriver::getInstance()->setEndstopTrippedPinSetting(false));
    ensure_driver_started();
}

void tearDown(void) {
    // clean stuff up here
}

void test_stepper_singleton() {
    StepperDriver *driver = StepperDriver::getInstance();

    TEST_ASSERT_EQUAL(driver, StepperDriver::getInstance());

}

void test_stepper_inactive_on_init() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        TEST_ASSERT_EQUAL(false, StepperDriver::getInstance()->isMotorRunning(i));
    }
}

void test_stepper_motorGoTo() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorGoTo((uint32_t)10,0x0010,i);
        TEST_ASSERT_EQUAL(true, StepperDriver::getInstance()->isMotorRunning(i));
        StepperDriver::getInstance()->abortCommand(i);

    }        
}

void test_stepper_motorGoTo_wait() {
    // ESP32PWM attachPin causes Unit Tests to hang.
    // Call to ESP32PWM attachPin disabled in unit test.
    // Unable to test this type of functionality in unit test.
    /* for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorGoTo(10,100,i);
        TEST_ASSERT_EQUAL(true, StepperDriver::getInstance()->isMotorRunning(i));
    }    

    sleep(5);    
    
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        TEST_ASSERT_EQUAL(false, StepperDriver::getInstance()->isMotorRunning(i));
    }    */
}

void test_stepper_motorMove() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorMove(-100,1,i);
        TEST_ASSERT_EQUAL(true, StepperDriver::getInstance()->isMotorRunning(i));
        StepperDriver::getInstance()->abortCommand(i);
    }         
}

void test_stepper_motorStop() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorStop(100,10,i);
        TEST_ASSERT_EQUAL(true, StepperDriver::getInstance()->isMotorRunning(i));
        StepperDriver::getInstance()->abortCommand(i);
    }         
}

void test_stepper_motorSleep() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorStop(100,10,i);
        TEST_ASSERT_EQUAL(true, StepperDriver::getInstance()->isMotorRunning(i));
        StepperDriver::getInstance()->abortCommand(i);
    }      
}

void test_stepper_abortCommand() {
    for (size_t i = 1; i <= motorsControlled; i++)
    {
        StepperDriver::getInstance()->motorStop(100,10,i);
        StepperDriver::getInstance()->abortCommand(i);
        TEST_ASSERT_EQUAL(false, StepperDriver::getInstance()->isMotorRunning(i));
    }     
}

void test_stepper_abortCommand_stops_active_pulse_without_opcode_context() {
    StepperDriver *driver = StepperDriver::getInstance();

    driver->motorGoTo(1000, 400, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    TEST_ASSERT_TRUE(PulseEngine::isRunning());

    driver->abortCommand(1);
    vTaskDelay(pdMS_TO_TICKS(10));
    TEST_ASSERT_FALSE(PulseEngine::isRunning());
    TEST_ASSERT_EQUAL(false, driver->isMotorRunning(1));
}

void test_endstop_init_cleared() {
    TEST_ASSERT_EQUAL(false, StepperDriver::getInstance()->isEndstopTripped());
}


extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    RUN_TEST(test_stepper_singleton);
    RUN_TEST(test_stepper_inactive_on_init);
    RUN_TEST(test_endstop_init_cleared);
    RUN_TEST(test_stepper_motorGoTo);
    RUN_TEST(test_stepper_motorMove);
    RUN_TEST(test_stepper_motorStop);
    RUN_TEST(test_stepper_motorSleep);
    RUN_TEST(test_stepper_abortCommand);
    RUN_TEST(test_stepper_abortCommand_stops_active_pulse_without_opcode_context);
    //RUN_TEST(test_stepper_motorGoTo_wait);

    UNITY_END(); // stop unit testing
}
