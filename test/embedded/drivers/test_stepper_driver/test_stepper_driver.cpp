// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
//#include <FS.h>
//#include <WiFi.h>
#include <ESP32Servo.h>

// Includes for this unit test
#include "StepperDriver.h"
#include "WifiController.h"  

#define UNITTEST

int motorsControlled = 1;

void setUp(void) {
    // set stuff up here
    // If pins do not have pull up set to true.
    // ESP32 Devkit: set to true
    // OpenMYR Stepper: set to false
    StepperDriver::getInstance()->setEndstopTrippedPinSetting(true);
}

void tearDown(void) {
    // clean stuff up here
}

void test_stepper_singleton() {
    StepperDriver *driver = StepperDriver::getInstance();

    TEST_ASSERT_EQUAL(driver, StepperDriver::getInstance());

}

void test_stepper_inactive_on_init() {
    StepperDriver::getInstance()->isrStartIoDriver();
    sleep(1);
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

void test_endstop_init_cleared() {
    TEST_ASSERT_EQUAL(false, StepperDriver::getInstance()->isEndstopTripped());
}


void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();
    RUN_TEST(test_stepper_singleton);
    RUN_TEST(test_stepper_inactive_on_init);
    RUN_TEST(test_endstop_init_cleared);
    RUN_TEST(test_stepper_motorGoTo);
    RUN_TEST(test_stepper_motorMove);
    RUN_TEST(test_stepper_motorStop);
    RUN_TEST(test_stepper_motorSleep);
    RUN_TEST(test_stepper_abortCommand);
    //RUN_TEST(test_stepper_motorGoTo_wait);

    UNITY_END(); // stop unit testing
}

void loop()
{
}