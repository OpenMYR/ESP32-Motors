// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
#include <FS.h>
#include <WiFi.h>

// Includes for this unit test
#include "Op.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void empty_op_constructor_should_make_blank_op(void)
{
    uint8_t data[11];
    Op opTest(data);

    //TEST_ASSERT_
}

void uint8_pointer_op_constructor_should_make_valid_op(void)
{
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;
}

void uint_8_pointer_and_uint_32_op_constructor_should_make_valid_op_with_ip_address(void)
{

}

void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();
    RUN_TEST(empty_op_constructor_should_make_blank_op);
    RUN_TEST(uint8_pointer_op_constructor_should_make_valid_op);
    RUN_TEST(uint_8_pointer_and_uint_32_op_constructor_should_make_valid_op_with_ip_address);

    UNITY_END(); // stop unit testing
}

void loop()
{
}