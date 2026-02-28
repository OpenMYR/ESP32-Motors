// Includes for unit test framework
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    Op opTest;
    TEST_ASSERT_EQUAL_UINT32(0, opTest.opSeq);
}

void uint8_pointer_op_constructor_should_make_valid_op(void)
{
    uint8_t data[11] = {0x12, 0x34, 'S', 0x02, 0x00, 0x00, 0x01, 0x2C, 0x03, 0xE8, 0x05};
    Op opIn(data);
    TEST_ASSERT_EQUAL_UINT16(0x1234, opIn.port);
    TEST_ASSERT_EQUAL_CHAR('S', opIn.opcode);
    TEST_ASSERT_EQUAL_UINT8(0x02, opIn.queue);
    TEST_ASSERT_EQUAL_INT32(300, opIn.stepNum);
    TEST_ASSERT_EQUAL_UINT16(1000, opIn.stepRate);
    TEST_ASSERT_EQUAL_UINT8(5, opIn.motorID);
    TEST_ASSERT_EQUAL_UINT32(0, opIn.sourceIPAddr);
    TEST_ASSERT_EQUAL_UINT32(0, opIn.opSeq);
}

void uint_8_pointer_and_uint_32_op_constructor_should_make_valid_op_with_ip_address(void)
{
    uint8_t data[11] = {0x00, 0x2A, 'M', 0x01, 0xFF, 0xFF, 0xFF, 0x9C, 0x00, 0x64, 0x03};
    Op opIn(data, 0xC0A80132);
    TEST_ASSERT_EQUAL_UINT16(42, opIn.port);
    TEST_ASSERT_EQUAL_CHAR('M', opIn.opcode);
    TEST_ASSERT_EQUAL_UINT8(1, opIn.queue);
    TEST_ASSERT_EQUAL_INT32(-100, opIn.stepNum);
    TEST_ASSERT_EQUAL_UINT16(100, opIn.stepRate);
    TEST_ASSERT_EQUAL_UINT8(3, opIn.motorID);
    TEST_ASSERT_EQUAL_UINT32(0xC0A80132, opIn.sourceIPAddr);
    TEST_ASSERT_EQUAL_UINT32(0, opIn.opSeq);
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    RUN_TEST(empty_op_constructor_should_make_blank_op);
    RUN_TEST(uint8_pointer_op_constructor_should_make_valid_op);
    RUN_TEST(uint_8_pointer_and_uint_32_op_constructor_should_make_valid_op_with_ip_address);

    UNITY_END(); // stop unit testing
}
