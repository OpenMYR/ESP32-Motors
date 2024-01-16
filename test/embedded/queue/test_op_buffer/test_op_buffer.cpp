// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
#include <FS.h>
#include <WiFi.h>

// Includes for this unit test
#include "OpBuffer.h"

OpBuffer* _opBuffer = NULL;
u_int8_t _bufferSize;
u_int8_t _bufferCount;

void setUp(void) {
    // set stuff up here
    _bufferSize = OP_BUFFER_SIZE;
    _bufferCount = OP_BUFFER_COUNT;
}

void tearDown(void) {
    // clean stuff up here
}

void setup_op_buffer(void){
    TEST_ASSERT_NULL(_opBuffer);

    _opBuffer = OpBuffer::getInstance();

    TEST_ASSERT_NOT_NULL(_opBuffer);
}

void is_new_buffer_empty(void){
    for (size_t i = 0; i < _bufferCount; i++)
    {
        TEST_ASSERT_EQUAL(true, _opBuffer->isEmpty(i));
    }
}

void buffer_stores_op(void){
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;

    bool failing = false;
    
    for (size_t i = 0; i < _bufferCount; i++)
    {
        opIn.motorID = i;
        if(_opBuffer->storeOp(&opIn) != 0)
        {
            failing |= true;
        }  
    }
    TEST_ASSERT_FALSE_MESSAGE(failing, "storeOp error");

    failing = false;

    for (size_t i = 0; i < _bufferCount; i++)
    {
        if(_opBuffer->isEmpty(i) == true)
        {
            failing |= true;
        }  
    }
    TEST_ASSERT_FALSE_MESSAGE(failing, "Op missing from queue");
}

void buffer_gives_stored_op(void){

    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    Op* opOut;
    opIn.motorID = 1;

    _opBuffer->storeOp(&opIn);
    opIn.opcode = 'G'; 
    opOut = _opBuffer->getOp(1);

    TEST_ASSERT_EQUAL_UINT8('S', opOut->opcode);
}

void buffer_item_is_not_pointer_to_input(void){
    
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 1;
    _opBuffer->storeOp(&opIn);

    TEST_ASSERT_FALSE(&opIn == _opBuffer->getOp(1));
}

void add_buffer_pop_buffer(void){
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;

    bool failing = false;

    TEST_ASSERT_TRUE(_opBuffer->storeOp(&opIn)>=0);
    TEST_ASSERT_NOT_NULL(_opBuffer->getOp(0));
    TEST_ASSERT_TRUE(_opBuffer->isEmpty(0));
}

void pop_from_empty_buffer(void){
    
    _opBuffer->clear(1);
    TEST_ASSERT_NULL(_opBuffer->getOp(1));
}

void overfill_buffer(void){
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;

    bool failing = false;

    for (size_t i = 0; i < _bufferSize; i++)
    {
        if(_opBuffer->storeOp(&opIn) != 0)
        {
            failing |= true;
        } 
    }
    TEST_ASSERT_FALSE_MESSAGE(failing, "Unable to fill buffer");
    
    TEST_ASSERT_TRUE(_opBuffer->storeOp(&opIn) < 0);
}

void clear_buffer(void){
    
    bool failing = false;
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;

    
    for (size_t i = 0; i < _bufferCount; i++)
    {
        opIn.motorID = i;
        _opBuffer->storeOp(&opIn); 
    }

    for (size_t i = 0; i < _bufferCount; i++)
    {
        _opBuffer->clear(i);
        if(_opBuffer->isEmpty(i) != true)
        {
            failing |= true;
        }  
    }

    TEST_ASSERT_FALSE_MESSAGE(failing, "Buffer not cleared");
}

void reset_opBuffer(void){
        
    bool failing = false;
    uint8_t data[11] = {0x00, 0x00, 'S', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Op opIn(data);
    opIn.motorID = 0;

    for (size_t i = 0; i < _bufferCount; i++)
    {
        opIn.motorID = i;
        _opBuffer->storeOp(&opIn); 
    }

    _opBuffer->reset();
    
    for (size_t i = 0; i < _bufferCount; i++)
    {
        if(_opBuffer->isEmpty(i) != true)
        {
            failing |= true;
        }  
    }
    TEST_ASSERT_FALSE_MESSAGE(failing, "Buffers not reset");
}

void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();
    RUN_TEST(setup_op_buffer);
    RUN_TEST(is_new_buffer_empty);
    RUN_TEST(buffer_stores_op);
    RUN_TEST(buffer_gives_stored_op);
    RUN_TEST(buffer_item_is_not_pointer_to_input);
    RUN_TEST(reset_opBuffer);
    RUN_TEST(add_buffer_pop_buffer);
    RUN_TEST(clear_buffer);
    RUN_TEST(pop_from_empty_buffer);
    RUN_TEST(overfill_buffer);
    RUN_TEST(clear_buffer);

    UNITY_END(); // stop unit testing
}

void loop()
{
}