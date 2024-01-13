// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
#include <FS.h>
#include <WiFi.h>

// Includes for this unit test
#include <freertos/FreeRTOS.h>

/*
 * Intent of these test are not to fully test FreeRTOS queue functionality.
 *   + Test we can setup a queue. Ensure high level behavior has not changed.
 *     For example the include path has changed overtime 
 */

xQueueHandle _queue;
uint8_t _queueSize = 0;
uint32_t _queueMaxTestWaitTicks = 20; 

void setUp(void) {
    // set stuff up here
    _queueSize = 32;
}

void tearDown(void) {
    // clean stuff up here
}

void setup_new_queue(void){

    TEST_ASSERT_NULL(_queue);
    if(!_queue){
        _queue = xQueueCreate(_queueSize, sizeof(bool));
    }
    TEST_ASSERT_NOT_NULL(_queue);
}

void enqueue_dequeue(void){

    bool sendBool = true;
    bool recvBool = false;

    TEST_ASSERT_NOT_NULL(_queue);

    TEST_ASSERT_EQUAL(pdPASS, xQueueSend(_queue, &sendBool, _queueMaxTestWaitTicks));

    TEST_ASSERT_EQUAL(pdPASS, xQueueReceive(_queue, &recvBool, _queueMaxTestWaitTicks));
    TEST_ASSERT_EQUAL(sendBool,recvBool);

}

void dequeue_from_empty_queue(void){
    bool recvBool;

    TEST_ASSERT_EQUAL(errQUEUE_EMPTY, xQueueReceive(_queue, &recvBool, 10));
}

void overfill_queue(void){
    bool sendBool = true;
    bool recvBool = false;

    // Fill the queue
    for (size_t i = 0; i < _queueSize; i++)
    {
        TEST_ASSERT_EQUAL(pdPASS, xQueueSend(_queue, &sendBool, _queueMaxTestWaitTicks));
    }

    // Try to overfill queue
    TEST_ASSERT_EQUAL(errQUEUE_FULL, xQueueSend(_queue, &sendBool, _queueMaxTestWaitTicks));

}

void clear_queue(void){
    bool recvBool = false;
    
    TEST_ASSERT_EQUAL(pdPASS, xQueueReset(_queue));

    TEST_ASSERT_EQUAL(errQUEUE_EMPTY, xQueueReceive(_queue, &recvBool, 10));

}

void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(setup_new_queue);
    RUN_TEST(enqueue_dequeue);
    RUN_TEST(dequeue_from_empty_queue);
    RUN_TEST(overfill_queue);
    RUN_TEST(clear_queue);


    UNITY_END(); // stop unit testing
}

void loop()
{
}