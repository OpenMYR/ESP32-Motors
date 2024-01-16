// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
#include <FS.h>
#include <WiFi.h>

// Includes for this unit test
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * Intent of these test are not to fully test FreeRTOS task functionality.
 *   + Test we can setup a queue. Ensure high level behavior has not changed.
 *     For example the include path has changed overtime 
 */

TaskHandle_t _task = NULL;
uint8_t _core = 1;
uint8_t _queueSize = 0;
uint32_t _queueMaxTestWaitTicks = 20; 
bool _localTestVar;

void setUp(void) {
    // set stuff up here
    _queueSize = 32;
    _localTestVar = false;

}

void tearDown(void) {
    // clean stuff up here
}

static void _taskFunc(void *);

static void _taskFunc(void *){
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        _localTestVar = true;
    }
    vTaskDelete(NULL);
}

void setup_new_pinnedTask(void){

    TEST_ASSERT_NULL(_task);
    xTaskCreatePinnedToCore(
        _taskFunc,
        "UnitTestTask",
        1000,
        (void *)1,
        0,
        &_task,
        _core);

    TEST_ASSERT_NOT_NULL(_task);

    vTaskDelete(_task);

    TEST_ASSERT_TRUE(_localTestVar);
}


void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(setup_new_pinnedTask);


    UNITY_END(); // stop unit testing
}

void loop()
{
}