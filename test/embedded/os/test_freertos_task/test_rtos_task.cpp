// Includes for unit test framework
#include <unity.h>

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
bool _localTestVar;

void setUp(void) {
    // set stuff up here
    _localTestVar = false;

}

void tearDown(void) {
    // clean stuff up here
}

static void _taskFunc(void *);

static void _taskFunc(void *){
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
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
    vTaskDelay(pdMS_TO_TICKS(100));

    vTaskDelete(_task);

    TEST_ASSERT_TRUE(_localTestVar);
}


extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();

    RUN_TEST(setup_new_pinnedTask);


    UNITY_END(); // stop unit testing
}
