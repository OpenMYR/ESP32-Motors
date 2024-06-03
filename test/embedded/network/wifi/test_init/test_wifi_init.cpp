#include <Arduino.h>
#include <unity.h>

#include "WifiController.h"


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_wifi_init(void){

    TEST_ASSERT_EQUAL(ESP_OK, WifiController::init());
}

void setup()
{

    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(test_wifi_init);

    UNITY_END(); // stop unit testing
}

void loop()
{
}