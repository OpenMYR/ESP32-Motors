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

    esp_err_t err;
    err = WifiController::init();
    TEST_ASSERT_TRUE(err == false);
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