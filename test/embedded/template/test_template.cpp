// Includes for unit test framework
#include <Arduino.h>
#include <unity.h>

// Includes for project libraries
#include <FS.h>
#include <WiFi.h>

// Includes for this unit test

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}


void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();
    //RUN_TEST();

    UNITY_END(); // stop unit testing
}

void loop()
{
}